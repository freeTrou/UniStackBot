#ifndef UNISTACKBOT_CONTROLLER__JOINT_STREAM_CONTROLLER_HPP_
#define UNISTACKBOT_CONTROLLER__JOINT_STREAM_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sp_latest/sp_latest.hpp"
#include "stale_watch/stale_watch.hpp"
#include "unistackbot_interface/joint_capacity.hpp"
#include "unistackbot_interface/msg/joint_command.hpp"

#include "ruckig/otg_stream.hpp"

namespace unistackbot_controller
{
/*
 * JointStreamController (2026-09-18): 关节级 topic 流式控制器 —— 取代 JTC action 层。
 *
 * 对外契约:
 *   ~/command  unistackbot_interface/JointCommand (reliable+KeepLast(1), 点流语义)
 *   插值可配置: interpolation = hold | ruckig
 *     hold   保持上一命令 (上层=500Hz 平滑流时的正解)
 *     ruckig OtgStream 点流整形 (慢上层 10-100Hz 的率失配填充, v/a/j 全受限)
 *   安全层 (与 CM 控制器同款): NaN 门 → 限位 clamp → 步长饱和 (URDF max_velocity/update_rate)
 *
 * 模式: 仿真链 (position 命令接口) 仅 CSP 生效; CSV/CST/MIT → Hold + 单次 WARN
 * (真机 MIT 硬件接口接入时启用)。
 *
 * 与 JTC 的关系: JTC 已从双链移除 (2026-09-18 裁决: action 层不必要, topic 性能优);
 * 轨迹插值/容差语义由 "上层流式 + 本控制器插值档" 承接。
 *
 * 关节集合: <ros2_control> 全部 position 命令关节 (含 gripper; mimic 关节归硬件层)。
 * 实例每线程一份; update 在 CM 的 RT 线程。
 */
class JointStreamController : public controller_interface::ControllerInterface
{
public:
	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(
		const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(
		const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_deactivate(
		const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_cleanup(
		const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
	const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
	const override;

	controller_interface::return_type update(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	enum class Interpolation : uint8_t
	{
		HOLD = 0,
		RUCKIG = 1,
	};

	// 命令消毒: NaN 门 → 限位 clamp; 返回 false = NaN (整拍保持)
	bool sanitize(std::vector<double> & q) const;

	// 值通道快照 (POD): 回调线程做名映射 → RT 侧零 string 零 vector。
	// JointCommand rosidl 类型非 POD, 不能直接进 SpLatest —— 快照即翻译层
	struct CmdSnapshot
	{
		double position[unistackbot_interface::kMaxJoints];
		uint64_t seq;
	};
	rclcpp::Subscription<unistackbot_interface::msg::JointCommand>::SharedPtr cmd_sub_;
	unistackbot_common::SpLatest<CmdSnapshot> rt_cmd_;
	uint64_t cmd_seq_{0};
	std::atomic<uint64_t> cmd_seq_pub_{0};   // 回调线程产序号 (唯一写者)

	// ---- 关节表 (on_configure 定容) ----
	std::vector<std::string> joint_names_;
	std::vector<double> q_min_, q_max_, vmax_;   // URDF ros2_control 限位 + 速度限
	std::vector<double> step_limits_;            // hold 档: 单拍步长上限 (max_velocity/hz)
	std::vector<double> cmd_;                    // 上一拍输出 (关节序 = 接口序)
	std::vector<double> target_;                 // 已采纳目标 (hold/ruckig 每周期朝它推进)

	// ---- 插值 ----
	Interpolation interpolation_{Interpolation::HOLD};
	unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints> otg_;
	double max_acceleration_{5.0};
	double max_jerk_{20.0};
	bool otg_ready_{false};

	// ---- StaleWatch 断流受控减速 (0c; 设计 §6.1: 过期→保持+受控减速, 与断流同路径) ----
	double stale_ms_cfg_{0.0};        // stale_timeout_ms 原始配置 (周期数首拍校准)
	double stale_decel_ms_cfg_{200.0};
	uint32_t stale_cycles_{0};        // 断流判定阈值 (周期数; 0=关闭)
	uint32_t stale_decel_cycles_{0};  // 刹停线性减速窗 (周期数)
	unistackbot_common::StaleWatch watch_;
	bool was_stale_{false};           // 边沿检测 (转换即日志, 无重复触发)
	std::vector<double> prev_cmd_;    // 周期首快照 (速度估计基准)
	std::vector<double> vel_;         // 上拍实际步长 (=关节速度估计)
	std::vector<double> decel_rate_;  // 断流进入时刻的每周期速度减量 (= vel/N)
	bool rate_calibrated_{false};     // update_rate 校准 (16 拍中位数; Humble 坑见 cpp)
	static constexpr uint32_t kPeriodSamples = 16u;
	double period_samples_[kPeriodSamples] = {0};
	uint32_t period_n_{0};

	// ---- 观测 ----
	uint64_t dropped_mode_{0};       // 非 CSP 模式拒绝计数
	uint64_t dropped_len_{0};        // 关节数不匹配拒绝计数
	uint64_t stale_events_{0};       // 断流事件计数 (观测; 进入即 +1)

};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__JOINT_STREAM_CONTROLLER_HPP_

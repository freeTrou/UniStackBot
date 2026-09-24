#ifndef UNISTACKBOT_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_
#define UNISTACKBOT_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <chrono>
#include <type_traits>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rt_tune/rt_tune.hpp"
#include "ruckig/otg_stream.hpp"
#include "sp_latest/sp_latest.hpp"
#include "stale_watch/stale_watch.hpp"
#include "unistackbot_interface/joint_capacity.hpp"
#include "unistackbot_interface/msg/cartesian_control.hpp"
#include "unistackbot_interface/msg/cartesian_motion_status.hpp"

#include "analytic_piper/analytic_piper.hpp"
#include "dls_ik/dls_ik.hpp"
#include "ik_solver/ik_solver.hpp"
#include "urdf_fk/urdf_fk.hpp"

// 算法库住 unistackbot_algorithm 包 (2026-09-17 拆分: 算法与控制器集成解耦);
// 控制器只消费类型, 命名空间仍属本包
using unistackbot_algorithm::CartesianPose;
using unistackbot_algorithm::DlsIk;
using unistackbot_algorithm::AnalyticPiper;
using unistackbot_algorithm::IkSolver;
using unistackbot_algorithm::DlsIkConfig;
using unistackbot_algorithm::DlsIkStats;
using unistackbot_algorithm::RedundancyPreference;
using unistackbot_algorithm::UrdfFk;

namespace unistackbot_controller
{

/*
 * 笛卡尔运动控制器 (P1.5; 2026-09-23 §16.6 语义统一改版) ——
 * 每条末端位姿消息 = 最新终点, 流式/终点是同一契约的频率档, 默认可打断
 * (OtgStream 每拍从当前状态重规划 C2 = 原生可打断语义)。
 *
 * 数据通路 (IK 出环, RT 只做整形):
 *   ~/target (点流) → 回调线程每条一次 IK (种子=实测, 墙钟预算 ik_timeout_ms)
 *   → SpLatest<SolvedTarget> → RT 环 OtgStream 追最新关节终点 (C2, vmax 界)
 *   → 安全钳位 (NaN 门 + 限位 + 步长, 皮带扣; OtgStream Limits 已主约束)
 *   → 接口输出
 * 旧 RT 内流式 IK + worker 冷启动机制整体退役 (500µs 预算难题随之消失);
 * 每条消息种子=实测 → 物理链下垂按消息率补偿 (闭环性保留)。
 *
 * 对外契约 (冻结于骨架期, 2026-09-17; §16.6 后语义不变):
 *   ~/target  geometry_msgs/PoseStamped   值通道 (base 系期望末端位姿, 点流)
 *   ~/control unistackbot_interface/CartesianControl   事件通道 (TRACKING/HOLD)
 *   ~/status  unistackbot_interface/CartesianMotionStatus   位姿反馈 + 求解器健康
 *   关节反馈不新增: /joint_states (joint_state_broadcaster) 是关节空间单一事实源
 *
 * 实例隔离 (KDL 暂存不跨线程, urdf_fk.hpp 用法契约): fk_ = RT 侧 (status 误差),
 * ik_ = 回调线程侧 (求解); init 在 configure 期单线程完成, 激活后两侧各用各的。
 */
class CartesianMotionController : public controller_interface::ControllerInterface
{
public:
	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	// 发布一次状态 (生命周期回调内调用; update 内的周期发布走 RealtimePublisher)
	void publishStatus(uint8_t mode);
	// RT 周期状态发布 (update 内, ~20Hz, tryPublish)
	void publishStatusRt(const rclcpp::Time & time, uint8_t mode,
		const CartesianPose & meas, bool meas_ok);
	// 激活末尾预热: 触达全部首触路径 (缺页/惰性绑定/线程私有 syscall), 见设计 §3
	void warmup();
	// WCET 记账 (每拍; 停用时终报 p50/p99/max)
	void recordWcet(const std::chrono::steady_clock::time_point & t0);
	// 输出安全钳位 (NaN 门 → 限位 clamp → 步长饱和; OtgStream Limits 之上的皮带扣)
	bool applySolution(const double * q);

	// ---- 参数 ----
	std::string base_link_, tip_link_;
	std::string seed_lib_package_, seed_lib_relpath_;
	double ik_timeout_ms_{0.0};          // 回调线程单次 IK 墙钟预算 (必填; 旧 RT 预算/冷启动预算合一)
	double max_acceleration_{0.0};       // OTG 关节加速度界 [rad/s²] (机型资产, 必填)
	double max_jerk_{0.0};               // [rad/s³] (必填)
	int degraded_n_{50};                 // 连续失败降级阈值 (status DEGRADED)
	double converge_pos_tol_{0.001};     // converged 判定: 位置容差 [m]
	double converge_rot_tol_{0.01};      // converged 判定: 姿态容差 [rad]

	// ---- 运动学 (fk_=RT 侧 status 误差; ik_=回调线程侧求解; 实例不跨线程) ----
	std::unique_ptr<UrdfFk> fk_;
	std::unique_ptr<IkSolver> ik_;
	std::string ik_solver_name_{"dls"};
	UrdfFk::Scratch fk_scratch_;

	// ---- 接口映射: 接口名序 (CM 声明序) != 链序 (fk.jointNames), 按名映射 ----
	std::vector<std::string> iface_joint_names_;   // state_interfaces_ 顺序的关节名
	std::vector<std::size_t> chain_from_iface_;    // [接口序] -> 链序下标

	// ---- RT 路径状态 (update 线程私有, 零周期分配) ----
	std::vector<double> cmd_;            // 上一拍命令 (链序)
	std::vector<double> q_meas_;         // 实测关节 (链序, 误差基准 + 回调种子源)
	bool has_target_{false};
	std::array<double, unistackbot_interface::kMaxJoints> goal_{};   // 最新成功解的关节终点 (OTG 追踪目标)
	bool have_goal_{false};
	CartesianPose target_pose_{};        // 当前采纳目标 (base 系; 来自解算载荷)
	uint64_t target_seq_{0};
	int consecutive_fail_{0};            // 连续求解失败计数 (回调按消息报, RT 采纳时记账)
	uint8_t last_result_{0};             // IkResult (status 透传)
	bool last_timed_out_{false};
	double last_min_sigma_{-1.0};
	std::string warned_frame_;           // 非本帧目标的单次警告去重 (回调线程私有)

	// ---- 跨线程通道 (POD; §16.6 改版核心) ----
	// 回调 → RT: 每条 ~/target 消息一次 IK 的结果 (关节终点 + 遥测)
	struct SolvedTarget
	{
		CartesianPose target;                       // 原始笛卡尔目标 (status 误差基准)
		double q[unistackbot_interface::kMaxJoints];
		uint32_t n;
		bool ok;
		uint8_t result;                             // IkResult
		bool timed_out;
		double min_sigma;
	};
	// RT → 回调: 最新实测关节 (IK 种子; 每拍覆盖写)
	struct MeasJoints
	{
		double q[unistackbot_interface::kMaxJoints];
		uint32_t n;
	};
	static_assert(std::is_trivially_copyable_v<SolvedTarget>, "SolvedTarget 须 POD (SpLatest 契约)");
	static_assert(std::is_trivially_copyable_v<MeasJoints>, "MeasJoints 须 POD (SpLatest 契约)");
	unistackbot_common::SpLatest<SolvedTarget> solved_ch_;
	unistackbot_common::SpLatest<MeasJoints> meas_ch_;
	std::vector<double> cb_seed_;         // 回调线程私有: 种子缓冲
	std::vector<double> cb_q_;            // 回调线程私有: 解缓冲

	// ---- OTG 输出级 (RT 线程私有; §16.6: OtgStream 唯一平滑入口) ----
	unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints> otg_;
	bool otg_ready_{false};

	// ---- StaleWatch 断流受控减速 (0c; 断流 = OtgStream 自目标刹停 C2) ----
	double stale_ms_cfg_{0.0};
	double stale_decel_ms_cfg_{200.0};
	uint32_t stale_cycles_{0};
	unistackbot_common::StaleWatch watch_;
	bool stream_stale_{false};
	bool was_stale_{false};
	bool rate_calibrated_{false};         // update_rate 定源 (权威参数优先, 见 cpp)
	static constexpr uint32_t kPeriodSamples = 16u;
	double period_samples_[kPeriodSamples] = {0};
	uint32_t period_n_{0};
	std::vector<double> vmax_;            // URDF max_velocity 原始值 (OtgStream Limits 源)
	std::vector<double> step_limits_;     // [链序] 单拍步长上限 (安全钳位底线)

	// ---- WCET 统计 (会话级, 停用时终报) ----
	std::chrono::steady_clock::time_point update_enter_{};
	double wcet_sum_us_{0.0}, wcet_max_us_{0.0};
	std::vector<double> wcet_samples_;
	uint64_t update_count_{0};

	// ---- 契约通道 ----
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
	rclcpp::Subscription<unistackbot_interface::msg::CartesianControl>::SharedPtr control_sub_;
	rclcpp::Publisher<unistackbot_interface::msg::CartesianMotionStatus>::SharedPtr status_pub_;
	std::shared_ptr<realtime_tools::RealtimePublisher<unistackbot_interface::msg::CartesianMotionStatus>> rt_status_;
	std::atomic<uint8_t> control_mode_{unistackbot_interface::msg::CartesianControl::TRACKING};
	rclcpp::Time last_status_time_{};
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_

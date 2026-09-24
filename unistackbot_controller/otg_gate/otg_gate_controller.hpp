#ifndef UNISTACKBOT_CONTROLLER__OTG_GATE_CONTROLLER_HPP_
#define UNISTACKBOT_CONTROLLER__OTG_GATE_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sp_latest/sp_latest.hpp"
#include "ruckig/otg_stream.hpp"
#include "unistackbot_interface/joint_capacity.hpp"
#include "unistackbot_interface/msg/joint_command.hpp"

#include "analytic_piper/analytic_piper.hpp"
#include "dls_ik/dls_ik.hpp"
#include "ik_solver/ik_solver.hpp"
#include "urdf_fk/urdf_fk.hpp"

namespace unistackbot_controller
{

/*
 * OtgGateController —— OTG 门 (P1, 设计 §16.4 路线 A 的壳, 2026-09-23)。
 *
 * 职责: "上层只有终点" 的对齐服务 —— 笛卡尔终点 → (回调线程一次 IK) → 关节终点
 * → (RT 环 OtgStream 每拍整形) → 总线频率 JointCommand 点流 → JointStream
 * (hold 档满速退化 = 透传) → write 层防线。关节 C2 平滑 + 加速度界由本路径引入。
 *
 * 线程模型 (ROS I/O 纪律):
 *   - 订阅回调 (executor 线程): ~/target → 一次 IK (COLD_START, 50ms 预算, 不占 RT);
 *     /joint_states → 实测快照。回调永不进 RT 线程。
 *   - RT update(): OtgStream.update (~µs 级) + RealtimePublisher trylock 发布;
 *     零锁零分配 (joint_names 激活时预填, RT 只写 position)。
 *
 * 关节表: 全部 position 命令关节 (URDF <ros2_control>, 与 JS 同源解析);
 * 链关节 = OTG 输出, 非链关节 (如手指) = 实测回显 (门不发明夹爪运动)。
 *
 * 参数纪律: base_link/tip_link/ik_solver/max_acceleration/max_jerk/ik_timeout_ms
 * 必填无默认 (算法↔机型对应与 OTG 机型资产均由人显式声明, 缺配置 fail-fast)。
 * 实例隔离: fk_/ik_ 仅回调线程使用 (KDL 暂存不跨线程), RT 环零运动学计算。
 */
class OtgGateController : public controller_interface::ControllerInterface
{
public:
	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

	// 只读控制器: 零接口认领 (命令由 JointStream 认领, 本门是它上游的点流源)
	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	// 订阅回调 (executor 线程)
	void onTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

	// ---- 参数 (必填无默认; on_configure 缺配置即 ERROR) ----
	std::string base_link_, tip_link_;
	std::string ik_solver_name_;
	double max_acceleration_{0.0};      // OTG 关节加速度界 [rad/s²] (机型资产, yaml 显式)
	double max_jerk_{0.0};              // [rad/s³]
	int64_t ik_timeout_ns_{0};          // 回调线程 IK 预算 (冷启动档; yaml ik_timeout_ms)

	// ---- 运动学 (回调线程私有; RT 环零运动学) ----
	std::unique_ptr<unistackbot_algorithm::UrdfFk> fk_;
	std::unique_ptr<unistackbot_algorithm::IkSolver> ik_;

	// ---- 关节表 (URDF <ros2_control> position 命令序) 与链序映射 ----
	std::vector<std::string> cmd_joint_names_;
	std::vector<double> cmd_vmax_;                  // [命令序] URDF max_velocity
	std::vector<int> chain_from_cmd_;               // [命令序] -> 链序 (-1=非链关节)
	std::vector<int> cmd_from_chain_;               // [链序] -> 命令序
	uint32_t chain_n_{0};

	// ---- 跨线程通道 (POD, SpLatest 值通道) ----
	struct GateTarget                    // 回调 → RT: 关节终点 (链序)
	{
		double q[unistackbot_interface::kMaxJoints];
		uint32_t n;
	};
	struct JointSnapshot                // 回调 → RT: 实测 (命令序, 非链关节回显源)
	{
		double pos[unistackbot_interface::kMaxJoints];
		uint32_t n;
	};
	static_assert(std::is_trivially_copyable_v<GateTarget>, "GateTarget 须 POD (SpLatest 契约)");
	static_assert(std::is_trivially_copyable_v<JointSnapshot>, "JointSnapshot 须 POD (SpLatest 契约)");
	unistackbot_common::SpLatest<GateTarget> target_ch_;
	unistackbot_common::SpLatest<JointSnapshot> snap_ch_;
	// 目标水位 (重激活防重放, 2026-09-23 审查补, 同 CM worker cold_req_ 模式):
	// SpLatest 会留着上一次激活期的目标, 无水位则重激活首拍把它当新目标追打。
	// on_activate 预读残值记 seq, update 只认 seq != 水位 的新目标; 本次激活期的
	// 目标存 goal_ 持续整形 (点流语义: 每拍都向当前目标发 OTG 输出)。
	uint64_t target_seq_{0};
	bool have_goal_{false};
	GateTarget goal_{};
	double js_cache_[unistackbot_interface::kMaxJoints] = {0};   // 回调线程私有: 实测累计 (未知关节保持旧值)

	// ---- OTG (RT 线程私有) ----
	unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints> otg_;
	bool otg_ready_{false};

	// ---- update_rate 校准 (Humble 坑: configure 期 get_update_rate() 无效; 同 JS/CM) ----
	static constexpr uint32_t kPeriodSamples = 16u;
	double period_samples_[kPeriodSamples] = {0};
	uint32_t period_n_{0};
	bool dt_known_{false};
	double dt_{0.002};

	// ---- 契约通道 ----
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
	rclcpp::Publisher<unistackbot_interface::msg::JointCommand>::SharedPtr cmd_pub_;
	std::shared_ptr<realtime_tools::RealtimePublisher<unistackbot_interface::msg::JointCommand>> rt_cmd_;
	std::string warned_frame_;   // 非本帧目标单次警告去重
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__OTG_GATE_CONTROLLER_HPP_

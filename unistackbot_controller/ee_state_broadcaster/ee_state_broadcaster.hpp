#ifndef UNISTACKBOT_CONTROLLER__EE_STATE_BROADCASTER_HPP_
#define UNISTACKBOT_CONTROLLER__EE_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "urdf_fk/urdf_fk.hpp"

using unistackbot_algorithm::UrdfFk;

namespace unistackbot_controller
{

/*
 * EE 位姿独立反馈流控制器 (仿真测试方案批次2, 2026-09-21)。
 *
 * 动机: CM ~/status 里的 current_pose 只在 CM active 时存在 (20Hz), 切到
 * JointStream 就没有 EE 位姿反馈话题; /tf 虽永远有但消费要 tf2 查询。
 * 本控制器与控制器选择无关地持续发布 ~/ee_state (PoseStamped, base 系)。
 *
 * 只读: 仅认领 state 接口 (与 JS/CM/JSB 共存, 无命令接口竞争)。
 *
 * 载体 = PoseStamped (四元数): **旋转表示决策挂起** (用户待定四元数/RPY/矩阵)——
 * 四元数与 TF 兼容且任何表示可导出; 决策后升自定义 EeState.msg 加并行字段,
 * PoseStamped 消费者不受影响 (docs/sim_environment_and_test_plan.md §3.2)。
 *
 * 契约:
 *   ~/ee_state  geometry_msgs/PoseStamped (base 系, FK(关节状态), 默认 50Hz)
 * 参数: base_link / tip_link / publish_hz / robot_description (CM 注入)
 */
class EeStateBroadcaster : public controller_interface::ControllerInterface
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
	// 参数 (on_init 声明)
	std::string base_link_, tip_link_;
	double publish_hz_{50.0};

	// FK (update 线程私有; KDL 有状态暂存, 实例不跨线程 —— urdf_fk.hpp 契约)
	std::unique_ptr<UrdfFk> fk_;
	UrdfFk::Scratch fk_scratch_;

	// 接口序 -> 链序映射 (接口声明序 != FK 链序, 按名映射 —— 与 CM 同款)
	std::vector<std::size_t> chain_from_iface_;

	// RT 发布 (trylock, 50Hz 时间节流)
	std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> pub_;
	rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__EE_STATE_BROADCASTER_HPP_

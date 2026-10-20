#ifndef UNISTACKBOT_CONTROLLER__UNI_JOINT_STATE_BROADCASTER_HPP_
#define UNISTACKBOT_CONTROLLER__UNI_JOINT_STATE_BROADCASTER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace unistackbot_controller
{

/*
 * 自研关节状态广播控制器 (2026-10-20, 三链取代官方 JSB 的位置):
 *
 * 动机 (mcap 实锤 stationlite mujoco 20261020): 官方 JointStateBroadcaster 默认模式
 * 输出序 = ResourceManager 无序容器遍历序, 无契约保证 —— 实测 /joint_states 乱序
 * (left_joint1, right_joint6, ...), 与 command 侧 (URDF 声明序) 错位, 按位置切数组
 * 的下游 (训练数据观测/动作拼接、录制分析) 全错。官方 joints+interfaces 参数可钉序
 * 但逐关节只认领一个接口 —— 丢 velocity/effort (mujoco 链真实动力学数据, 不舍)。
 *
 * 契约:
 *   /joint_states  sensor_msgs/JointState (顶层绝对名, 官方 JSB 同款), name 序 = URDF <ros2_control> 声明序
 *                  (与 command 侧 parseJointsFromUrdf 同一遍历口径 —— 共享
 *                  forEachRos2ControlJoint 骨架, 命令关节子序列逐位对齐);
 *                  全接口透传, 无该接口的字段填 0 (官方 JSB 同语义)。
 * 参数: publish_hz (默认 0 = 全率跟随 update_rate, 与官方行为一致; >0 时间节流) /
 *        robot_description (controller_manager 注入)
 *
 * 只读控制器: 零命令接口认领 (与 JS/CM/EEB 共存)。
 * 实例名保持 joint_state_broadcaster → 话题名与官方在位时完全一致, 下游零感知。
 * (库名 uni_ 前缀避让官方 libjoint_state_broadcaster.so —— 同名 so class loader
 *  按 LD_LIBRARY_PATH 顺序定位, 不可控)
 */
class UniJointStateBroadcaster : public controller_interface::ControllerInterface
{
public:
	static constexpr std::size_t kIfaceCount = 3;   // position / velocity / effort

	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	std::vector<std::string> names_;       // URDF 声明序 (输出契约)
	std::vector<std::string> state_claims_;   // "name/iface" 认领清单 (names 序 × 接口序, 单一事实源)

	// 每关节各接口 → state_interfaces_ 索引 (-1 = 该关节无此接口, 填 0)
	struct Channels
	{
		std::array<int, kIfaceCount> idx{{-1, -1, -1}};
	};
	std::vector<Channels> chan_;   // on_activate 按名映射填 (接口到手序不可信, EEB 同款防御)

	double publish_hz_{0.0};

	std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> pub_;
	rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__UNI_JOINT_STATE_BROADCASTER_HPP_

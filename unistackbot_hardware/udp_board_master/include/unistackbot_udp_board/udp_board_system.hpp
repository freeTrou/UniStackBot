#ifndef UNISTACKBOT_UDP_BOARD__UDP_BOARD_SYSTEM_HPP_
#define UNISTACKBOT_UDP_BOARD__UDP_BOARD_SYSTEM_HPP_
/*
 * UdpBoardSystem —— ros2_control SystemInterface 薄消费者 (2026-11-02 总线/协议拆分)
 *
 * 全工程唯一碰 ROS 的总线路径 (MasterBase 契约): createMaster("udp") → start(cfg)
 * → read()=take_state / write()=publish_cmd。总线/协议/状态机全在轴上:
 *   UdpMaster (本包) · board_protocol (unistackbot_protocol) · BoardSafety (statemachine)
 *
 * URDF <param>: dest_ip / dest_port / bind_port / rate_hz (缺省 500)
 * (节点表 = URDF joint 声明序, 槽位 = 序号; 底盘槽 M4)。
 * 仿真模式: dest_ip=127.0.0.1 → FakeBoard (零硬件端到端); 真机 = 板地址。
 */
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "unistackbot_bus/master_base.hpp"

namespace unistackbot_udp_board
{

class UdpBoardSystem : public hardware_interface::SystemInterface
{
public:
	RCLCPP_SHARED_PTR_DEFINITIONS(UdpBoardSystem)

	[[nodiscard]] hardware_interface::CallbackReturn on_init(
		const hardware_interface::HardwareInfo & info) override;
	[[nodiscard]] hardware_interface::CallbackReturn on_configure(
		const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] hardware_interface::CallbackReturn on_activate(
		const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] hardware_interface::CallbackReturn on_deactivate(
		const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
	[[nodiscard]] std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

	[[nodiscard]] hardware_interface::return_type read(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;
	[[nodiscard]] hardware_interface::return_type write(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	std::unique_ptr<unistackbot_bus::MasterBase> master_;
	std::vector<unistackbot_bus::NodeSnapshot> snap_;

	// 接口镜像 (interface 序 = URDF 声明序 = 节点槽序)
	std::vector<double> cmd_pos_;
	std::vector<double> state_pos_;
	std::vector<double> state_vel_;
	std::vector<double> state_eff_;
	std::vector<std::string> joint_names_;
	std::vector<unistackbot_protocol::NodeCommand> cmd_tx_;
};

}  // namespace unistackbot_udp_board
#endif  // UNISTACKBOT_UDP_BOARD__UDP_BOARD_SYSTEM_HPP_

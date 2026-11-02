#ifndef UNISTACKBOT_UDP_BOARD__UDP_BOARD_SYSTEM_HPP_
#define UNISTACKBOT_UDP_BOARD__UDP_BOARD_SYSTEM_HPP_
/*
 * UdpBoardSystem —— ros2_control SystemInterface 插件 (Phase 3, 2026-10-30)
 *
 * udp_board = 第四条总线 (ODE 裁决4): Lumos 电控板接入。
 * 泵模型: read() 非阻塞收 Response 帧 → 解码填状态接口; write() 打包 Control 帧 → 非阻塞发。
 * 控制器管理器 500Hz 驱动 read→update→write = 总线泵拍 (与板固件 200Hz 兼容——主站节拍可高,
 * 板侧按收到即回的语义工作; 真板对齐留 M4 真机批次)。
 *
 * 仿真模式: dest_ip=127.0.0.1 → FakeBoard (零硬件端到端);
 * 真机模式: dest_ip=<板地址> (M4 真机批次, 协议同款)。
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unistackbot_udp_board/board_protocol.hpp"

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
	// UDP
	int sock_{-1};
	sockaddr_in board_addr_{};
	uint16_t bind_port_{9090};
	uint8_t tx_seq_{0};
	uint8_t rx_seq_{0};

	// 关节状态 (interface 序 = URDF 声明序)
	std::vector<double> cmd_pos_;    // 写入命令 (position)
	std::vector<double> state_pos_;  // 读取状态 (position)
	std::vector<double> state_vel_;  // 读取状态 (velocity)
	std::vector<double> state_eff_;  // 读取状态 (effort, 恒 0)
	std::vector<std::string> joint_names_;

	// 帧缓冲
	uint8_t tx_buf_[kControlFrameSize];
	uint8_t rx_buf_[kResponseFrameSize];

	// 定标 (raw ↔ 物理; Phase 3 简化为恒 1:1, 真机批次接 HAL 标定 yaml)
	static constexpr double kPosScale = 1000.0;   // raw ↔ rad (实录: tar_vel raw ≈ rad/s × 1000)
};

}  // namespace unistackbot_udp_board

#endif  // UNISTACKBOT_UDP_BOARD__UDP_BOARD_SYSTEM_HPP_

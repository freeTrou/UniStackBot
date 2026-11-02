#include <fcntl.h>
#include "unistackbot_udp_board/udp_board_system.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace unistackbot_udp_board
{

static auto & logger() { static auto lg = rclcpp::get_logger("udp_board"); return lg; }

hardware_interface::CallbackReturn UdpBoardSystem::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	// 参数 (URDF <param> 来源)
	const auto & params = info.hardware_parameters;
	auto it = params.find("dest_ip");
	const std::string dest_ip = it != params.end() ? it->second : "127.0.0.1";
	it = params.find("dest_port");
	const int dest_port = it != params.end() ? std::atoi(it->second.c_str()) : 10086;
	it = params.find("bind_port");
	bind_port_ = it != params.end() ? static_cast<uint16_t>(std::atoi(it->second.c_str())) : 9090;

	// 关节集 (URDF 声明序 = 命令/状态 interface 序)
	joint_names_.clear();
	for (const auto & j : info_.joints)
	{
		joint_names_.push_back(j.name);
	}
	const size_t n = joint_names_.size();
	cmd_pos_.assign(n, 0.0);
	state_pos_.assign(n, 0.0);
	state_vel_.assign(n, 0.0);
	state_eff_.assign(n, 0.0);

	// UDP socket
	sock_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock_ < 0)
	{
		RCLCPP_ERROR(logger(), "udp_board: socket 创建失败");
		return hardware_interface::CallbackReturn::ERROR;
	}
	// 绑定接收端口
	sockaddr_in bind_addr{};
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_addr.s_addr = INADDR_ANY;
	bind_addr.sin_port = htons(bind_port_);
	if (bind(sock_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0)
	{
		RCLCPP_ERROR(logger(), "udp_board: bind 端口 %u 失败", bind_port_);
		close(sock_);
		sock_ = -1;
		return hardware_interface::CallbackReturn::ERROR;
	}
	// 非阻塞
	int flags = fcntl(sock_, F_GETFL, 0);
	fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

	// 板地址
	memset(&board_addr_, 0, sizeof(board_addr_));
	board_addr_.sin_family = AF_INET;
	board_addr_.sin_addr.s_addr = inet_addr(dest_ip.c_str());
	board_addr_.sin_port = htons(static_cast<uint16_t>(dest_port));

	RCLCPP_INFO(logger(), "udp_board: on_init (%zu 关节, dest=%s:%d, bind=%u)", n, dest_ip.c_str(), dest_port, bind_port_);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UdpBoardSystem::on_configure(const rclcpp_lifecycle::State &)
{
	// 零位基准
	std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
	std::fill(state_pos_.begin(), state_pos_.end(), 0.0);
	std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
	RCLCPP_INFO(logger(), "udp_board: 配置完成 (零位基准)");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UdpBoardSystem::on_activate(const rclcpp_lifecycle::State &)
{
	tx_seq_ = 0;
	rx_seq_ = 0;
	RCLCPP_INFO(logger(), "udp_board: 激活");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UdpBoardSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger(), "udp_board: 去激活");
	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UdpBoardSystem::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> interfaces;
	for (size_t i = 0; i < joint_names_.size(); ++i)
	{
		interfaces.emplace_back(joint_names_[i], "position", &state_pos_[i]);
		interfaces.emplace_back(joint_names_[i], "velocity", &state_vel_[i]);
		interfaces.emplace_back(joint_names_[i], "effort", &state_eff_[i]);
	}
	return interfaces;
}

std::vector<hardware_interface::CommandInterface> UdpBoardSystem::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> interfaces;
	for (size_t i = 0; i < joint_names_.size(); ++i)
	{
		interfaces.emplace_back(joint_names_[i], "position", &cmd_pos_[i]);
	}
	return interfaces;
}

hardware_interface::return_type UdpBoardSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
	// 非阻塞收: 有 Response 帧就解码填状态 (无帧/坏帧状态保持)
	sockaddr_in from{};
	socklen_t flen = sizeof(from);
	const ssize_t n = recvfrom(sock_, rx_buf_, sizeof(rx_buf_), MSG_DONTWAIT,
		reinterpret_cast<sockaddr *>(&from), &flen);
	if (n != static_cast<ssize_t>(kResponseFrameSize))
	{
		return hardware_interface::return_type::OK;
	}

	ResponseFrameBits rsp{};
	if (!decodeResponse(rx_buf_, static_cast<size_t>(n), rsp))
	{
		return hardware_interface::return_type::OK;
	}
	rx_seq_ = rsp.ts;

	// 解码填状态 (关节序 = URDF 序; 总线序 = [臂左7+夹左] [臂右7+夹右] [头2] [body1] [轮8])
	// Phase 3: 1:1 映射 (URDF ros2_control 声明序 == 总线关节序 == P20 URDF <ros2_control> 序)
	const size_t nj = joint_names_.size();
	for (size_t i = 0; i < nj && i < kArmBusCount * kArmMotorsPerBus; ++i)
	{
		const size_t bus = i / kArmMotorsPerBus;
		const size_t motor = i % kArmMotorsPerBus;
		if (bus < kArmBusCount)
		{
			state_pos_[i] = static_cast<double>(rsp.bus[bus].motor[motor].cur_pos) / kPosScale;
			state_vel_[i] = static_cast<double>(rsp.bus[bus].motor[motor].cur_vel) / kPosScale;
			state_eff_[i] = 0.0;   // 位置传感器无 effort 反馈
		}
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type UdpBoardSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
	// 打包命令帧
	ControlFrameBits cmd{};
	cmd.ts = tx_seq_;

	const size_t nj = joint_names_.size();
	for (size_t i = 0; i < nj && i < kArmBusCount * kArmMotorsPerBus; ++i)
	{
		const size_t bus = i / kArmMotorsPerBus;
		const size_t motor = i % kArmMotorsPerBus;
		if (bus < kArmBusCount)
		{
			cmd.bus[bus].ctrl_word = kCtrlWordPdMit;   // PD/MIT 模式
			cmd.bus[bus].motor[motor].enable = 1;
			cmd.bus[bus].motor[motor].tar_pos =
				static_cast<int16_t>(cmd_pos_[i] * kPosScale);
		}
	}

	if (!encodeControl(cmd, tx_buf_, sizeof(tx_buf_)))
	{
		return hardware_interface::return_type::ERROR;
	}
	sendto(sock_, tx_buf_, sizeof(tx_buf_), 0,
		reinterpret_cast<const sockaddr *>(&board_addr_), sizeof(board_addr_));
	++tx_seq_;

	return hardware_interface::return_type::OK;
}

}  // namespace unistackbot_udp_board

PLUGINLIB_EXPORT_CLASS(unistackbot_udp_board::UdpBoardSystem, hardware_interface::SystemInterface)

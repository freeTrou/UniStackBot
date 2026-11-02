#include "unistackbot_udp_board/udp_board_system.hpp"

#include <cstdlib>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_udp_board/udp_master.hpp"

namespace unistackbot_udp_board
{

static auto & logger() { static auto lg = rclcpp::get_logger("udp_board"); return lg; }

hardware_interface::CallbackReturn UdpBoardSystem::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}
	const auto & params = info_.hardware_parameters;
	auto it = params.find("dest_ip");
	const std::string dest_ip = it != params.end() ? it->second : "127.0.0.1";
	it = params.find("dest_port");
	const int dest_port = it != params.end() ? std::atoi(it->second.c_str()) : 10086;
	it = params.find("bind_port");
	const int bind_port = it != params.end() ? std::atoi(it->second.c_str()) : 9090;
	it = params.find("rate_hz");
	const double rate_hz = it != params.end() ? std::atof(it->second.c_str()) : 500.0;

	// 节点表 = URDF joint 声明序 (槽位 = 序号; 底盘槽 M4)
	joint_names_.clear();
	unistackbot_bus::MasterConfig cfg;
	cfg.endpoint = dest_ip + ":" + std::to_string(dest_port) + ":" + std::to_string(bind_port);
	cfg.rate_hz = rate_hz;
	cfg.protocol = "udp_board";
	cfg.statemachine = "udp_board";
	for (const auto & j : info_.joints)
	{
		joint_names_.push_back(j.name);
		unistackbot_bus::NodeSpec spec;
		spec.id = static_cast<std::uint8_t>(cfg.nodes.size());
		cfg.nodes.push_back(spec);
	}
	const std::size_t n = joint_names_.size();
	cmd_pos_.assign(n, 0.0);
	state_pos_.assign(n, 0.0);
	state_vel_.assign(n, 0.0);
	state_eff_.assign(n, 0.0);
	cmd_tx_.assign(n, unistackbot_protocol::NodeCommand{});
	snap_.assign(n, unistackbot_bus::NodeSnapshot{});

	// 组合根: 工厂注册 + 创建 (serial 先例)
	static const bool registered = UdpMaster::registerToMasterFactory();
	(void)registered;
	master_ = unistackbot_bus::createMaster("udp");
	if (!master_)
	{
		RCLCPP_ERROR(logger(), "udp_board: createMaster(\"udp\") 失败");
		return hardware_interface::CallbackReturn::ERROR;
	}
	if (!master_->start(cfg))
	{
		RCLCPP_ERROR(logger(), "udp_board: 主站 start 拒绝 (endpoint=%s, %zu 节点)",
			cfg.endpoint.c_str(), n);
		master_.reset();
		return hardware_interface::CallbackReturn::ERROR;
	}
	RCLCPP_INFO(logger(), "udp_board: 主站启动 (%zu 关节, %s @%.0fHz)", n, cfg.endpoint.c_str(), rate_hz);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UdpBoardSystem::on_configure(const rclcpp_lifecycle::State &)
{
	std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
	std::fill(state_pos_.begin(), state_pos_.end(), 0.0);
	std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UdpBoardSystem::on_activate(const rclcpp_lifecycle::State &)
{
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
	for (std::size_t i = 0; i < joint_names_.size(); ++i)
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
	for (std::size_t i = 0; i < joint_names_.size(); ++i)
	{
		interfaces.emplace_back(joint_names_[i], "position", &cmd_pos_[i]);
	}
	return interfaces;
}

hardware_interface::return_type UdpBoardSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
	if (!master_ || !master_->take_state(snap_))
	{
		return hardware_interface::return_type::OK;   // 无新快照, 状态保持
	}
	for (std::size_t i = 0; i < joint_names_.size() && i < snap_.size(); ++i)
	{
		state_pos_[i] = snap_[i].fb.position;
		state_vel_[i] = snap_[i].fb.speed;
		state_eff_[i] = snap_[i].fb.tau;
	}
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type UdpBoardSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
	if (!master_)
	{
		return hardware_interface::return_type::ERROR;
	}
	for (std::size_t i = 0; i < cmd_tx_.size(); ++i)
	{
		cmd_tx_[i].mode = unistackbot_protocol::NodeMode::kRun;
		cmd_tx_[i].position = cmd_pos_[i];
	}
	master_->publish_cmd(cmd_tx_);
	return hardware_interface::return_type::OK;
}

}  // namespace unistackbot_udp_board

PLUGINLIB_EXPORT_CLASS(unistackbot_udp_board::UdpBoardSystem, hardware_interface::SystemInterface)

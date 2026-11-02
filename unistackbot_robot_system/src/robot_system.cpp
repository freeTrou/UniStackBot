#include "unistackbot_robot_system/robot_system.hpp"

#include <algorithm>
#include <cstdlib>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_udp_board/udp_master.hpp"

namespace unistackbot_robot_system
{

static auto & logger() { static auto lg = rclcpp::get_logger("robot_system"); return lg; }

void RobotSystem::registerKnownBuses()
{
	// 组合根注册表 (加总线 = 加一行; 各包自注册幂等)
	static const bool done = []() {
		(void)unistackbot_udp_board::UdpMaster::registerToMasterFactory();
		return true;
	}();
	(void)done;
}

hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}
	const auto & params = info_.hardware_parameters;
	auto it = params.find("bus");
	if (it == params.end() || it->second.empty())
	{
		RCLCPP_ERROR(logger(), "robot_system: 缺 <param name=\"bus\"> (总线名 = createMaster 工厂 key)");
		return hardware_interface::CallbackReturn::ERROR;
	}
	const std::string bus = it->second;
	it = params.find("endpoint");
	if (it == params.end() || it->second.empty())
	{
		RCLCPP_ERROR(logger(), "robot_system: 缺 <param name=\"endpoint\"> (格式由总线解释)");
		return hardware_interface::CallbackReturn::ERROR;
	}
	double rate_hz = 500.0;
	it = params.find("rate_hz");
	if (it != params.end())
	{
		rate_hz = std::atof(it->second.c_str());
	}

	// 节点表 = URDF joint 声明序 (state-only 关节 = 只读节点, 主站按 kind 分槽)
	joint_names_.clear();
	unistackbot_bus::MasterConfig cfg;
	cfg.endpoint = params.at("endpoint");
	cfg.rate_hz = rate_hz;
	if ((it = params.find("protocol")) != params.end())
	{
		cfg.protocol = it->second;
	}
	if ((it = params.find("statemachine")) != params.end())
	{
		cfg.statemachine = it->second;
	}
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

	registerKnownBuses();
	master_ = unistackbot_bus::createMaster(bus);
	if (!master_)
	{
		RCLCPP_ERROR(logger(), "robot_system: createMaster('%s') 失败 (未注册的总线名)", bus.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	if (!master_->start(cfg))
	{
		RCLCPP_ERROR(logger(), "robot_system: 总线 '%s' start 拒绝 (endpoint=%s, %zu 节点)",
			bus.c_str(), cfg.endpoint.c_str(), n);
		master_.reset();
		return hardware_interface::CallbackReturn::ERROR;
	}
	RCLCPP_INFO(logger(), "robot_system: 总线 '%s' 启动 (%zu 关节, %s @%.0fHz)",
		bus.c_str(), n, cfg.endpoint.c_str(), rate_hz);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State &)
{
	std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
	std::fill(state_pos_.begin(), state_pos_.end(), 0.0);
	std::fill(state_vel_.begin(), state_vel_.end(), 0.0);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger(), "robot_system: 激活");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(logger(), "robot_system: 去激活");
	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> interfaces;
	for (std::size_t i = 0; i < joint_names_.size(); ++i)
	{
		interfaces.emplace_back(joint_names_[i], "position", &cmd_pos_[i]);
	}
	return interfaces;
}

hardware_interface::return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
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

hardware_interface::return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
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

}  // namespace unistackbot_robot_system

PLUGINLIB_EXPORT_CLASS(unistackbot_robot_system::RobotSystem, hardware_interface::SystemInterface)

#include "unistackbot_sim_control/sim_control_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistackbot_sim_control/backend_kinematic.hpp"

namespace unistackbot_sim_control
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("SimControlHardware");

namespace
{

constexpr double kDefaultMaxVelocity = 5.0;   // 未声明 max_velocity 时的默认执行器速度上限

hardware_interface::CallbackReturn toCallback(const bool ok)
{
	return ok ? hardware_interface::CallbackReturn::SUCCESS
						: hardware_interface::CallbackReturn::ERROR;
}

}  // namespace

hardware_interface::CallbackReturn SimControlHardware::on_init(
	const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SystemInterface::on_init(info) !=
		hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	RCLCPP_INFO(LOGGER, "SimControlHardware on_init: %zu joints", info_.joints.size());

	const size_t joint_count = info_.joints.size();
	if (joint_count > kMaxJoints) {
		RCLCPP_ERROR(LOGGER, "joint count %zu exceeds kMaxJoints=%u",
			joint_count, kMaxJoints);
		return hardware_interface::CallbackReturn::ERROR;
	}

	joints_.resize(joint_count);
	cmd_position_.assign(joint_count, 0.0);
	state_position_.assign(joint_count, 0.0);
	state_velocity_.assign(joint_count, 0.0);
	state_effort_.assign(joint_count, 0.0);

	// 第一遍: 名字索引 / 限位 / 速度上限 / 接口契约校验
	std::unordered_map<std::string, size_t> joint_index;
	for (size_t i = 0; i < joint_count; ++i) {
		const auto & joint = info_.joints[i];
		joint_index[joint.name] = i;
		joints_[i].name = joint.name;

		joints_[i].min = 0.0;
		joints_[i].max = 0.0;
		if (joint.parameters.count("min")) {
			joints_[i].min = std::stod(joint.parameters.at("min"));
		}
		if (joint.parameters.count("max")) {
			joints_[i].max = std::stod(joint.parameters.at("max"));
		}
		joints_[i].max_velocity = kDefaultMaxVelocity;
		if (joint.parameters.count("max_velocity")) {
			joints_[i].max_velocity = std::stod(joint.parameters.at("max_velocity"));
		}

		// 命令接口契约: 恰好一个 position
		if (joint.command_interfaces.size() != 1 ||
			joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_ERROR(LOGGER,
				"Joint '%s' must declare exactly one position command interface",
				joint.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		// 状态接口契约: position 必须有, 其余只能是 velocity/effort (effort 恒 0)
		bool has_position = false;
		for (const auto & si : joint.state_interfaces) {
			if (si.name == hardware_interface::HW_IF_POSITION) {
				has_position = true;
			} else if (si.name != hardware_interface::HW_IF_VELOCITY &&
				si.name != hardware_interface::HW_IF_EFFORT)
			{
				RCLCPP_ERROR(LOGGER,
					"Joint '%s' declares unsupported state interface '%s'",
					joint.name.c_str(), si.name.c_str());
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		if (!has_position) {
			RCLCPP_ERROR(LOGGER,
				"Joint '%s' must declare a position state interface", joint.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
	}

	// 第二遍: mimic 关系解析
	for (size_t i = 0; i < joint_count; ++i) {
		const auto & params = info_.joints[i].parameters;
		auto mimic_it = params.find("mimic");
		if (mimic_it == params.end() || mimic_it->second.empty()) {
			continue;
		}
		auto src_it = joint_index.find(mimic_it->second);
		if (src_it == joint_index.end()) {
			RCLCPP_ERROR(LOGGER, "Joint '%s' mimics unknown joint '%s'",
				info_.joints[i].name.c_str(), mimic_it->second.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		joints_[i].mimic_source = static_cast<int>(src_it->second);
		joints_[i].mimic_multiplier = 1.0;
		joints_[i].mimic_offset = 0.0;
		if (params.count("multiplier")) {
			joints_[i].mimic_multiplier = std::stod(params.at("multiplier"));
		}
		if (params.count("offset")) {
			joints_[i].mimic_offset = std::stod(params.at("offset"));
		}
		RCLCPP_INFO(LOGGER, "Joint '%s' mimics '%s' (multiplier %.3f, offset %.3f)",
			joints_[i].name.c_str(), mimic_it->second.c_str(),
			joints_[i].mimic_multiplier, joints_[i].mimic_offset);
	}

	// 后端分类 (对内): "backend" 为 <hardware> 级参数
	std::string backend_name = "kinematic";
	if (info_.hardware_parameters.count("backend")) {
		backend_name = info_.hardware_parameters.at("backend");
	}
	std::string message;
	if (backend_name == "kinematic") {
		backend_ = std::make_unique<BackendKinematic>();
	} else {
		RCLCPP_ERROR(LOGGER, "Unknown backend '%s' (supported: kinematic)",
			backend_name.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	if (!backend_->init(joints_, message)) {
		RCLCPP_ERROR(LOGGER, "Backend '%s' init failed: %s",
			backend_name.c_str(), message.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	RCLCPP_INFO(LOGGER, "Backend '%s' ready (%s)", backend_name.c_str(), message.c_str());

	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SimControlHardware::export_state_interfaces()
{
	// 镜像 URDF 声明: 声明了什么状态接口就导出什么 (effort 恒 0)
	std::vector<hardware_interface::StateInterface> interfaces;
	for (size_t i = 0; i < info_.joints.size(); ++i) {
		for (const auto & si : info_.joints[i].state_interfaces) {
			if (si.name == hardware_interface::HW_IF_POSITION) {
				interfaces.emplace_back(
					info_.joints[i].name, si.name, &state_position_[i]);
			} else if (si.name == hardware_interface::HW_IF_VELOCITY) {
				interfaces.emplace_back(
					info_.joints[i].name, si.name, &state_velocity_[i]);
			} else if (si.name == hardware_interface::HW_IF_EFFORT) {
				interfaces.emplace_back(
					info_.joints[i].name, si.name, &state_effort_[i]);
			}
		}
	}
	return interfaces;
}

std::vector<hardware_interface::CommandInterface>
SimControlHardware::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> interfaces;
	interfaces.reserve(cmd_position_.size());
	for (size_t i = 0; i < cmd_position_.size(); ++i) {
		interfaces.emplace_back(
			info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
	}
	return interfaces;
}

hardware_interface::CallbackReturn SimControlHardware::on_configure(
	const rclcpp_lifecycle::State &)
{
	// /sim_control 服务: 独立节点 + 独立 executor 线程 (非实时)
	svc_node_ = rclcpp::Node::make_shared("sim_control");
	server_ = std::make_unique<SimControlServer>(
		svc_node_,
		[this](const SimCommand & cmd, std::string & message) {
			return enqueueSink(cmd, message);
		},
		[this](const std::vector<std::string> & names,
			const std::vector<double> & positions,
			const std::vector<double> & velocities,
			SimCommand & command,
			std::string & message) {
			return validateSetState(names, positions, velocities, command, message);
		});
	svc_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	svc_executor_->add_node(svc_node_);
	svc_running_ = true;
	svc_thread_ = std::thread([this]() {
		while (svc_running_.load() && rclcpp::ok()) {
			svc_executor_->spin_once(std::chrono::milliseconds(50));
		}
	});

	RCLCPP_INFO(LOGGER, "/sim_control services started");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimControlHardware::on_cleanup(
	const rclcpp_lifecycle::State &)
{
	svc_running_ = false;
	if (svc_thread_.joinable()) {
		svc_thread_.join();
	}
	svc_executor_->remove_node(svc_node_);
	server_.reset();
	svc_node_.reset();
	RCLCPP_INFO(LOGGER, "/sim_control services stopped");
	return hardware_interface::CallbackReturn::SUCCESS;
}

void SimControlHardware::drainCommands()
{
	SimCommand cmd;
	while (cmd_queue_.pop(cmd)) {
		switch (cmd.type) {
			case SimCmdType::RESET:
				for (size_t i = 0; i < joints_.size(); ++i) {
					if (joints_[i].is_mimic()) {
						continue;
					}
					state_position_[i] = 0.0;
					state_velocity_[i] = 0.0;
				}
				backend_->step(cmd_position_, state_position_, state_velocity_, 0.0, false);
				break;
			case SimCmdType::SET_STATE:
				for (uint32_t k = 0; k < cmd.count && k < kMaxJoints; ++k) {
					if (cmd.mask[k] != 0) {
						state_position_[k] = cmd.positions[k];
						state_velocity_[k] = cmd.velocities[k];
					}
				}
				backend_->step(cmd_position_, state_position_, state_velocity_, 0.0, false);
				break;
			case SimCmdType::PAUSE:
				paused_.store(true);
				break;
			case SimCmdType::RESUME:
				paused_.store(false);
				break;
			case SimCmdType::STEP:
				step_req_.fetch_add(1);
				break;
		}
	}
}

hardware_interface::return_type SimControlHardware::read(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	drainCommands();

	const double dt = period.seconds();
	bool integrate = dt > 0.0;
	if (paused_.load()) {
		if (step_req_.load() > 0) {
			step_req_.fetch_sub(1);   // 消费一次单步请求
		} else {
			integrate = false;        // 冻结 (mimic 仍按源推导)
		}
	}

	backend_->step(cmd_position_, state_position_, state_velocity_, dt, integrate);
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type SimControlHardware::write(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	// 运动学后端无外部设备: 命令已在 read() 中被消化
	return hardware_interface::return_type::OK;
}

bool SimControlHardware::validateSetState(
	const std::vector<std::string> & names,
	const std::vector<double> & positions,
	const std::vector<double> & velocities,
	SimCommand & command,
	std::string & message)
{
	if (names.empty()) {
		message = "joint_names is empty";
		return false;
	}
	if (positions.size() != names.size() ||
		(!velocities.empty() && velocities.size() != names.size()))
	{
		message = "positions/velocities size mismatch with joint_names";
		return false;
	}

	std::unordered_map<std::string, size_t> index;
	for (size_t i = 0; i < joints_.size(); ++i) {
		index[joints_[i].name] = i;
	}

	// 展开为按关节索引填充的命令; 未点名的关节 mask=0, 保持原状态
	command = SimCommand{};
	command.type = SimCmdType::SET_STATE;
	command.count = static_cast<uint32_t>(joints_.size());
	for (size_t k = 0; k < names.size(); ++k) {
		auto it = index.find(names[k]);
		if (it == index.end()) {
			message = "unknown joint '" + names[k] + "'";
			return false;
		}
		const JointMeta & joint = joints_[it->second];
		if (joint.is_mimic()) {
			message = "joint '" + names[k] + "' is a mimic joint (derived, not settable)";
			return false;
		}
		if (positions[k] < joint.min || positions[k] > joint.max) {
			message = "position for '" + names[k] + "' out of limits [" +
				std::to_string(joint.min) + ", " + std::to_string(joint.max) + "]";
			return false;
		}
		command.mask[it->second] = 1;
		command.positions[it->second] = positions[k];
		command.velocities[it->second] =
			velocities.empty() ? 0.0 : velocities[k];
	}
	return true;
}

bool SimControlHardware::enqueueSink(const SimCommand & cmd, std::string & message)
{
	if (!cmd_queue_.push(cmd)) {
		message = "sim command queue full";
		return false;
	}
	return true;
}

}  // namespace unistackbot_sim_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	unistackbot_sim_control::SimControlHardware,
	hardware_interface::SystemInterface)

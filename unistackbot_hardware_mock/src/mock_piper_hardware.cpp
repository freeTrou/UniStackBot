#include "unistackbot_hardware_mock/mock_piper_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace unistackbot_hardware_mock
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MockPiperHardware");

namespace
{

const std::unordered_map<std::string, double> JOINT_MAX_VELOCITY =
{
	{"joint1", 5.0}, {"joint2", 5.0}, {"joint3", 5.0},
	{"joint4", 5.0}, {"joint5", 5.0}, {"joint6", 5.0}, {"gripper", 3.0},
};

const std::unordered_map<std::string, bool> JOINT_HAS_EFFORT =
{
	{"joint1", true}, {"joint2", true}, {"joint3", true},
	{"joint4", true}, {"joint5", true}, {"joint6", true}, {"gripper", false},
};

double parse_param_or(const std::unordered_map<std::string, std::string> & params, const std::string & key, double fallback)
{
	auto it = params.find(key);
	if (it == params.end() || it->second.empty())
	{
		return fallback;
	}
	try
	{
		return std::stod(it->second);
	}
	catch (...)
	{
		return fallback;
	}
}

}  // namespace

hardware_interface::CallbackReturn MockPiperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	// 关节数量动态化: 除 6 个臂关节 + gripper 外, 还可有 mimic 跟随关节 (gripper_joint1/2)
	const size_t joint_count = info.joints.size();
	RCLCPP_INFO(LOGGER, "Initializing mock hardware with %zu joints", joint_count);

	cmd_position_.assign(joint_count, 0.0);
	state_position_.assign(joint_count, 0.0);
	state_velocity_.assign(joint_count, 0.0);
	state_effort_.assign(joint_count, 0.0);
	joint_lower_.resize(joint_count);
	joint_upper_.resize(joint_count);
	joint_max_velocity_.resize(joint_count);
	is_mimic_.assign(joint_count, false);
	mimic_source_.assign(joint_count, 0);
	mimic_multiplier_.assign(joint_count, 1.0);
	mimic_offset_.assign(joint_count, 0.0);

	std::unordered_map<std::string, size_t> joint_index;
	for (size_t i = 0; i < joint_count; ++i)
	{
		const auto & joint = info.joints[i];
		joint_index[joint.name] = i;
		const auto & params = joint.parameters;

		joint_lower_[i] = parse_param_or(params, "min", -std::numeric_limits<double>::infinity());
		joint_upper_[i] = parse_param_or(params, "max", std::numeric_limits<double>::infinity());

		auto vel_it = JOINT_MAX_VELOCITY.find(joint.name);
		joint_max_velocity_[i] = (vel_it != JOINT_MAX_VELOCITY.end()) ? vel_it->second : std::numeric_limits<double>::infinity();

		if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			RCLCPP_ERROR(
				LOGGER,
				"Joint '%s' must declare exactly one position command interface",
				joint.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		const auto eff_it = JOINT_HAS_EFFORT.find(joint.name);
		const bool needs_effort = (eff_it != JOINT_HAS_EFFORT.end()) && eff_it->second;
		const size_t expected_state = needs_effort ? 3 : 2;
		if (joint.state_interfaces.size() != expected_state)
		{
			RCLCPP_ERROR(
				LOGGER,
				"Joint '%s' must declare %zu state interfaces, got %zu",
				joint.name.c_str(), expected_state, joint.state_interfaces.size());
			return hardware_interface::CallbackReturn::ERROR;
		}
	}

	// mimic 跟随关节: 记录源关节与系数, 状态在 read() 中由源关节推导
	for (size_t i = 0; i < joint_count; ++i)
	{
		const auto & params = info.joints[i].parameters;
		auto mimic_it = params.find("mimic");
		if (mimic_it == params.end() || mimic_it->second.empty())
		{
			continue;
		}

		auto src_it = joint_index.find(mimic_it->second);
		if (src_it == joint_index.end())
		{
			RCLCPP_ERROR(
				LOGGER, "Joint '%s' mimics unknown joint '%s'",
				info.joints[i].name.c_str(), mimic_it->second.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		is_mimic_[i] = true;
		mimic_source_[i] = src_it->second;
		mimic_multiplier_[i] = parse_param_or(params, "multiplier", 1.0);
		mimic_offset_[i] = parse_param_or(params, "offset", 0.0);
		RCLCPP_INFO(
			LOGGER, "Joint '%s' mimics '%s' (multiplier %.3f, offset %.3f)",
			info.joints[i].name.c_str(), mimic_it->second.c_str(),
			mimic_multiplier_[i], mimic_offset_[i]);
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

bool MockPiperHardware::has_effort_at(size_t index) const
{
	if (index >= info_.joints.size())
	{
		return false;
	}
	const auto it = JOINT_HAS_EFFORT.find(info_.joints[index].name);

	return (it != JOINT_HAS_EFFORT.end()) && it->second;
}

std::vector<hardware_interface::StateInterface> MockPiperHardware::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> interfaces;
	interfaces.reserve(state_position_.size() * 3);

	for (size_t i = 0; i < state_position_.size(); ++i)
	{
		interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
		interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);

		if (has_effort_at(i))
		{
			interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
		}
	}
	return interfaces;
}

std::vector<hardware_interface::CommandInterface> MockPiperHardware::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> interfaces;
	interfaces.reserve(cmd_position_.size());
	for (size_t i = 0; i < cmd_position_.size(); ++i)
	{
		interfaces.emplace_back(
		info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
	}

	return interfaces;
}

hardware_interface::CallbackReturn MockPiperHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockPiperHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockPiperHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MockPiperHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	const double dt = period.seconds();
	if (dt <= 0.0)
	{
		return hardware_interface::return_type::OK;
	}

	for (size_t i = 0; i < state_position_.size(); ++i)
	{
		if (is_mimic_[i])
		{
			// 跟随关节: 直接由源关节推导, 不做速度限幅积分
			const size_t src = mimic_source_[i];
			state_position_[i] = mimic_multiplier_[i] * state_position_[src] + mimic_offset_[i];
			state_velocity_[i] = mimic_multiplier_[i] * state_velocity_[src];
			state_effort_[i] = 0.0;
			continue;
		}

		const double target = std::clamp(cmd_position_[i], joint_lower_[i], joint_upper_[i]);
		const double error = target - state_position_[i];
		const double max_step = joint_max_velocity_[i] * dt;
		const double step =
		(std::abs(error) <= max_step) ? error : std::copysign(max_step, error);

		state_position_[i] = std::clamp(state_position_[i] + step, joint_lower_[i], joint_upper_[i]);
		state_velocity_[i] = step / dt;
		state_effort_[i] = 0.0;
	}

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockPiperHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	return hardware_interface::return_type::OK;
}

}  // namespace unistackbot_hardware_mock

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  unistackbot_hardware_mock::MockPiperHardware,
  hardware_interface::SystemInterface)

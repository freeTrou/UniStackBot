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

const std::unordered_map<std::string, double> JOINT_MAX_VELOCITY = {
  {"joint1", 5.0}, {"joint2", 5.0}, {"joint3", 5.0},
  {"joint4", 5.0}, {"joint5", 5.0}, {"joint6", 5.0}, {"gripper", 3.0},
};

const std::unordered_map<std::string, bool> JOINT_HAS_EFFORT = {
  {"joint1", true}, {"joint2", true}, {"joint3", true},
  {"joint4", true}, {"joint5", true}, {"joint6", true}, {"gripper", false},
};

double parse_param_or(const std::unordered_map<std::string, std::string> & params,
                      const std::string & key, double fallback)
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

hardware_interface::CallbackReturn MockPiperHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints.size() != JOINT_COUNT)
  {
    RCLCPP_ERROR(
      LOGGER, "Expected %zu joints, got %zu", JOINT_COUNT, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  cmd_position_.assign(JOINT_COUNT, 0.0);
  state_position_.assign(JOINT_COUNT, 0.0);
  state_velocity_.assign(JOINT_COUNT, 0.0);
  state_effort_.assign(JOINT_COUNT, 0.0);
  joint_lower_.resize(JOINT_COUNT);
  joint_upper_.resize(JOINT_COUNT);
  joint_max_velocity_.resize(JOINT_COUNT);

  for (size_t i = 0; i < JOINT_COUNT; ++i)
  {
    const auto & joint = info.joints[i];
    const auto & params = joint.parameters;

    joint_lower_[i] = parse_param_or(params, "min", -std::numeric_limits<double>::infinity());
    joint_upper_[i] = parse_param_or(params, "max", std::numeric_limits<double>::infinity());

    auto vel_it = JOINT_MAX_VELOCITY.find(joint.name);
    joint_max_velocity_[i] = (vel_it != JOINT_MAX_VELOCITY.end())
      ? vel_it->second
      : std::numeric_limits<double>::infinity();

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
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
  interfaces.reserve(JOINT_COUNT * 3);

  for (size_t i = 0; i < JOINT_COUNT; ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    if (has_effort_at(i))
    {
      interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MockPiperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(JOINT_COUNT);
  for (size_t i = 0; i < JOINT_COUNT; ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn MockPiperHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockPiperHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockPiperHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MockPiperHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0)
  {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < JOINT_COUNT; ++i)
  {
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

hardware_interface::return_type MockPiperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace unistackbot_hardware_mock

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  unistackbot_hardware_mock::MockPiperHardware,
  hardware_interface::SystemInterface)

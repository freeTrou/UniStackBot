#ifndef UNISTACKBOT_HARDWARE_MOCK__MOCK_PIPER_HARDWARE_HPP_
#define UNISTACKBOT_HARDWARE_MOCK__MOCK_PIPER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace unistackbot_hardware_mock
{

class MockPiperHardware
  : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface,
    public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockPiperHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  const size_t JOINT_COUNT = 7;

  std::vector<double> cmd_position_;
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> state_effort_;

  std::vector<double> joint_lower_;
  std::vector<double> joint_upper_;
  std::vector<double> joint_max_velocity_;

  bool has_effort_at(size_t index) const;
};

}  // namespace unistackbot_hardware_mock

#endif  // UNISTACKBOT_HARDWARE_MOCK__MOCK_PIPER_HARDWARE_HPP_

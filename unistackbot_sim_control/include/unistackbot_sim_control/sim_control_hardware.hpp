#ifndef UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_HARDWARE_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_HARDWARE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unistackbot_sim_control/backend.hpp"
#include "unistackbot_sim_control/sim_command.hpp"
#include "unistackbot_sim_control/sim_control_server.hpp"

namespace unistackbot_sim_control
{

/// 统一仿真控制层硬件插件 —— ros2_control 调用的入口。
/// 对外: 对 ros2_control 提供统一接口 (position 命令 + pos/vel/effort 状态);
/// 对内: 按 "backend" 参数分类处理 (kinematic/...), 新仿真平台 = 新后端;
/// 同时承载 /sim_control/* 仿真控制服务 (reset/set_joint_state/pause/resume/step)。
class SimControlHardware : public hardware_interface::SystemInterface
{
public:
	RCLCPP_SHARED_PTR_DEFINITIONS(SimControlHardware)

	hardware_interface::CallbackReturn on_init(
		const hardware_interface::HardwareInfo & info) override;

	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

	hardware_interface::CallbackReturn on_configure(
		const rclcpp_lifecycle::State & previous_state) override;

	hardware_interface::CallbackReturn on_cleanup(
		const rclcpp_lifecycle::State & previous_state) override;

	hardware_interface::return_type read(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;

	hardware_interface::return_type write(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	/// 排空无锁队列, 按序应用 /sim_control 命令 (RT 线程内执行)
	void drainCommands();

	// 关节元数据与接口内存
	std::vector<JointMeta> joints_;
	std::vector<double> cmd_position_;
	std::vector<double> state_position_;
	std::vector<double> state_velocity_;
	std::vector<double> state_effort_;

	// 后端 (对内分类: kinematic / 未来 mujoco / ...)
	std::unique_ptr<SimBackend> backend_;

	// /sim_control 服务 (非实时线程)
	rclcpp::Node::SharedPtr svc_node_;
	rclcpp::executors::SingleThreadedExecutor::SharedPtr svc_executor_;
	std::thread svc_thread_;
	std::unique_ptr<SimControlServer> server_;
	SpscRing<SimCommand, kQueueCapacity> cmd_queue_;
	std::atomic<bool> paused_{false};
	std::atomic<uint32_t> step_req_{0};
	std::atomic<bool> svc_running_{false};

	// 服务校验器/下沉函数用的回调
	bool validateSetState(const std::vector<std::string> & names,
		const std::vector<double> & positions,
		const std::vector<double> & velocities,
		SimCommand & command,
		std::string & message);
	bool enqueueSink(const SimCommand & cmd, std::string & message);
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_HARDWARE_HPP_

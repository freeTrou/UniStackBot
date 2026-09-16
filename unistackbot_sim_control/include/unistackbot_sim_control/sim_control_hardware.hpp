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

#include "unistackbot_sim_control/sim_backend.hpp"
#include "unistackbot_sim_control/sim_command_queue.hpp"
#include "unistackbot_interface/sim_control_contract.hpp"

namespace unistackbot_sim_control
{

/*
 * 统一仿真控制层硬件插件 —— ros2_control 调用的入口。
 * 对外: 对 ros2_control 提供统一接口 (position 命令 + pos/vel/effort 状态);
 * 对内: 按 "backend" 参数分类处理 (kinematic/...), 新仿真平台 = 新后端;
 * 同时承载 /sim_control 全部仿真控制服务 (reset/set_joint_state/pause/resume/step)。
 */
class SimControlHardware : public hardware_interface::SystemInterface
{
public:
	RCLCPP_SHARED_PTR_DEFINITIONS(SimControlHardware)
	
	//解析 <ros2_control> 块: 接口契约校验 + mimic 解析 + 后端构造; 契约违背硬失败
	hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
	//状态接口镜像 URDF 声明 (声明什么导出什么, effort 恒 0)
	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
	//命令接口固定契约: 每关节恰一个 position
	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
	//拉起 /sim_control 服务线程 (与 on_cleanup 对称)
	hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	//激活 (INACTIVE → ACTIVE): RT 循环由此启动; 默认 SUCCESS + 日志
	//将来真机驱动在重写此处做首拍同步 (cmd = state, 防上电猛冲)
	hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	//停用 (ACTIVE → INACTIVE): RT 循环停止; 默认 SUCCESS + 日志
	hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
	//停服务线程并回收 (先降旗标再 join, 有界退出)
	hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
	//进程退出的实际路径: deactivate → shutdown 直接进 FINALIZED, 不经过 cleanup —— 线程必须在此收
	hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
	//RT 周期入口: 排空命令队列 + 后端推进一个周期
	hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
	//运动学后端无外部设备 —— 空实现, 命令已在 read() 消化
	hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	//排空无锁队列, 按序应用 /sim_control 命令 (RT 线程内执行)
	void drainCommands();

	// ---- 后端访问收口: backend_ 只在 on_init(创建处) 与以下收口方法中出现 ----
	//起 plant (异步后端在此起线程; 失败则激活失败, RT 循环不启动)
	bool activateBackend();
	//停 plant (异步后端在此停线程, 有界 join)
	void deactivateBackend();
	//后端一拍驱动 (read() 内, 全类唯一 step 调用点; dt=0 且 integrate=false = 只重算 mimic 派生量)
	void stepBackend(double dt, bool integrate);
	//发命令给 plant (write() 内; 同步后端 = 空操作)
	void writeBackend();
	//状态直写下发 plant (瞬移/回零用; 同步后端 = 空操作)
	void stateBackend();

	//校验 set_joint_state 并展开为按关节索引的命令 (未知关节/mimic/超限即拒绝, 不入队)
	[[nodiscard]] bool validateSetState(const std::vector<std::string> & names, const std::vector<double> & positions, 
		const std::vector<double> & velocities, SimCommand & command, std::string & message);
	
	//命令入队 (非阻塞; 队列满即拒绝, RT 循环绝不等待)
	[[nodiscard]] bool enqueueSink(const SimCommand & cmd, std::string & message);

	//停收 /sim_control 服务线程 (幂等; on_cleanup 与 on_shutdown 共用)
	void stopServices();

private:
	// ---- 数据成员 (集中一块) ----
	// 关节元数据与接口内存 (RT 循环每周期读写)
	std::vector<JointMeta> joints_;
	std::vector<double> cmd_position_;
	std::vector<double> state_position_;
	std::vector<double> state_velocity_;
	std::vector<double> state_effort_;

	// 后端 (对内分类: kinematic / 未来 mujoco / ...) (RT)
	std::unique_ptr<SimBackend> backend_;

	// /sim_control 服务 (非实时线程)
	rclcpp::Node::SharedPtr service_node_;
	rclcpp::executors::SingleThreadedExecutor::SharedPtr service_executor_;
	std::thread service_thread_;
	std::unique_ptr<SimControlServer> server_;              // /sim_control 服务门面
	SpscRing<SimCommand, kQueueCapacity> cmd_queue_;        // 命令队列 (生产: service 线程 / 消费: RT read)
	std::atomic<bool> paused_{false};                       // 冻结状态推进 (mimic 仍按源推导)
	std::atomic<uint32_t> step_requests_{0};                // 暂停期间待处理的单步请求数
	std::atomic<bool> service_running_{false};              // 服务线程退出旗标 (false 时线程自查退出)
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_HARDWARE_HPP_

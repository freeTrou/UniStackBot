#ifndef UNISTACKBOT_BRINGUP__BUS_SYSTEM_HPP_
#define UNISTACKBOT_BRINGUP__BUS_SYSTEM_HPP_
/*
 * BusSystem —— 通用总线 SystemInterface 适配器 (2026-11-02, 用户裁决住组合根 bringup):
 * "全工程唯一碰 ROS 的总线路径" (master_base.hpp 契约) 的**单数**实现——一条代码服务
 * 所有 MasterBase 总线, 总线差异全部住配置与总线包。
 *
 * 组合根模式: bringup 知道所有部件——本类的 registerKnownBuses() 链接各总线包的
 * 工厂注册 (加总线 = 加一行); 总线包保持零 ROS 纯骨架, 不反向依赖 bringup。
 *
 * URDF <param> (机型侧声明, 适配器零机型知识):
 *   bus          (必填) 总线名 = createMaster 工厂 key (udp | serial | canfd | ...)
 *   endpoint     (必填) 总线端点, 格式由总线解释 (串口=路径 / CAN=接口名 / UDP=ip:port[:bind])
 *   rate_hz      总线拍频 (默认 500)
 *   protocol     段级协议名 (默认空 → 主站自行校验; 节点级可覆写)
 *   statemachine 段级状态机名 (同上)
 * 节点表 = URDF joint 声明序 (槽位 = 序号; state-only 关节 = 只读节点)。
 */
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "unistackbot_bus/master_base.hpp"

namespace unistackbot_bringup
{

class BusSystem : public hardware_interface::SystemInterface
{
public:
	RCLCPP_SHARED_PTR_DEFINITIONS(BusSystem)

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

	// 组合根: 链接全部已知总线的工厂注册 (加总线 = 加一行; 本地节点 cpp 内实现)
	static void registerKnownBuses();

private:
	std::unique_ptr<unistackbot_bus::MasterBase> master_;
	std::vector<unistackbot_bus::NodeSnapshot> snap_;
	std::vector<unistackbot_protocol::NodeCommand> cmd_tx_;
	std::vector<double> cmd_pos_;
	std::vector<double> state_pos_;
	std::vector<double> state_vel_;
	std::vector<double> state_eff_;
	std::vector<std::string> joint_names_;
};

}  // namespace unistackbot_bringup
#endif  // UNISTACKBOT_BRINGUP__BUS_SYSTEM_HPP_

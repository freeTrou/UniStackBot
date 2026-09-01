#ifndef UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_SERVER_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_SERVER_HPP_

#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "unistackbot_sim_control/sim_command.hpp"

namespace unistackbot_sim_control
{

/// /sim_control/* 统一服务门面。
/// 在给定节点上创建 5 个绝对名服务:
///   /sim_control/reset            std_srvs/srv/Trigger
///   /sim_control/set_joint_state  unistackbot_sim_control/srv/SetJointState
///   /sim_control/pause            std_srvs/srv/Trigger
///   /sim_control/resume           std_srvs/srv/Trigger
///   /sim_control/step             std_srvs/srv/Trigger
/// 五个服务挂同一个 MutuallyExclusiveCallbackGroup —— 回调串行化,
/// 保证命令队列的严格单生产者前提。
///
/// 应答语义: success=true 表示命令已被后端"接受"(入队 / ign 请求已发出),
/// 不代表执行完成。
class SimControlServer
{
public:
	/// 命令下沉函数: 后端决定命令去向 (mock=入队, gz=ign 服务调用)。
	/// 返回 false + 原因 = 拒绝 (队列满 / 后端不支持), 会透传进服务应答。
	using Sink = std::function<bool(const SimCommand &, std::string & message)>;

	/// set_joint_state 校验器: 关节存在性 / 非 mimic / 限位, 后端特有。
	/// 校验通过后由后端把请求展开成命令 (mask/positions/velocities 按关节索引填充)。
	/// 返回 false + 原因时命令不会入队。
	using SetStateValidator = std::function<bool(
		const std::vector<std::string> & names,
		const std::vector<double> & positions,
		const std::vector<double> & velocities,
		SimCommand & command,
		std::string & message)>;

	SimControlServer(const rclcpp::Node::SharedPtr & node,
		Sink sink, SetStateValidator validate_set_state);

private:
	rclcpp::CallbackGroup::SharedPtr callback_group_;
	std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_SERVER_HPP_

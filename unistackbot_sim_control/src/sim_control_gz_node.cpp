/// Gazebo (Fortress) 后端适配器: 把统一仿真控制契约翻译成 ign 世界服务。
///
/// 能力矩阵 (诚实声明):
///   pause / resume / step  -> /world/<world>/control (WorldControl)  [支持]
///   reset / set_joint_state -> Fortress 无原生等价                    [拒绝并说明]
///
/// 参数: world (string, 默认 piper_world)

#include <functional>
#include <memory>
#include <string>

#include "ignition/msgs/boolean.pb.h"
#include "ignition/msgs/world_control.pb.h"
#include "ignition/transport.hh"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "unistackbot_sim_control/sim_control_server.hpp"
#include "unistackbot_sim_control/srv/set_joint_state.hpp"

using unistackbot_sim_control::SimCmdType;
using unistackbot_sim_control::SimCommand;
using unistackbot_sim_control::SimControlServer;

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("sim_control_gz");
	const std::string world =
		node->declare_parameter<std::string>("world", "piper_world");
	const std::string control_srv = "/world/" + world + "/control";

	ignition::transport::Node gz_node;
	auto logger = node->get_logger();

	auto sink = [&](const SimCommand & cmd, std::string & message) -> bool {
			ignition::msgs::WorldControl wc;
			switch (cmd.type) {
				case SimCmdType::PAUSE:
					wc.set_pause(true);
					break;
				case SimCmdType::RESUME:
					wc.set_pause(false);
					break;
				case SimCmdType::STEP:
					wc.set_step(true);
					break;
				default:
					message = "gz adapter: 仅支持 pause/resume/step "
						"(reset/set_joint_state 在 Fortress 无原生等价)";
					return false;
			}
			// 异步发出请求; 应答只代表"已接受"。
			// 注意: std::function 重载按非 const 左值引用收回调, 必须传具名变量;
			// 该重载不带 timeout 参数 ( transport11 的签名如此 )
			std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
				[logger](const ignition::msgs::Boolean & rep, const bool result) {
					if (!result || !rep.data()) {
						RCLCPP_WARN(logger, "ign 世界服务调用未成功确认");
					}
				};
			gz_node.Request<ignition::msgs::WorldControl, ignition::msgs::Boolean>(
				control_srv, wc, cb);
			message = "已发送 " + control_srv;
			return true;
		};

	auto reject_all = [](const std::vector<std::string> &,
		const std::vector<double> &,
		const std::vector<double> &,
		SimCommand &,
		std::string & message) {
			message = "gz adapter: 不支持 set_joint_state "
				"(Fortress 无关节空间瞬移, Garden+ 可用)";
			return false;
		};

	SimControlServer server(node, sink, reject_all);

	RCLCPP_INFO(logger, "sim_control_gz 就绪 (world=%s, 控制服务 %s)",
		world.c_str(), control_srv.c_str());
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

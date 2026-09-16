/*
 * Gazebo (Fortress) 后端适配器: 把统一仿真控制契约翻译成 ROS 桥接的世界控制服务。
 *
 * 能力矩阵 (诚实声明, 2026-09-16 实测):
 *   pause / resume / step  -> /world/<world>/control (经 ros_gz_bridge 桥接的
 *                             ros_gz_interfaces/srv/ControlWorld)            [支持]
 *   reset                  -> Fortress 实测有毒: 服务返回 success=true 但世界随即
 *                             进入 negative-timestep 错误态、仿真停摆; 关节复位
 *                             请发 JTC 归零轨迹, Garden+ 才有可用 reset       [拒绝并说明]
 *   set_joint_state        -> Fortress 无关节空间瞬移 (Garden+ 可用)          [拒绝并说明]
 *
 * 本节点是纯 ROS 进程: 无 ignition 编译依赖, ign 侧全部经由 launch 里的
 * parameter_bridge (话题 /clock、/stats + 服务 /world/<world>/control)。
 *
 * 参数: world (string, 默认 unistack_world) —— 须与 world 文件里的 <world name>
 *       及 launch 里桥接的服务名一致。
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros_gz_interfaces/srv/control_world.hpp"
#include "ulog/ulog.hpp"
#include "unistackbot_sim_control/sim_control_contract.hpp"

using ControlWorld = ros_gz_interfaces::srv::ControlWorld;
using unistackbot_sim_control::SimCmdType;
using unistackbot_sim_control::SimCommand;
using unistackbot_sim_control::SimControlServer;

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("sim_control_gz");
	const std::string world = node->declare_parameter<std::string>("world", "unistack_world");
	const std::string control_service = "/world/" + world + "/control";

	auto logger = node->get_logger();
	auto world_control = node->create_client<ControlWorld>(control_service);

	auto sink = [&](const SimCommand & cmd, std::string & message) -> bool
	{
		auto request = std::make_shared<ControlWorld::Request>();
		switch (cmd.type)
		{
			case SimCmdType::PAUSE:
			{
				request->world_control.pause = true;
			}
			break;
			case SimCmdType::RESUME:
			{
				request->world_control.pause = false;
			}
			break;
			case SimCmdType::STEP:
			{
				// 契约语义: step = 暂停状态下推进一步; 同时置 pause 保证调用者
				// 未先 pause 时行为也确定 (世界运行中收到 step -> 暂停并推进一步)
				request->world_control.pause = true;
				request->world_control.step = true;
			}
			break;
			default:
			{
				message = "gz adapter: reset 在 Fortress 实测有毒 —— 世界控制服务"
					"返回 success=true 但世界随即进入 negative-timestep 错误态、"
					"仿真停摆; 关节复位请发 JTC 归零轨迹 (Garden+ 才有可用 reset)";
				return false;
			}
		}
		if (!world_control->service_is_ready())
		{
			message = "gz 世界控制服务未就绪 (gz server 未起或桥接未建立): " + control_service;
			return false;
		}
		// 异步发出请求; 应答只代表"已接受" (契约语义), 结果经回调告警
		world_control->async_send_request(request,
			[logger](rclcpp::Client<ControlWorld>::SharedFuture future)
			{
				const auto & response = future.get();
				if (!response->success)
				{
					ULOG_WARN("世界控制服务应答 success=false");
				}
			});
		message = "已发送 " + control_service;
		return true;
	};

	auto reject_all = [](const std::vector<std::string> &, const std::vector<double> &, const std::vector<double> &, SimCommand &, std::string & message)
	{
		message = "gz adapter: 不支持 set_joint_state "
			"(Fortress 无关节空间瞬移, Garden+ 可用)";
		return false;
	};

	SimControlServer server(node, sink, reject_all);

	// 启动预热: gz server / 桥接可能晚于本节点起。只等待+告警、不致命
	// (不在 sink 内阻塞等待 —— sink 跑在互斥回调组, 阻塞会卡住全部 /sim_control 服务);
	// 未就绪期间每次调用仍有 service_is_ready 前置检查兜底
	if (world_control->wait_for_service(std::chrono::seconds(10)))
	{
		ULOG_INFO("sim_control_gz 就绪 (world=%s, 控制服务 %s)", world.c_str(), control_service.c_str());
	}
	else
	{
		ULOG_WARN("世界控制服务 10s 未出现 (%s); 链路继续, 调用时将再检", control_service.c_str());
	}
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

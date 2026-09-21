/*
 * MuJoCo 后端适配器 (0d, 2026-09-21): 把统一仿真控制契约翻译成 mujoco_ros2_control
 * 定制节点自带的原生服务 (/mujoco_ros2_control_node/*)。
 *
 * 能力矩阵 (对照 gz 适配器, 诚实声明):
 *   pause / resume -> SetPause(paused=true/false)                              [支持]
 *   step           -> StepSimulation(steps=1); 语义=暂停态推进一步 (调用者先 pause,
 *                     与契约一致; 运行态收到会在运行中多步一步, 无损)          [支持]
 *   reset          -> ResetWorld (空 keyframe = 复位到启动时捕获的初态;
 *                     原生支持 —— 比 gz 强, Fortress 的 reset 实测有毒只能拒)
 *                     注意语义: 世界级复位 (含仿真时间回卷 + 控制器命令保持位
 *                     每拍覆写, 复位后命令通道会持续拉回 —— 需配合 pause 使用) [支持]
 *   set_joint_state-> 拒绝: 原生无运行期关节瞬移 (ResetWorld 的 state_overrides
 *                     可以带关节覆写, 但它是世界级复位+时间回卷, 语义≠瞬移)   [拒绝并说明]
 *
 * 本节点纯 ROS 进程零 MuJoCo 编译依赖 (服务类型来自 mujoco_ros2_control_msgs),
 * 契约头从 unistackbot_sim_control include (依赖方向 mujoco → sim_control, 同 gz)。
 * 由 mujoco.launch.py 启动。
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mujoco_ros2_control_msgs/srv/reset_world.hpp"
#include "mujoco_ros2_control_msgs/srv/set_pause.hpp"
#include "mujoco_ros2_control_msgs/srv/step_simulation.hpp"
#include "ulog/ulog.hpp"
#include "unistackbot_sim_control/sim_control_contract.hpp"

using ResetWorld = mujoco_ros2_control_msgs::srv::ResetWorld;
using SetPause = mujoco_ros2_control_msgs::srv::SetPause;
using StepSimulation = mujoco_ros2_control_msgs::srv::StepSimulation;
using unistackbot_sim_control::SimCmdType;
using unistackbot_sim_control::SimCommand;
using unistackbot_sim_control::SimControlServer;

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("sim_control_mujoco");
	const std::string ns = "/mujoco_ros2_control_node";

	auto logger = node->get_logger();
	auto pause_cli = node->create_client<SetPause>(ns + "/set_pause");
	auto step_cli = node->create_client<StepSimulation>(ns + "/step_simulation");
	auto reset_cli = node->create_client<ResetWorld>(ns + "/reset_world");

	// 应答只代表"已接受" (契约语义); 结果经回调告警 (与 gz 适配器同款)
	auto sink = [&](const SimCommand & cmd, std::string & message) -> bool
	{
		switch (cmd.type)
		{
			case SimCmdType::PAUSE:
			{
				auto req = std::make_shared<SetPause::Request>();
				req->paused = true;
				if (!pause_cli->service_is_ready())
				{
					message = "mujoco set_pause 服务未就绪 (链未起?)";
					return false;
				}
				pause_cli->async_send_request(req);
				message = "已发送 " + ns + "/set_pause(true)";
				return true;
			}
			case SimCmdType::RESUME:
			{
				auto req = std::make_shared<SetPause::Request>();
				req->paused = false;
				if (!pause_cli->service_is_ready())
				{
					message = "mujoco set_pause 服务未就绪 (链未起?)";
					return false;
				}
				pause_cli->async_send_request(req);
				message = "已发送 " + ns + "/set_pause(false)";
				return true;
			}
			case SimCmdType::STEP:
			{
				auto req = std::make_shared<StepSimulation::Request>();
				req->steps = 1;
				if (!step_cli->service_is_ready())
				{
					message = "mujoco step_simulation 服务未就绪 (链未起?)";
					return false;
				}
				step_cli->async_send_request(req);
				message = "已发送 " + ns + "/step_simulation(1) — 语义=暂停态推进一步";
				return true;
			}
			case SimCmdType::RESET:
			{
				auto req = std::make_shared<ResetWorld::Request>();
				// 空 keyframe = 启动时捕获的初态; 不用 state_overrides (那是世界级复位)
				if (!reset_cli->service_is_ready())
				{
					message = "mujoco reset_world 服务未就绪 (链未起?)";
					return false;
				}
				reset_cli->async_send_request(req,
					[logger](rclcpp::Client<ResetWorld>::SharedFuture future)
					{
						const auto & response = future.get();
						if (!response->success)
						{
							ULOG_WARN("reset_world 应答 success=false: %s",
								response->message.c_str());
						}
					});
				message = "已发送 " + ns + "/reset_world (世界级复位含时间回卷; "
					"控制器命令保持位会拉回, 建议配合 pause 使用)";
				return true;
			}
			default:
			{
				message = "不应到达";
				return false;
			}
		}
	};

	auto reject_all = [](const std::vector<std::string> &,
		const std::vector<double> &, const std::vector<double> &,
		SimCommand &, std::string & message)
	{
		message = "mujoco adapter: 不支持 set_joint_state 运行期瞬移 "
			"(原生无等价; ResetWorld 的 state_overrides 是世界级复位+时间回卷, "
			"语义≠瞬移; MJCF keyframe 经 reset 使用)";
		return false;
	};

	SimControlServer server(node, sink, reject_all);

	// 启动预热: 定制节点可能晚于本节点起。只等待+告警、不致命
	// (sink 跑在互斥回调组, 不在其中阻塞 —— gz 适配器同款纪律)
	if (pause_cli->wait_for_service(std::chrono::seconds(10)))
	{
		ULOG_INFO("sim_control_mujoco 就绪 (原生服务 %s/*)", ns.c_str());
	}
	else
	{
		ULOG_WARN("mujoco 原生服务 10s 未出现 (%s/*); 链路继续, 调用时将再检",
			ns.c_str());
	}
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

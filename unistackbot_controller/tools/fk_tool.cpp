/*
 * fk_tool —— FK 命令行工具 (调试与 TF 对拍的弹药)。
 *
 * 用法:
 *   ros2 run unistackbot_controller fk_tool --joints j1=0.5,j2=-0.3,...
 * 关节表读自运行链路的 /joint_trajectory_controller 参数; URDF 读自
 * /robot_state_publisher 参数 (链路活着时始终可用); 拓扑 = link_base -> link7
 * 由机型约定给出 (tip 可 --tip 覆盖)。
 * 输出行格式 (对拍脚本解析):
 *   position [x, y, z]
 *   quaternion [w, x, y, z]
 */
#include <cstdio>
#include <string>
#include <vector>

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include "urdf_fk/urdf_fk.hpp"

int main(int argc, char ** argv)
{
	std::string joints_arg, tip = "link7", base = "link_base";
	for (int i = 1; i < argc; ++i)
	{
		const std::string a = argv[i];
		if (a == "--joints" && i + 1 < argc)
		{
			joints_arg = argv[++i];
		}
		else if (a == "--tip" && i + 1 < argc)
		{
			tip = argv[++i];
		}
		else if (a == "--base" && i + 1 < argc)
		{
			base = argv[++i];
		}
	}
	if (joints_arg.empty())
	{
		std::fprintf(stderr, "用法: fk_tool --joints j1=0.5,... [--base link_base] [--tip link7]\n");
		return 2;
	}

	// 解析 j=值 表 (保持给出顺序)
	std::vector<std::string> names;
	std::vector<double> values;
	{
		size_t pos = 0;
		while (pos < joints_arg.size())
		{
			const size_t comma = joints_arg.find(',', pos);
			const std::string item = joints_arg.substr(pos, comma - pos);
			const size_t eq = item.find('=');
			if (eq == std::string::npos)
			{
				std::fprintf(stderr, "坏参数: %s\n", item.c_str());
				return 2;
			}
			names.push_back(item.substr(0, eq));
			values.push_back(std::stod(item.substr(eq + 1)));
			if (comma == std::string::npos) {break;}
			pos = comma + 1;
		}
	}

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("fk_tool");
	auto get_param = [&](const std::string & node_name, const std::string & name)
	{
		auto cli = node->create_client<rcl_interfaces::srv::GetParameters>(node_name + "/get_parameters");
		if (!cli->wait_for_service(std::chrono::seconds(5)))
		{
			std::fprintf(stderr, "参数服务未就绪: %s (链路未起?)\n", node_name.c_str());
			std::exit(1);
		}
		auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
		req->names = {name};
		auto fut = cli->async_send_request(req);
		if (rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(5)) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			std::fprintf(stderr, "读参数超时: %s.%s\n", node_name.c_str(), name.c_str());
			std::exit(1);
		}
		return fut.get()->values[0];
	};

	// 关节序按 URDF 链序对齐: fk_tool 按 FK 库的关节顺序出解, 名字顺序由参数给出
	// ——这里要求给出的顺序即链序 (脚本端由 URDF 顺序保证)
	// (2026-09-21 去 JTC 参数依赖: 链上已无 joint_trajectory_controller, robot_description 已足够)
	const auto urdf_param = get_param("/robot_state_publisher", "robot_description");
	if (urdf_param.string_value.empty() && urdf_param.byte_array_value.empty())
	{
		std::fprintf(stderr, "robot_description 为空\n");
		return 1;
	}

	// 先 init 拿链序关节名, 再把用户给的 j=值 按链序重排
	const std::string urdf_text = urdf_param.string_value;
	unistackbot_algorithm::UrdfFk fk;
	std::string msg;
	if (!fk.init(urdf_text, base, tip, msg))
	{
		std::fprintf(stderr, "%s\n", msg.c_str());
		return 1;
	}
	std::vector<double> q;
	for (const auto & chain_name : fk.jointNames())
	{
		bool found = false;
		for (size_t k = 0; k < names.size(); ++k)
		{
			if (names[k] == chain_name)
			{
				q.push_back(values[k]);
				found = true;
				break;
			}
		}
		if (!found)
		{
			std::fprintf(stderr, "缺关节 %s 的值\n", chain_name.c_str());
			return 1;
		}
	}
	if (q.size() != names.size())
	{
		std::fprintf(stderr, "给定的关节名与 URDF 不匹配 (%zu/%zu)\n", q.size(), names.size());
		return 1;
	}

	unistackbot_algorithm::CartesianPose p;
	if (!fk.fk(q, p))
	{
		std::fprintf(stderr, "FK 失败: 关节数 %zu vs %u\n", q.size(), fk.jointCount());
		return 1;
	}

	std::printf("position [%.12f, %.12f, %.12f]\n", p.x, p.y, p.z);
	std::printf("quaternion [%.12f, %.12f, %.12f, %.12f]\n", p.qw, p.qx, p.qy, p.qz);
	rclcpp::shutdown();
	return 0;
}

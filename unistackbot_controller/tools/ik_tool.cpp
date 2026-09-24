/*
 * ik_tool —— IK 命令行工具 (调试 + RViz 演示用)。
 *
 * 用法:
 *   ros2 run unistackbot_controller ik_tool --pos 0.35,0.1,0.45 --quat 0,1,0,0
 * 关节表/URDF 读自运行链路 (同 fk_tool); 种子 = 限位中心 (冷启动)。
 * 输出:
 *   joints [q1, ..., qn]
 *   fk_error <位置误差 m>   (解回代闭合, 自证)
 */
#include <cstdio>
#include <string>
#include <vector>

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dls_ik/dls_ik.hpp"

int main(int argc, char ** argv)
{
	std::string pos_arg, quat_arg, tip, base;
	for (int i = 1; i < argc; ++i)
	{
		const std::string a = argv[i];
		if (a == "--pos" && i + 1 < argc) {pos_arg = argv[++i];}
		else if (a == "--quat" && i + 1 < argc) {quat_arg = argv[++i];}
		else if (a == "--tip" && i + 1 < argc) {tip = argv[++i];}
		else if (a == "--base" && i + 1 < argc) {base = argv[++i];}
	}
	// 机型/拓扑参数无默认值纪律 (2026-09-23 修: 旧默认 link_base/link7 是 xarm7
	// 约定的隐性硬编码, piper 消费者撞默认即建链失败)
	if (pos_arg.empty() || quat_arg.empty() || base.empty() || tip.empty())
	{
		std::fprintf(stderr, "用法: ik_tool --pos x,y,z --quat w,x,y,z --base <基座link> --tip <末端link>\n"
			"  (拓扑无默认, 必填; 例 piper: --base base_link --tip link6; xarm7: --base link_base --tip link7)\n");
		return 2;
	}
	const auto parse3or4 = [](const std::string & s, std::vector<double> & out)
	{
		size_t pos = 0;
		while (pos <= s.size())
		{
			const size_t comma = s.find(',', pos);
			out.push_back(std::stod(s.substr(pos, comma - pos)));
			if (comma == std::string::npos) {break;}
			pos = comma + 1;
		}
	};
	std::vector<double> pos, quat;
	parse3or4(pos_arg, pos);
	parse3or4(quat_arg, quat);
	if (pos.size() != 3 || quat.size() != 4)
	{
		std::fprintf(stderr, "--pos 需 3 个数, --quat 需 4 个数 (w,x,y,z)\n");
		return 2;
	}

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("ik_tool");
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
			std::fprintf(stderr, "读参数超时\n");
			std::exit(1);
		}
		return fut.get()->values[0];
	};
	const auto urdf_text = get_param("/robot_state_publisher", "robot_description").string_value;
	if (urdf_text.empty())
	{
		std::fprintf(stderr, "robot_description 为空\n");
		return 1;
	}

	unistackbot_algorithm::UrdfFk fk;
	std::string msg;
	if (!fk.init(urdf_text, base, tip, msg))
	{
		std::fprintf(stderr, "%s\n", msg.c_str());
		return 1;
	}

	unistackbot_algorithm::DlsIk ik;
	if (!ik.init(&fk, msg))
	{
		std::fprintf(stderr, "%s\n", msg.c_str());
		return 1;
	}

	unistackbot_algorithm::CartesianPose target;
	target.x = pos[0]; target.y = pos[1]; target.z = pos[2];
	target.qw = quat[0]; target.qx = quat[1]; target.qy = quat[2]; target.qz = quat[3];

	// 种子 = 限位中心 (冷启动)
	const unsigned n = fk.jointCount();
	std::vector<double> seed(n);
	for (unsigned i = 0; i < n; ++i)
	{
		seed[i] = (fk.qMin()[i] + fk.qMax()[i]) / 2.0;
	}

	unistackbot_algorithm::RedundancyPreference pre;
	std::vector<double> q;
	const auto r = ik.solve(target, seed, pre, q);
	if (r != unistackbot_algorithm::IkResult::OK)
	{
		std::fprintf(stderr, "求解失败: %s\n", unistackbot_interface::ik_result_message(r));
		return 1;
	}

	unistackbot_algorithm::CartesianPose back;
	if (!fk.fk(q, back))
	{
		std::fprintf(stderr, "解回代 FK 失败\n");
		return 1;
	}
	const double err = std::sqrt(std::pow(back.x - target.x, 2) + std::pow(back.y - target.y, 2) +
		std::pow(back.z - target.z, 2));

	std::printf("joints [");
	for (unsigned i = 0; i < n; ++i)
	{
		std::printf("%s%.9f", i ? ", " : "", q[i]);
	}
	std::printf("]\n");
	std::printf("fk_error %.2e\n", err);
	rclcpp::shutdown();
	return 0;
}

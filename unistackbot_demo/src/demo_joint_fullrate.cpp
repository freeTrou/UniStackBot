// demo_joint_fullrate —— 关节满速流 demo (命令域③: 命令频率 = 控制环/总线频率, 透传路径)。
//
// 三命令域 (设计 §16.2 透传原则): ① demo_cartesian.py 末端位姿流 ② demo_motion.py
// 关节慢流 (< 控制频率, JS hold 填充) ③ 本程序 关节满速流 (= 控制频率, hold 退化 =
// 透传)。**C++ 实现的理由**: python rclpy 发布 500Hz 全是抖动 (100Hz 稳/200Hz 勉强),
// 满速档必须是真调度 —— steady_clock sleep_until 周期发布。
//
// 机型/链无关: 关节表+限位+mimic 解析自 robot_state_publisher 的 robot_description
// (demo 自持解析 —— 外部消费者独立消费契约, 同 py demo 口径); 控制频率自校准
// (/joint_states 到达周期中位数, ≈ JSB 发布率 = update_rate)。
//
// 前置: 三链任一已启动且 joint_stream_controller active。
// 用法: ros2 run unistackbot_demo demo_joint_fullrate [--ros-args -p duration:=8.0 -p hz:=0.0]
//   hz=0 (默认) 自校准; 显式指定则以指定值发布 (实际达成率结束时如实报告)。
// 运动内容: 实测位 → 各直控关节向限位区间中位偏移 30% (正弦过渡) → 回实测位。
// 收尾: 停流即走 JS 断流受控减速 (yaml stale_timeout)。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tinyxml2.h>

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unistackbot_interface/msg/joint_command.hpp"

namespace
{

struct JointInfo
{
	std::string name;
	double qmin{0.0};
	double qmax{0.0};
	bool has_limits{false};
};
struct MimicInfo
{
	std::string name;
	std::string master;
	double k{1.0};
	double off{0.0};
};

// URDF <ros2_control> 解析: position 命令关节 + 限位 + mimic (与 JS 消费口径一致)
bool parseUrdf(const std::string & urdf, std::vector<JointInfo> & joints,
	std::vector<MimicInfo> & mimics, std::string & err)
{
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf.c_str(), urdf.size()) != tinyxml2::XML_SUCCESS)
	{
		err = "URDF XML 解析失败";
		return false;
	}
	const tinyxml2::XMLElement * root = doc.RootElement();
	if (root == nullptr)
	{
		err = "URDF 无根元素";
		return false;
	}
	for (const tinyxml2::XMLElement * rc = root->FirstChildElement("ros2_control"); rc != nullptr;
		rc = rc->NextSiblingElement("ros2_control"))
	{
		for (const tinyxml2::XMLElement * j = rc->FirstChildElement("joint"); j != nullptr;
			j = j->NextSiblingElement("joint"))
		{
			const char * jn = j->Attribute("name");
			if (jn == nullptr)
			{
				continue;
			}
			bool has_cmd = false;
			bool has_min = false;
			bool has_max = false;
			double mn = 0.0, mx = 0.0;
			std::string mimic_master;
			double mk = 1.0, moff = 0.0;
			for (const tinyxml2::XMLElement * prm = j->FirstChildElement("param"); prm != nullptr;
				prm = prm->NextSiblingElement("param"))
			{
				const char * pn = prm->Attribute("name");
				const char * txt = prm->GetText();
				if (pn == nullptr || txt == nullptr)
				{
					continue;
				}
				if (std::string(pn) == "min") {mn = std::atof(txt); has_min = true;}
				else if (std::string(pn) == "max") {mx = std::atof(txt); has_max = true;}
				else if (std::string(pn) == "mimic") {mimic_master = txt;}
				else if (std::string(pn) == "multiplier") {mk = std::atof(txt);}
				else if (std::string(pn) == "offset") {moff = std::atof(txt);}
			}
			for (const tinyxml2::XMLElement * ci = j->FirstChildElement("command_interface");
				ci != nullptr; ci = ci->NextSiblingElement("command_interface"))
			{
				const char * nm = ci->Attribute("name");
				if (nm != nullptr && std::string(nm) == "position")
				{
					has_cmd = true;
					break;
				}
			}
			if (!has_cmd)
			{
				continue;
			}
			if (!mimic_master.empty())
			{
				mimics.push_back(MimicInfo{jn, mimic_master, mk, moff});
				continue;
			}
			JointInfo info;
			info.name = jn;
			info.qmin = mn;
			info.qmax = mx;
			info.has_limits = has_min && has_max && mx > mn;
			joints.push_back(info);
		}
	}
	if (joints.empty())
	{
		err = "URDF <ros2_control> 无 position 命令关节";
		return false;
	}
	return true;
}

}  // namespace

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("demo_joint_fullrate");
	node->declare_parameter<double>("duration", 8.0);
	node->declare_parameter<double>("hz", 0.0);

	// ① robot_description (经参数服务读 robot_state_publisher, 同 py demo) + 关节表解析
	std::vector<JointInfo> joints;
	std::vector<MimicInfo> mimics;
	{
		auto cli = node->create_client<rcl_interfaces::srv::GetParameters>(
			"/robot_state_publisher/get_parameters");
		if (!cli->wait_for_service(std::chrono::seconds(5)))
		{
			std::printf("FAIL: robot_state_publisher 参数服务 5s 未出现 (链路未起?)\n");
			return 1;
		}
		auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
		req->names = {"robot_description"};
		auto fut = cli->async_send_request(req);
		if (rclcpp::spin_until_future_complete(node, fut, std::chrono::seconds(5)) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			std::printf("FAIL: 读 robot_description 超时\n");
			return 1;
		}
		const std::string urdf = fut.get()->values[0].string_value;
		if (urdf.empty())
		{
			std::printf("FAIL: robot_description 为空\n");
			return 1;
		}
		std::string err;
		if (!parseUrdf(urdf, joints, mimics, err))
		{
			std::printf("FAIL: %s\n", err.c_str());
			return 1;
		}
		std::printf("关节表: %zu 直控", joints.size());
		for (const auto & j : joints)
		{
			std::printf(" %s", j.name.c_str());
		}
		std::printf(" + %zu mimic", mimics.size());
		for (const auto & m : mimics)
		{
			std::printf(" %s(=%.1f×%s)", m.name.c_str(), m.k, m.master.c_str());
		}
		std::printf("\n");
	}

	// ② 发现期: /joint_states → (a) 实测当前位 (b) hz 自校准
	//    校准源 = 消息 header.stamp 差分中位数 (JSB 的真发布节拍)。
	//    2026-09-23 修正: 首版用回调到达时刻差, 被"spin_some+sleep 2ms"轮询量化抬高中位数
	//    (~4% 低读, 500→480.5Hz 实测); stamp 是发布侧打的, 与消费侧轮询无关。
	const double duration = node->get_parameter("duration").as_double();
	double hz = node->get_parameter("hz").as_double();
	std::vector<double> home(joints.size(), 0.0);
	std::vector<bool> seen(joints.size(), false);
	std::vector<double> periods;
	bool have_last_stamp = false;
	rclcpp::Time last_stamp(0, 0);
	{
		auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", rclcpp::QoS(50),
			[&](const sensor_msgs::msg::JointState::SharedPtr msg)
			{
				if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
				{
					const rclcpp::Time stamp(msg->header.stamp);
					if (have_last_stamp && stamp > last_stamp)
					{
						const double dt = (stamp - last_stamp).seconds();
						if (dt > 1e-5 && dt < 1.0)
						{
							periods.push_back(dt);
						}
					}
					last_stamp = stamp;
					have_last_stamp = true;
				}
				for (std::size_t i = 0; i < joints.size(); ++i)
				{
					for (std::size_t s = 0; s < msg->name.size(); ++s)
					{
						if (msg->name[s] == joints[i].name)
						{
							home[i] = msg->position[s];
							seen[i] = true;
						}
					}
				}
			});
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
		while (std::chrono::steady_clock::now() < deadline && periods.size() < 200)
		{
			rclcpp::spin_some(node);
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
	}
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		if (!seen[i])
		{
			std::printf("FAIL: /joint_states 未见关节 %s\n", joints[i].name.c_str());
			return 1;
		}
	}
	if (hz <= 0.0)
	{
		if (periods.size() < 20)
		{
			std::printf("FAIL: /joint_states 采样不足 (%zu), 无法自校准频率\n", periods.size());
			return 1;
		}
		std::vector<double> srt = periods;
		std::sort(srt.begin(), srt.end());
		hz = 1.0 / srt[srt.size() / 2];
	}
	// 校准/显式值一致性钳位 (审查补, 2026-09-23): 仿真时间跳变/暂停会让 stamp 差分
	// 出 20µs 级离群 → 校准出几十 kHz → 定时器全速狂发。钳到 [10, 2000] 合理带。
	if (hz < 10.0 || hz > 2000.0)
	{
		std::printf("WARN: 校准/指定频率 %.1fHz 超合理带 [10,2000], 钳位后继续\n", hz);
		hz = std::min(std::max(hz, 10.0), 2000.0);
	}
	std::printf("控制频率: %.1f Hz (%s)\n", hz,
		node->get_parameter("hz").as_double() > 0.0 ? "显式指定" : "/joint_states 自校准");

	// ③ 计划: home → 各关节向限位中位偏移 30% (正弦) → home
	std::vector<double> wp(joints.size(), 0.0);
	for (std::size_t i = 0; i < joints.size(); ++i)
	{
		if (joints[i].has_limits)
		{
			const double mid = 0.5 * (joints[i].qmin + joints[i].qmax);
			wp[i] = home[i] + 0.3 * (mid - home[i]);
		}
		else
		{
			wp[i] = home[i];
		}
	}

	// ④ 满速发布 (rclcpp wall 定时器下发, 2026-09-23 用户裁决: 不手写调度环;
	//    消息骨架一次构建, 每拍只改 position —— 回调内零分配)
	auto pub = node->create_publisher<unistackbot_interface::msg::JointCommand>(
		"/joint_stream_controller/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
	unistackbot_interface::msg::JointCommand cmd;
	cmd.mode = unistackbot_interface::msg::JointCommand::MODE_CSP;
	for (const auto & j : joints)
	{
		cmd.joint_names.push_back(j.name);
	}
	for (const auto & m : mimics)
	{
		cmd.joint_names.push_back(m.name);
	}
	cmd.position.assign(cmd.joint_names.size(), 0.0);
	cmd.velocity.assign(cmd.joint_names.size(), 0.0);
	cmd.effort.assign(cmd.joint_names.size(), 0.0);
	// mimic → 主关节索引一次性解析 (审查补: 定时器回调内做 O(n·m) 字符串搜索是
	// 每拍浪费; 主关节缺失在此 fail-loud, 0.0 静默替代会发明不存在的跟随运动)
	std::vector<int> mimic_master_idx(mimics.size(), -1);
	for (std::size_t m = 0; m < mimics.size(); ++m)
	{
		for (std::size_t i = 0; i < joints.size(); ++i)
		{
			if (joints[i].name == mimics[m].master)
			{
				mimic_master_idx[m] = static_cast<int>(i);
				break;
			}
		}
		if (mimic_master_idx[m] < 0)
		{
			std::printf("FAIL: mimic 关节 %s 的主关节 %s 不在直控关节表\n",
				mimics[m].name.c_str(), mimics[m].master.c_str());
			return 1;
		}
	}

	const double period = 1.0 / hz;
	const auto t0 = std::chrono::steady_clock::now();
	uint64_t count = 0;
	std::printf("发送满速流 @%.1fHz, 时长 %.1fs (去回两段正弦)...\n", hz, duration);
	rclcpp::TimerBase::SharedPtr timer;
	timer = node->create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
		[&]()
		{
			const double t =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
			if (t >= duration)
			{
				timer->cancel();
				const double elapsed =
					std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
				std::printf("结束: %lu 条 / %.2fs = 实际 %.1f Hz (名义 %.1f Hz, 达成率 %.1f%%)。\n"
					"JS 断流受控减速收尾 (链上 yaml stale_timeout 生效)\n",
					static_cast<unsigned long>(count), elapsed, count / std::max(elapsed, 1e-9),
					hz, 100.0 * count / std::max(hz * elapsed, 1e-9));
				rclcpp::shutdown();
				return;
			}
			const double u = std::min(t / (duration * 0.5), 2.0);   // [0,2): 段1 去, 段2 回
			const bool back = u >= 1.0;
			const double a = back ? u - 1.0 : u;
			const double s = 0.5 * (1.0 - std::cos(M_PI * a));
			for (std::size_t i = 0; i < joints.size(); ++i)
			{
				cmd.position[i] = back ? wp[i] + (home[i] - wp[i]) * s
				                       : home[i] + (wp[i] - home[i]) * s;
			}
			for (std::size_t m = 0; m < mimics.size(); ++m)
			{
				cmd.position[joints.size() + m] =
					cmd.position[static_cast<std::size_t>(mimic_master_idx[m])] * mimics[m].k +
					mimics[m].off;
			}
			pub->publish(cmd);
			++count;
		});
	rclcpp::spin(node);
	return 0;
}

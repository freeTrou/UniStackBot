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
// 用法: ros2 run unistackbot_demo demo_joint_fullrate [--ros-args -p duration:=14.5 -p hz:=0.0 -p robot:=<机型>]
//   hz=0 (默认) 自校准; 显式指定则以指定值发布 (实际达成率结束时如实报告)。
// 运动内容 (2026-09-24 用户需求: >10s + 大幅 + 有快有慢): 实测位 → 85% → 15% →
//   85% → 回实测位 (限位区间百分比), 四段快慢交替 (2.0/6.0/2.5/4.0s ≈14.5s);
//   duration=总时长, 各段按设计占比缩放 (快慢对比保持); 段内正弦过渡。
// 收尾: 停流即走 JS 断流受控减速 (yaml stale_timeout)。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tinyxml2.h>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

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
	node->declare_parameter<double>("duration", 14.5);   // 四段快慢总时长 (设计值, 2026-09-24)
	node->declare_parameter<double>("hz", 0.0);
	node->declare_parameter<std::string>("robot", "");   // 机型名 (多份配置时必填)

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
	const std::string robot = node->get_parameter("robot").as_string();
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
	const char * hz_src = "显式指定";
	if (hz <= 0.0)
	{
		// 权威源 = bringup 配置文件的 /**.update_rate 直读 (2026-09-24 用户裁决: 不调参数
		// 服务, 直接读配置内容——链上服务在 RT 负载下对 demo 超时, 文件即单一事实源。
		// 注意: launch bus_hz:= 是运行期覆盖, 文件读不到——矩阵测试时显式 -p hz:=N)
		bool got = false;
		{
			namespace fs = std::filesystem;
			std::string cfg_dir;
			try
			{
				cfg_dir = ament_index_cpp::get_package_share_directory("unistackbot_bringup") + "/config";
			}
			catch (const std::exception &)
			{
				std::printf("WARN: unistackbot_bringup 包不可解析, 走实测兜底\n");
			}
			if (!cfg_dir.empty())
			{
				std::vector<fs::path> cands;
				std::error_code ec;
				constexpr char kCfgSuffix[] = "_controllers.yaml";
				for (const auto & e : fs::directory_iterator(cfg_dir, ec))
				{
					const auto n = e.path().filename().string();
					if (n.rfind(kCfgSuffix) == n.size() - sizeof(kCfgSuffix) + 1)
					{
						if (!robot.empty())
						{
							if (n == robot + kCfgSuffix) {cands = {e.path()};}
						}
						else
						{
							cands.push_back(e.path());
						}
					}
				}
				if (!robot.empty() && cands.empty())
				{
					std::printf("FAIL: 机型 '%s' 无配置 %s/%s_controllers.yaml\n",
						robot.c_str(), cfg_dir.c_str(), robot.c_str());
					return 1;
				}
				if (cands.size() > 1)
				{
					std::printf("FAIL: 多份 *_controllers.yaml 且未指定 -p robot:= (机型无默认纪律):\n");
					for (const auto & c : cands) {std::printf("  %s\n", c.string().c_str());}
					return 1;
				}
				if (cands.size() == 1)
				{
					// 直读 /**: ros__parameters: update_rate: N —— 文件内首个
					// "行首剥离空白后以 update_rate: 开头"的行即通配节值 (人工解析免 regex)
					std::ifstream f(cands[0]);
					std::string line;
					while (std::getline(f, line))
					{
						const auto first = line.find_first_not_of(" \t");
						if (first == std::string::npos ||
							line.compare(first, 12, "update_rate:") != 0)
						{
							continue;
						}
						const auto num = std::strtoul(line.c_str() + first + 12, nullptr, 10);
						if (num > 0)
						{
							hz = static_cast<double>(num);
							hz_src = "配置文件直读 (bringup config, 单一事实源)";
							got = true;
							break;
						}
					}
					if (!got)
					{
						std::printf("WARN: %s 无 update_rate 行, 走实测兜底\n", cands[0].string().c_str());
					}
				}
			}
		}
		if (!got)
		{
			// 兜底: /joint_states stamp 差分中位数 (配置不可读时; 1000Hz 流会被 2ms 轮询
			// 欠采样低读到 ~500, 权威路径不可用时的最后手段)
			if (periods.size() < 20)
			{
				std::printf("FAIL: 配置不可读且 /joint_states 采样不足 (%zu)\n", periods.size());
				return 1;
			}
			std::vector<double> srt = periods;
			std::sort(srt.begin(), srt.end());
			hz = 1.0 / srt[srt.size() / 2];
			hz_src = "实测中位数兜底 (高流会低读)";
		}
	}
	// 校准/显式值一致性钳位 (审查补, 2026-09-23): 仿真时间跳变/暂停会让 stamp 差分
	// 出 20µs 级离群 → 校准出几十 kHz → 定时器全速狂发。钳到 [10, 2000] 合理带。
	if (hz < 10.0 || hz > 2000.0)
	{
		std::printf("WARN: 校准/指定频率 %.1fHz 超合理带 [10,2000], 钳位后继续\n", hz);
		hz = std::min(std::max(hz, 10.0), 2000.0);
	}
	std::printf("控制频率: %.1f Hz (%s)\n", hz, hz_src);

	// ③ 计划: 多段快慢变速大幅序列 (2026-09-24 用户需求: >10s + 大幅 + 有快有慢)。
	//    段目标 = 限位区间百分比 (frac<0 = 回实测 home); 段时长显式, 快/慢腿交替
	//    (同跨度命令速度对比 ~3×)。每段起止位置一次性预展开成表 —— 定时器回调
	//    内只查表插值, 零分配。
	struct Leg
	{
		double frac;
		double dur;
	};
	const std::vector<Leg> legs = {
		{0.85, 2.0},   // 快: 部署大跨
		{0.15, 6.0},   // 慢: 摆到对侧 (~70% 限位区间)
		{0.85, 2.5},   // 快: 回摆全跨度
		{-1.0, 4.0},   // 慢: 收尾回实测位
	};
	double design_total = 0.0;
	for (const auto & l : legs) {design_total += l.dur;}
	const double scale = duration / design_total;
	std::vector<double> leg_t0(legs.size(), 0.0);
	std::vector<double> leg_dur(legs.size(), 0.0);
	std::vector<std::vector<double>> leg_from(legs.size(), std::vector<double>(joints.size(), 0.0));
	std::vector<std::vector<double>> leg_to(legs.size(), std::vector<double>(joints.size(), 0.0));
	{
		std::vector<double> prev = home;
		double t_acc = 0.0;
		for (std::size_t l = 0; l < legs.size(); ++l)
		{
			leg_t0[l] = t_acc;
			leg_dur[l] = legs[l].dur * scale;
			t_acc += leg_dur[l];
			leg_from[l] = prev;
			for (std::size_t i = 0; i < joints.size(); ++i)
			{
				if (legs[l].frac >= 0.0 && joints[i].has_limits)
				{
					leg_to[l][i] = joints[i].qmin + legs[l].frac * (joints[i].qmax - joints[i].qmin);
				}
				else
				{
					leg_to[l][i] = home[i];   // 回实测位 / 无限位关节
				}
			}
			prev = leg_to[l];
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
	std::printf("发送满速流 @%.1fHz, 时长 %.1fs (四段快慢: 85%%→15%%→85%%→回实测位, 正弦过渡)...\n",
		hz, duration);
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
			// 定位当前段 (段数固定 4, 线性扫够用; a 钳 1 兜尾拍)
			std::size_t l = 0;
			while (l + 1 < legs.size() && t >= leg_t0[l + 1]) {++l;}
			const double a = leg_dur[l] > 0.0
				? std::min((t - leg_t0[l]) / leg_dur[l], 1.0) : 1.0;
			const double s = 0.5 * (1.0 - std::cos(M_PI * a));
			for (std::size_t i = 0; i < joints.size(); ++i)
			{
				cmd.position[i] = leg_from[l][i] + (leg_to[l][i] - leg_from[l][i]) * s;
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

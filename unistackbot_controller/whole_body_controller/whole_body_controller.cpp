#include "whole_body_controller/whole_body_controller.hpp"

#include <algorithm>
#include <cstdlib>
#include <unordered_map>

#include "rclcpp/qos.hpp"
#include <tinyxml2.h>
#include "ulog/ulog.hpp"

namespace unistackbot_controller
{
namespace
{
inline uint64_t nowMs()
{
	return static_cast<uint64_t>(
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
			.count());
}
// worker 心跳看门狗阈值: 解算停供超过此窗口 = RT 保持最后命令 (恢复自动续供)
constexpr uint64_t kWorkerTimeoutMs = 200;
}   // namespace

controller_interface::CallbackReturn WholeBodyController::on_init()
{
	try
	{
		auto_declare<std::string>("solver", "");   // WholeBodySolver 插件名: 必填无默认 (算法选择显式手动)
		auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
		auto_declare<std::vector<std::string>>("end_frames", std::vector<std::string>{});
		auto_declare<std::string>("base_frame", "");
		auto_declare<double>("dt_s", 0.0);
		auto_declare<std::string>("vendor_config_path", "");
		auto_declare<double>("max_delta_rad", 0.01);
		auto_declare<std::string>("robot_description", "");
	}
	catch (const std::exception & e)
	{
		ULOG_ERROR("wb: on_init 参数声明失败: %s", e.what());
		return controller_interface::CallbackReturn::ERROR;
	}
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_configure(const rclcpp_lifecycle::State &)
{
	// ---- 参数面 (显式无默认纪律: solver/base_frame/dt_s 缺一即拒) ----
	const std::string solver_name = get_node()->get_parameter("solver").as_string();
	if (solver_name.empty())
	{
		ULOG_ERROR("wb: 缺 solver 显式配置 (算法选择必须显式手动; 可选 = 已安装 WholeBodySolver 插件)");
		return controller_interface::CallbackReturn::ERROR;
	}
	base_frame_ = get_node()->get_parameter("base_frame").as_string();
	if (base_frame_.empty())
	{
		ULOG_ERROR("wb: 缺 base_frame (目标参考系必须显式声明)");
		return controller_interface::CallbackReturn::ERROR;
	}
	dt_s_ = get_node()->get_parameter("dt_s").as_double();
	if (!(dt_s_ > 0.0))
	{
		ULOG_ERROR("wb: dt_s 须 > 0 (worker 节拍约定值)");
		return controller_interface::CallbackReturn::ERROR;
	}
	max_delta_rad_ = get_node()->get_parameter("max_delta_rad").as_double();
	joint_names_ = get_node()->get_parameter("joints").as_string_array();
	end_frames_ = get_node()->get_parameter("end_frames").as_string_array();
	if (joint_names_.empty() || joint_names_.size() > kMaxJointsWb)
	{
		ULOG_ERROR("wb: joints 显式列表为空或超容量 %zu", kMaxJointsWb);
		return controller_interface::CallbackReturn::ERROR;
	}
	// 壳 v0 契约 = 恰 2 目标帧 (GoalsPod 双位姿定容); 不靠求解器恰好拒配置兜底
	if (end_frames_.size() != 2)
	{
		ULOG_ERROR("wb: end_frames 须恰为 2 (左/右末端), 收到 %zu", end_frames_.size());
		return controller_interface::CallbackReturn::ERROR;
	}

	// ---- 限位来源: URDF (关节名 → limit; 未知关节即拒) ----
	std::string urdf;
	if (!get_node()->get_parameter("robot_description", urdf) || urdf.empty())
	{
		ULOG_ERROR("wb: robot_description 未注入");
		return controller_interface::CallbackReturn::ERROR;
	}
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf.c_str(), urdf.size()) != tinyxml2::XML_SUCCESS)
	{
		ULOG_ERROR("wb: URDF 解析失败");
		return controller_interface::CallbackReturn::ERROR;
	}
	// JS 同款 tinyxml2 限位解析 (controller_common/urdf_command_joints.hpp 先例)
	std::unordered_map<std::string, std::pair<double, double>> limits;
	for (tinyxml2::XMLElement * j = doc.RootElement()->FirstChildElement("joint"); j != nullptr;
		j = j->NextSiblingElement("joint"))
	{
		const char * name = j->Attribute("name");
		tinyxml2::XMLElement * lim = j->FirstChildElement("limit");
		if (name != nullptr && lim != nullptr && lim->Attribute("lower") != nullptr && lim->Attribute("upper") != nullptr)
		{
			limits[name] = {std::atof(lim->Attribute("lower")), std::atof(lim->Attribute("upper"))};
		}
	}
	q_min_.assign(joint_names_.size(), 0.0);
	q_max_.assign(joint_names_.size(), 0.0);
	for (size_t i = 0; i < joint_names_.size(); ++i)
	{
		const auto it = limits.find(joint_names_[i]);
		if (it == limits.end())
		{
			ULOG_ERROR("wb: 关节 '%s' 不在 URDF 或无 limit", joint_names_[i].c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
		q_min_[i] = it->second.first;
		q_max_[i] = it->second.second;
	}

	// ---- 求解器装载 (whole_body_plugins 注册库; ik_solver 缝③ 同款) ----
	try
	{
		solver_loader_ = std::make_unique<pluginlib::ClassLoader<unistackbot_algorithm::WholeBodySolver>>(
			"unistackbot_algorithm", "unistackbot_algorithm::WholeBodySolver");
		solver_ = solver_loader_->createSharedInstance(solver_name);
	}
	catch (const pluginlib::LibraryLoadException & e)
	{
		ULOG_ERROR("wb: 求解器 '%s' 装载失败: %s (已注册: 仓内 dualarm_dls)", solver_name.c_str(), e.what());
		return controller_interface::CallbackReturn::ERROR;
	}
	unistackbot_algorithm::WholeBodyConfig cfg;
	cfg.urdf = urdf;
	cfg.base_frame = base_frame_;
	cfg.control_joints = joint_names_;
	cfg.end_frames = end_frames_;
	cfg.telemetry_frames = {};   // 壳最小版不消费遥测帧 (status 面随 M6 全量版)
	cfg.dt_s = dt_s_;
	cfg.vendor_config_path = get_node()->get_parameter("vendor_config_path").as_string();
	std::string why;
	if (!solver_->init(cfg, why))
	{
		ULOG_ERROR("wb: 求解器 init 拒绝: %s", why.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}

	// ---- 值通道 + 订阅 (QoS = 点流契约: reliable + KeepLast(1)) ----
	goals_ch_.init(GoalsPod{});
	meas_ch_.init(MeasPod{});
	sol_ch_.init(MeasPod{});
	command_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseArray>(
		"~/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
		[this](const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
			GoalsPod g{};
			if (msg->poses.size() != 2)
			{
				return;   // 维度门静默丢 (CM 同款防御)
			}
			g.l.x = msg->poses[0].position.x;
			g.l.y = msg->poses[0].position.y;
			g.l.z = msg->poses[0].position.z;
			g.l.qw = msg->poses[0].orientation.w;
			g.l.qx = msg->poses[0].orientation.x;
			g.l.qy = msg->poses[0].orientation.y;
			g.l.qz = msg->poses[0].orientation.z;
			g.r.x = msg->poses[1].position.x;
			g.r.y = msg->poses[1].position.y;
			g.r.z = msg->poses[1].position.z;
			g.r.qw = msg->poses[1].orientation.w;
			g.r.qx = msg->poses[1].orientation.x;
			g.r.qy = msg->poses[1].orientation.y;
			g.r.qz = msg->poses[1].orientation.z;
			g.stamp_ms = nowMs();
			goals_ch_.publish(g);
		});

	// ~/status (RealtimePublisher, 20Hz 节流 trylock)
	status_pub_ = get_node()->create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).reliable());
	rt_status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(status_pub_);

	// worker scratch 定容 (worker 线程私有, 零分配)
	wb_in_.q.assign(joint_names_.size(), 0.0);
	wb_in_.dq.assign(joint_names_.size(), 0.0);
	wb_in_.goals.resize(2);

	ULOG_INFO("wb: 配置完成 (solver=%s, %zu 关节, end=%zu 帧, dt=%.1fms)", solver_name.c_str(),
		joint_names_.size(), end_frames_.size(), dt_s_ * 1e3);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration WholeBodyController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	for (const auto & name : joint_names_)
	{
		conf.names.push_back(name + "/position");
	}
	return conf;
}

controller_interface::InterfaceConfiguration WholeBodyController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	for (const auto & name : joint_names_)
	{
		conf.names.push_back(name + "/position");
		conf.names.push_back(name + "/velocity");
	}
	return conf;
}

controller_interface::CallbackReturn WholeBodyController::on_activate(const rclcpp_lifecycle::State &)
{
	// 接口到手序不可信 (EEB 同款防御): 激活期按名映射一次 → RT 环纯索引 (零字符串查找)
	const size_t n = joint_names_.size();
	cmd_last_.assign(n, 0.0);
	cmd_try_.assign(n, 0.0);
	have_last_ = false;
	idx_q_.assign(n, -1);
	idx_v_.assign(n, -1);
	idx_cmd_.assign(n, -1);
	for (size_t i = 0; i < n; ++i)
	{
		for (size_t k = 0; k < state_interfaces_.size(); ++k)
		{
			if (state_interfaces_[k].get_name() == joint_names_[i] + "/position")
			{
				idx_q_[i] = static_cast<int>(k);
			}
			else if (state_interfaces_[k].get_name() == joint_names_[i] + "/velocity")
			{
				idx_v_[i] = static_cast<int>(k);
			}
		}
		for (size_t k = 0; k < command_interfaces_.size(); ++k)
		{
			if (command_interfaces_[k].get_name() == joint_names_[i] + "/position")
			{
				idx_cmd_[i] = static_cast<int>(k);
			}
		}
		if (idx_q_[i] < 0 || idx_v_[i] < 0 || idx_cmd_[i] < 0)
		{
			ULOG_ERROR("wb: 激活失败: 关节 '%s' 接口缺失 (q/v/cmd)", joint_names_[i].c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
		cmd_last_[i] = state_interfaces_[static_cast<size_t>(idx_q_[i])].get_value();
	}
	have_last_ = true;

	worker_running_.store(true);
	worker_stamp_ms_.store(nowMs());
	worker_ = std::thread([this]() { workerLoop(); });
	ULOG_INFO("wb: 激活 (%zu 关节, worker @ %.0fHz)", joint_names_.size(), 1.0 / dt_s_);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyController::on_deactivate(const rclcpp_lifecycle::State &)
{
	worker_running_.store(false);
	if (worker_.joinable())
	{
		worker_.join();
	}
	ULOG_INFO("wb: 去激活 (worker 已停)");
	return controller_interface::CallbackReturn::SUCCESS;
}

void WholeBodyController::workerLoop()
{
	auto next_wakeup = std::chrono::steady_clock::now();
	const std::chrono::nanoseconds period_ns(static_cast<int64_t>(dt_s_ * 1e9));
	while (worker_running_.load())
	{
		next_wakeup += period_ns;
		std::this_thread::sleep_until(next_wakeup);
		worker_stamp_ms_.store(nowMs(), std::memory_order_relaxed);

		// 最新测量 + 目标 (拿不到沿用旧值 —— 值通道语义)
		uint64_t seq = 0;
		if (meas_ch_.read(meas_rx_, seq))
		{
			(void)seq;
		}
		if (goals_ch_.read(goals_rx_, seq))
		{
			(void)seq;
		}
		const size_t n = meas_rx_.n;
		if (n == 0 || n > kMaxJointsWb)
		{
			continue;   // RT 侧尚未供测量 (激活首拍前) —— 空转等待
		}
		for (size_t i = 0; i < n; ++i)
		{
			wb_in_.q[i] = meas_rx_.q[i];
			wb_in_.dq[i] = meas_rx_.dq[i];
		}
		wb_in_.goals[0].frame = end_frames_[0];
		wb_in_.goals[0].target = goals_rx_.l;
		wb_in_.goals[1].frame = end_frames_[1];
		wb_in_.goals[1].target = goals_rx_.r;

		const auto result = solver_->solve(wb_in_, wb_out_);
		if (result == unistackbot_algorithm::WholeBodyResult::OK)
		{
			sol_tx_.n = n;
			for (size_t i = 0; i < n; ++i)
			{
				sol_tx_.q[i] = wb_out_.q[i];
			}
			sol_ch_.publish(sol_tx_);
		}
		// ~/status: 节流 (worker 每 50 拍发一次; dt=5ms 时约 4Hz), RealtimePublisher trylock
		if (++status_count_ % 50 == 0 && rt_status_pub_)
		{
			if (rt_status_pub_->trylock())
			{
				auto & msg = rt_status_pub_->msg_;
				msg.data.clear();
				msg.data += "result=" + std::to_string(static_cast<int>(result));
				for (const auto & t : wb_out_.tasks)
				{
					msg.data += " " + t.task + "_err=" + std::to_string(t.error);
				}
				rt_status_pub_->unlockAndPublish();
			}
		}
		// 非 OK: 不发布 (解通道保持旧值 → RT 保持最后命令; 诚实拒绝语义)
	}
}

bool WholeBodyController::sanitize(std::vector<double> & q)
{
	for (size_t i = 0; i < q.size(); ++i)
	{
		if (!std::isfinite(q[i]))
		{
			return false;   // NaN 门: 整拍保持
		}
		q[i] = std::clamp(q[i], q_min_[i], q_max_[i]);
		if (have_last_)
		{
			q[i] = std::clamp(q[i], cmd_last_[i] - max_delta_rad_, cmd_last_[i] + max_delta_rad_);
		}
	}
	return true;
}

controller_interface::return_type WholeBodyController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
	const size_t n = joint_names_.size();

	// ① 测量发布 (激活期索引, RT 零字符串查找)
	MeasPod meas_tx{};
	meas_tx.n = n;
	for (size_t i = 0; i < n; ++i)
	{
		meas_tx.q[i] = state_interfaces_[static_cast<size_t>(idx_q_[i])].get_value();
		meas_tx.dq[i] = state_interfaces_[static_cast<size_t>(idx_v_[i])].get_value();
	}
	meas_ch_.publish(meas_tx);

	// ② worker 心跳看门狗: 超时 = 保持最后命令 (解算停供不动作)
	const uint64_t stamp = worker_stamp_ms_.load(std::memory_order_relaxed);
	if (nowMs() - stamp > kWorkerTimeoutMs)
	{
		return controller_interface::return_type::OK;
	}

	// ③ 最新解 → 安全落地 → 命令接口
	uint64_t seq = 0;
	MeasPod sol_rx{};
	if (sol_ch_.read(sol_rx, seq) && sol_rx.n == n)
	{
		for (size_t i = 0; i < n; ++i)
		{
			cmd_try_[i] = sol_rx.q[i];
		}
		if (sanitize(cmd_try_))
		{
			for (size_t i = 0; i < n; ++i)
			{
				command_interfaces_[static_cast<size_t>(idx_cmd_[i])].set_value(cmd_try_[i]);
				cmd_last_[i] = cmd_try_[i];
			}
		}
		// 消毒失败 (NaN): 保持 cmd_last_ (已写入的接口值不变 = 保持)
	}
	return controller_interface::return_type::OK;
}

}  // namespace unistackbot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unistackbot_controller::WholeBodyController, controller_interface::ControllerInterface)

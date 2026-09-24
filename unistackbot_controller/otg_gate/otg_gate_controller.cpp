#include "otg_gate/otg_gate_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

#include "controller_common/urdf_command_joints.hpp"
#include "ulog/ulog.hpp"

namespace
{

constexpr char kPluginName[] = "otg_gate_controller";

}  // namespace

namespace unistackbot_controller
{

using unistackbot_algorithm::CartesianPose;
using unistackbot_algorithm::AnalyticPiper;
using unistackbot_algorithm::DlsIk;
using unistackbot_algorithm::DlsIkConfig;
using unistackbot_algorithm::IkSolver;
using unistackbot_algorithm::RedundancyPreference;
using unistackbot_algorithm::UrdfFk;

controller_interface::CallbackReturn OtgGateController::on_init()
{
	// 必填无默认族: 全部 0/"" 哨兵声明, on_configure 校验缺配置即 ERROR (机型资产纪律)
	auto_declare<std::string>("robot_description", "");
	auto_declare<std::string>("base_link", "");       // 必填无默认 (机型/拓扑类, 显式纪律)
	auto_declare<std::string>("tip_link", "");
	auto_declare<std::string>("ik_solver", "");       // 必填无默认 (算法选择显式手动)
	auto_declare<double>("max_acceleration", 0.0);    // OTG 机型资产, yaml 显式
	auto_declare<double>("max_jerk", 0.0);
	auto_declare<double>("ik_timeout_ms", 0.0);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OtgGateController::on_configure(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	unistackbot_common::ulog_config log_cfg;
	log_cfg.console = true;
	log_cfg.file_path = nullptr;
	log_cfg.level = unistackbot_common::ulog_level::info;
	log_cfg.max_bytes = 0;
	log_cfg.backups = 0;
	unistackbot_common::ulog_init(log_cfg);   // 幂等 + 引用计数 (与同进程其他组件共存)

	base_link_ = get_node()->get_parameter("base_link").as_string();
	tip_link_ = get_node()->get_parameter("tip_link").as_string();
	ik_solver_name_ = get_node()->get_parameter("ik_solver").as_string();
	max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
	max_jerk_ = get_node()->get_parameter("max_jerk").as_double();
	const double ik_timeout_ms = get_node()->get_parameter("ik_timeout_ms").as_double();
	// 数值参数 configure 期校验 (审查补, 2026-09-23): 缺配置/非法值在此拒绝,
	// 不留给 RT 环 (负值曾会经 uint64 转换回绕, 静默关闭 IK 墙钟预算)
	if (base_link_.empty() || tip_link_.empty())
	{
		ULOG_ERROR("%s: 缺 base_link/tip_link 显式配置 (链拓扑必须显式声明)", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	if (ik_solver_name_.empty())
	{
		ULOG_ERROR("%s: 缺 ik_solver 显式配置 (算法选择必须显式手动; 当前可选: dls | analytic_piper)",
			kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	if (max_acceleration_ <= 0.0 || max_jerk_ <= 0.0 || ik_timeout_ms <= 0.0)
	{
		ULOG_ERROR("%s: OTG 机型资产缺显式配置或非法 (max_acceleration=%.3g, max_jerk=%.3g, "
			"ik_timeout_ms=%.3g; 均须 > 0, yaml <robot>_controllers.yaml 显式声明)",
			kPluginName, max_acceleration_, max_jerk_, ik_timeout_ms);
		return controller_interface::CallbackReturn::ERROR;
	}
	ik_timeout_ns_ = static_cast<int64_t>(ik_timeout_ms * 1e6);

	std::string urdf;
	if (!get_node()->get_parameter("robot_description", urdf) || urdf.empty())
	{
		ULOG_ERROR("%s: robot_description 未注入", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	std::vector<double> qmin, qmax;
	std::string err;
	if (!parseJointsFromUrdf(urdf, cmd_joint_names_, qmin, qmax, cmd_vmax_, err))
	{
		ULOG_ERROR("%s: %s", kPluginName, err.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}

	// 运动学 (回调线程私有实例; RT 环零运动学计算)
	fk_ = std::make_unique<UrdfFk>();
	if (!fk_->init(urdf, base_link_, tip_link_, err))
	{
		ULOG_ERROR("%s: 建链失败: %s", kPluginName, err.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	if (ik_solver_name_ == "dls")
	{
		auto dls = std::make_unique<DlsIk>();
		DlsIkConfig cfg;
		cfg.timeout_ns = static_cast<uint64_t>(ik_timeout_ns_);
		cfg.max_iterations = 200;   // 冷启动世界: 不限时档配置 (同 CM worker)
		dls->setConfig(cfg);
		ik_ = std::move(dls);
	}
	else if (ik_solver_name_ == "analytic_piper")
	{
		ik_ = std::make_unique<AnalyticPiper>();
	}
	else
	{
		ULOG_ERROR("%s: 未知 ik_solver '%s' (可选: dls | analytic_piper)",
			kPluginName, ik_solver_name_.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	if (!ik_->init(fk_.get(), err))
	{
		ULOG_ERROR("%s: IK init 失败: %s", kPluginName, err.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	ULOG_INFO("%s: IK 求解器 = '%s' (显式配置), 链 %s -> %s", kPluginName,
		ik_solver_name_.c_str(), base_link_.c_str(), tip_link_.c_str());

	// 链序 ↔ 命令序映射 (链关节必须全部可命令)
	const auto & chain_names = fk_->jointNames();
	chain_n_ = static_cast<uint32_t>(chain_names.size());
	cmd_from_chain_.assign(chain_n_, -1);
	chain_from_cmd_.assign(cmd_joint_names_.size(), -1);
	for (uint32_t k = 0; k < chain_n_; ++k)
	{
		for (std::size_t i = 0; i < cmd_joint_names_.size(); ++i)
		{
			if (chain_names[k] == cmd_joint_names_[i])
			{
				cmd_from_chain_[k] = static_cast<int>(i);
				chain_from_cmd_[i] = static_cast<int>(k);
				break;
			}
		}
		if (cmd_from_chain_[k] < 0)
		{
			ULOG_ERROR("%s: 链关节 '%s' 不在 <ros2_control> position 命令集", kPluginName,
				chain_names[k].c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
	}

	// 契约通道: 订阅 (executor 线程) + 发布 (RT trylock; JS 的命令话题是值通道)
	target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
		"~/target", rclcpp::QoS(10),
		[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {onTarget(msg);});
	js_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
		"/joint_states", rclcpp::QoS(50),
		[this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointState(msg);});
	cmd_pub_ = get_node()->create_publisher<unistackbot_interface::msg::JointCommand>(
		"/joint_stream_controller/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
	rt_cmd_ = std::make_shared<realtime_tools::RealtimePublisher<
		unistackbot_interface::msg::JointCommand>>(cmd_pub_);

	// 消息骨架预填 (非 RT 期一次性分配; RT 环只写 position 数值 —— 零分配)
	{
		auto & m = rt_cmd_->msg_;
		m.joint_names = cmd_joint_names_;
		m.mode = unistackbot_interface::msg::JointCommand::MODE_CSP;
		m.position.assign(cmd_joint_names_.size(), 0.0);
		m.velocity.assign(cmd_joint_names_.size(), 0.0);
		m.effort.assign(cmd_joint_names_.size(), 0.0);
		m.kp.clear();
		m.kd.clear();
	}

	dt_known_ = false;
	otg_ready_ = false;
	period_n_ = 0;
	ULOG_INFO("%s: 配置完成 (%zu 命令关节, 链 %u; 目标话题 ~/target base 系)",
		kPluginName, cmd_joint_names_.size(), chain_n_);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OtgGateController::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	otg_ready_ = false;   // 重激活从新实测快照重建 OTG 基准
	// 重激活防重放 (同 CM worker cold_req_ 模式): 预读残值记 seq 水位并弃用 ——
	// SpLatest 留着上一次激活期的目标, 无水位则重激活首拍把它当新目标追打
	// (审查实锤 2026-09-23: 门在别处动过臂后重激活, 会无令自走回旧目标)
	GateTarget residual;
	(void)target_ch_.read(residual, target_seq_);
	have_goal_ = false;
	ULOG_INFO("%s: 激活 (等待 update_rate 校准 + 首帧实测)", kPluginName);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OtgGateController::command_interface_configuration() const
{
	return controller_interface::InterfaceConfiguration{
		controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration OtgGateController::state_interface_configuration() const
{
	return controller_interface::InterfaceConfiguration{
		controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type OtgGateController::update(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	// update_rate 定源 (2026-09-23, 同 JS/CM): yaml /** 通配节覆盖控制器节点 →
	// configure 期 get_update_rate() 即权威值; 首 16 拍中位数兜底 + 交叉校验
	// (激活瞬间 switch 服务同步 update 的 µs period 污染中位数)
	if (!dt_known_)
	{
		const double p = period.seconds();
		if (std::isfinite(p) && p > 1e-7 && p < 1.0)
		{
			period_samples_[period_n_++] = p;
		}
		if (period_n_ < kPeriodSamples)
		{
			return controller_interface::return_type::OK;   // 校准期不发命令 (JS 保持现状)
		}
		std::array<double, kPeriodSamples> srt{};
		std::copy(period_samples_, period_samples_ + kPeriodSamples, srt.begin());
		std::sort(srt.begin(), srt.end());
		const double dt_med = srt[kPeriodSamples / 2];
		double dt = dt_med;
		const unsigned int hz_auth = get_update_rate();
		if (hz_auth > 0)
		{
			dt = 1.0 / static_cast<double>(hz_auth);
			if (std::fabs(dt_med - dt) / dt > 0.2)
			{
				ULOG_WARN("%s: 实测中位数 dt=%.0fµs 偏离权威 update_rate %uHz"
					" (激活期 period 污染), 以参数为准", kPluginName, dt_med * 1e6, hz_auth);
			}
		}
		dt_ = dt;
		unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints>::Limits lim;
		lim.max_velocity.fill(3.0);
		lim.max_acceleration.fill(max_acceleration_);
		lim.max_jerk.fill(max_jerk_);
		for (uint32_t k = 0; k < chain_n_; ++k)
		{
			lim.max_velocity[k] = cmd_vmax_[cmd_from_chain_[k]];   // URDF max_velocity 单一事实源
		}
		if (!otg_.init(dt_, lim, 0.3))
		{
			ULOG_ERROR("%s: OtgStream init 失败", kPluginName);
			return controller_interface::return_type::ERROR;
		}
		dt_known_ = true;
		if (hz_auth > 0)
		{
			ULOG_INFO("%s: 就绪 (dt=%.0fµs 权威参数, OTG a=%.1f j=%.1f)", kPluginName,
				dt_ * 1e6, max_acceleration_, max_jerk_);
		}
		else
		{
			// 兜底路径必须显眼 (yaml 缺 /**.update_rate 时会退回实测中位数 ——
			// 该中位数可被激活期 µs period 污染, 是 35 倍减速 bug 的根)
			ULOG_WARN("%s: 就绪 (dt=%.0fµs 实测中位数兜底 —— yaml 缺 /**.update_rate 覆盖,"
				" 建议补上; OTG a=%.1f j=%.1f)", kPluginName, dt_ * 1e6,
				max_acceleration_, max_jerk_);
		}
	}
	if (!otg_ready_)
	{
		// 首帧实测 = OTG reset 基准 (激活位即保持位, 无跳变)
		JointSnapshot snap;
		uint64_t seq = 0;
		if (!snap_ch_.read(snap, seq))
		{
			return controller_interface::return_type::OK;
		}
		std::array<double, unistackbot_interface::kMaxJoints> q0{};
		for (std::size_t i = 0; i < cmd_joint_names_.size(); ++i)
		{
			if (chain_from_cmd_[i] >= 0)
			{
				q0[chain_from_cmd_[i]] = snap.pos[i];
			}
		}
		otg_.reset(q0);
		otg_ready_ = true;
	}

	// 最新关节终点 (点流): 只认水位之上的新目标 (重激活防重放), 本次激活期的目标
	// 存 goal_ 持续整形 —— 每拍都发 OTG 输出, JS 侧看到的是连续总线频率点流
	GateTarget gt;
	uint64_t tseq = 0;
	if (target_ch_.read(gt, tseq) && tseq != target_seq_)
	{
		target_seq_ = tseq;
		goal_ = gt;
		have_goal_ = true;
	}
	if (!have_goal_)
	{
		return controller_interface::return_type::OK;   // 无目标 = 不发, JS 保持
	}
	std::array<double, unistackbot_interface::kMaxJoints> tgt{};
	for (uint32_t k = 0; k < goal_.n && k < unistackbot_interface::kMaxJoints; ++k)
	{
		tgt[k] = goal_.q[k];
	}
	std::array<double, unistackbot_interface::kMaxJoints> out{};
	(void)otg_.update(tgt, out);   // Hold 时 out 恒有效 (OtgStream 契约) —— 照发, JS+write 层兜底

	// 发布: 链关节 = OTG 输出, 非链关节 = 实测回显 (门不发明夹爪运动)
	JointSnapshot snap;
	uint64_t sseq = 0;
	const bool have_snap = snap_ch_.read(snap, sseq);
	if (!rt_cmd_->trylock())
	{
		return controller_interface::return_type::OK;   // 拿不到锁跳本拍 (点流语义, 下拍重试)
	}
	auto & m = rt_cmd_->msg_;
	for (std::size_t i = 0; i < cmd_joint_names_.size(); ++i)
	{
		const int k = chain_from_cmd_[i];
		m.position[i] = (k >= 0) ? out[k] : (have_snap ? snap.pos[i] : 0.0);
	}
	rt_cmd_->unlockAndPublish();
	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OtgGateController::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	otg_ready_ = false;
	ULOG_INFO("%s: 停用 (JS 保持末位命令)", kPluginName);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OtgGateController::on_cleanup(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	rt_cmd_.reset();
	cmd_pub_.reset();
	target_sub_.reset();
	js_sub_.reset();
	ik_.reset();
	fk_.reset();
	warned_frame_.clear();
	return controller_interface::CallbackReturn::SUCCESS;
}

void OtgGateController::onTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	const std::string & frame = msg->header.frame_id;
	if (!frame.empty() && frame != base_link_)
	{
		if (warned_frame_ != frame)
		{
			warned_frame_ = frame;
			ULOG_WARN("%s: 目标帧 '%s' != base '%s', 丢弃 (base 系语义)", kPluginName,
				frame.c_str(), base_link_.c_str());
		}
		return;
	}
	// 实测种子 (无快照 = 无可靠种子, 诚实丢弃; 最近解语义需要真种子)
	JointSnapshot snap;
	uint64_t seq = 0;
	if (!snap_ch_.read(snap, seq))
	{
		ULOG_WARN("%s: 无 /joint_states 快照, 终点丢弃", kPluginName);
		return;
	}
	std::vector<double> seed(chain_n_, 0.0);
	for (std::size_t i = 0; i < cmd_joint_names_.size(); ++i)
	{
		if (chain_from_cmd_[i] >= 0)
		{
			seed[chain_from_cmd_[i]] = snap.pos[i];
		}
	}
	CartesianPose target;
	target.x = msg->pose.position.x;
	target.y = msg->pose.position.y;
	target.z = msg->pose.position.z;
	target.qw = msg->pose.orientation.w;
	target.qx = msg->pose.orientation.x;
	target.qy = msg->pose.orientation.y;
	target.qz = msg->pose.orientation.z;
	std::vector<double> out = seed;
	const auto r = ik_->solve(target, seed, RedundancyPreference{}, out, nullptr,
		unistackbot_algorithm::SolveMode::COLD_START);
	if (r != unistackbot_algorithm::IkResult::OK)
	{
		ULOG_WARN("%s: 终点求解拒绝 (result=%d), 保持上一目标", kPluginName,
			static_cast<int>(r));
		return;
	}
	GateTarget gt{};
	gt.n = chain_n_;
	for (uint32_t k = 0; k < chain_n_; ++k)
	{
		if (!std::isfinite(out[k]))
		{
			ULOG_WARN("%s: 解含非有限值, 丢弃 (皮带扣; 契约已保证)", kPluginName);
			return;
		}
		gt.q[k] = out[k];
	}
	target_ch_.publish(gt);
	ULOG_INFO("%s: 采纳终点 → OTG 整形 (solver=%s)", kPluginName, ik_solver_name_.c_str());
}

void OtgGateController::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
	// 按名映射 (JSB 乱序是常态); 未知关节保持旧值; /joint_states 是关节单一事实源
	if (msg->name.size() != msg->position.size())
	{
		return;
	}
	for (std::size_t s = 0; s < msg->name.size(); ++s)
	{
		for (std::size_t i = 0; i < cmd_joint_names_.size(); ++i)
		{
			if (msg->name[s] == cmd_joint_names_[i])
			{
				if (std::isfinite(msg->position[s]))
				{
					js_cache_[i] = msg->position[s];
				}
				break;
			}
		}
	}
	JointSnapshot snap{};
	snap.n = static_cast<uint32_t>(cmd_joint_names_.size());
	for (std::size_t i = 0; i < cmd_joint_names_.size() && i < unistackbot_interface::kMaxJoints; ++i)
	{
		snap.pos[i] = js_cache_[i];
	}
	snap_ch_.publish(snap);
}

}  // namespace unistackbot_controller

PLUGINLIB_EXPORT_CLASS(unistackbot_controller::OtgGateController, controller_interface::ControllerInterface)

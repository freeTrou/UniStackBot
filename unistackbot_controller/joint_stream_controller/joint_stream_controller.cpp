#include "joint_stream_controller/joint_stream_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <tinyxml2.h>

#include <pluginlib/class_list_macros.hpp>

#include "controller_common/urdf_command_joints.hpp"
#include "ulog/ulog.hpp"

namespace
{

constexpr char kPluginName[] = "joint_stream_controller";

}  // namespace

namespace unistackbot_controller
{

controller_interface::CallbackReturn JointStreamController::on_init()
{
	auto_declare<std::string>("interpolation", "hold");   // hold | ruckig
	auto_declare<double>("max_acceleration", max_acceleration_);
	auto_declare<double>("max_jerk", max_jerk_);
	auto_declare<std::string>("robot_description", "");
	// 应用命令观测面 (~/applied_command): 默认关 —— 透传原则, 观测不默认占资源
	auto_declare<bool>("observe_applied_command", false);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStreamController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	unistackbot_common::ulog_config log_cfg;
	log_cfg.console = true;
	log_cfg.file_path = nullptr;
	log_cfg.level = unistackbot_common::ulog_level::info;
	log_cfg.max_bytes = 0;
	log_cfg.backups = 0;
	unistackbot_common::ulog_init(log_cfg);   // 幂等 + 引用计数 (与同进程其他组件共存)

	const std::string interp = get_node()->get_parameter("interpolation").as_string();
	if (interp == "hold")
	{
		interpolation_ = Interpolation::HOLD;
	}
	else if (interp == "ruckig")
	{
		interpolation_ = Interpolation::RUCKIG;
	}
	else
	{
		ULOG_ERROR("%s: interpolation 非法 (%s), 可选 hold|ruckig", kPluginName, interp.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
	max_jerk_ = get_node()->get_parameter("max_jerk").as_double();

	std::string urdf;
	if (!get_node()->get_parameter("robot_description", urdf) || urdf.empty())
	{
		ULOG_ERROR("%s: robot_description 未注入", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	std::string err;
	// 关节编组 (2026-10-14 单块化收编): ros2_control 块 = 硬件描述 (一机型一块),
	// 控制器编组 (js_arms/js_torso 等多实例) 归 yaml joints 显式列表 —— 编组是
	// 控制器层决策, 各实例认领互斥子集; 空列表 = 单块机型旧行为 (URDF 全合并)。
	// 显式列表的限位/速度仍逐关节回 URDF 解析 (单一事实源不变), 缺关节 fail-fast。
	// yaml 已带 joints 时 declare 会撞 "already declared" (重配置路径)——
	// 先读, 读不到再声明 (auto_declare 的防御序)
	std::vector<std::string> sel;
	if (!get_node()->get_parameter("joints", sel))
	{
		get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
		get_node()->get_parameter("joints", sel);
	}
	std::vector<std::string> all_names;
	std::vector<double> all_qmin, all_qmax, all_vmax;
	if (!parseJointsFromUrdf(urdf, all_names, all_qmin, all_qmax, all_vmax, err))
	{
		ULOG_ERROR("%s: %s", kPluginName, err.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	bool explicit_joints = !sel.empty();
	if (!explicit_joints)
	{
		joint_names_ = all_names;
		q_min_ = all_qmin;
		q_max_ = all_qmax;
		vmax_ = all_vmax;
	}
	else
	{
		// 顺序即 yaml 声明序; 重复/未知关节 fail-fast (列表与 URDF 漂移配置期暴露)
		for (const std::string & name : sel)
		{
			std::size_t k = 0;
			while (k < joint_names_.size() && joint_names_[k] != name)
			{
				++k;
			}
			if (k < joint_names_.size())
			{
				ULOG_ERROR("%s: joints 列表重复关节 '%s'", kPluginName, name.c_str());
				return controller_interface::CallbackReturn::ERROR;
			}
			std::size_t i = 0;
			while (i < all_names.size() && all_names[i] != name)
			{
				++i;
			}
			if (i == all_names.size())
			{
				ULOG_ERROR("%s: joints 关节 '%s' 不在 URDF position 命令表 (共 %zu 个: %s...)", kPluginName, name.c_str(), all_names.size(), all_names[0].c_str());
				return controller_interface::CallbackReturn::ERROR;
			}
			joint_names_.push_back(name);
			q_min_.push_back(all_qmin[i]);
			q_max_.push_back(all_qmax[i]);
			vmax_.push_back(all_vmax[i]);
		}
	}
	const double hz = std::max<double>(static_cast<double>(get_update_rate()), 1.0);
	// update_rate 定源 (2026-09-23, Humble 2.54.2 源码核实): yaml /** 通配节把
	// update_rate 覆盖到控制器节点, configure 期 get_update_rate() 即权威值 (旧注释
	// "configure 期取不到"的真身 = 参数只写在 controller_manager 节名下, 控制器节点
	// 匹配不到 → 0)。拿不到时占位 500Hz 先验, 激活后首 16 拍中位数兜底 (该中位数会被
	// 激活瞬间 switch 服务同步 update 的 µs period 污染, 实测 0.4ms → 2500Hz 荒唐值,
	// 故校准完成时以权威参数优先, 见 update())。
	step_limits_.clear();
	for (double v : vmax_)
	{
		step_limits_.push_back(v / (hz > 1.0 ? hz : 500.0));   // 占位; 16 拍后权威/兜底定源
	}
	cmd_.assign(joint_names_.size(), 0.0);
	target_.assign(joint_names_.size(), 0.0);

	// 观测面 (可选): 全率发布插值后的应用命令 (与 CM status 同款 RealtimePublisher)
	observe_applied_ = get_node()->get_parameter("observe_applied_command").as_bool();
	if (observe_applied_)
	{
		auto pub = get_node()->create_publisher<unistackbot_interface::msg::JointCommand>("~/applied_command", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
		rt_applied_ = std::make_shared<realtime_tools::RealtimePublisher<
			unistackbot_interface::msg::JointCommand>>(pub);
		// 预分配复用载体: RT 热路径只写 position 值, 零分配
		applied_msg_.mode = unistackbot_interface::msg::JointCommand::MODE_CSP;
		applied_msg_.joint_names = joint_names_;
		applied_msg_.position.assign(joint_names_.size(), 0.0);
		applied_msg_.velocity.clear();
		applied_msg_.effort.clear();
		applied_msg_.kp.clear();
		applied_msg_.kd.clear();
		ULOG_INFO("%s: 应用命令观测 ~/applied_command @全率 (tryPublish, 丢拍可接受)", kPluginName);
	}
	rate_calibrated_ = false;
	period_n_ = 0;

	// ruckig 档: OtgStream 装配 (编译期 16 上限, 运行期用前 n 轴)
	if (interpolation_ == Interpolation::RUCKIG)
	{
		unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints>::Limits lim;
		for (std::size_t i = 0; i < lim.max_velocity.size(); ++i)
		{
			lim.max_velocity[i] = (i < vmax_.size()) ? vmax_[i] : 3.0;   // 各关节速度限 (URDF)
		}
		lim.max_acceleration.fill(max_acceleration_);
		lim.max_jerk.fill(max_jerk_);
		if (!otg_.init(1.0 / hz, lim, 0.3))
		{
			ULOG_ERROR("%s: OtgStream init 失败", kPluginName);
			return controller_interface::CallbackReturn::ERROR;
		}
		otg_ready_ = true;
	}

	// 契约通道 (reliable+KeepLast(1), 与 CM ~/target 同款裁决)
	cmd_sub_ = get_node()->create_subscription<unistackbot_interface::msg::JointCommand>("~/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
		[this](const unistackbot_interface::msg::JointCommand::SharedPtr m)
		{
			if (m->mode != unistackbot_interface::msg::JointCommand::MODE_CSP)
			{
				++dropped_mode_;   // 仿真链仅 CSP; 其余模式计数 (真机 MIT 批次启用)
				return;
			}
			if (m->joint_names.size() != joint_names_.size() || m->position.size() < joint_names_.size())
			{
				++dropped_len_;
				return;
			}
			// 按名映射: 消息关节序 → 本控制器关节序 (缺关节整条丢弃), 产出 POD 快照
			CmdSnapshot snap{};
			snap.seq = ++cmd_seq_pub_;
			for (std::size_t i = 0; i < joint_names_.size(); ++i)
			{
				const auto it = std::find(m->joint_names.begin(), m->joint_names.end(), joint_names_[i]);
				if (it == m->joint_names.end())
				{
					++dropped_len_;
					return;
				}
				snap.position[i] = m->position[static_cast<std::size_t>(it - m->joint_names.begin())];
			}
			rt_cmd_.publish(snap);
			// 投递遥测 (2026-10-09 走廊冻结排查): 回调级计数 —— 停止增长=DDS 投递断
			if (cmd_seq_pub_ % 25 == 0)
			{
				ULOG_INFO("%s: 回调采纳 seq=%lu j4=%.3f", kPluginName, static_cast<unsigned long>(cmd_seq_pub_), snap.position[3]);
			}
		});

	ULOG_INFO("%s: %zu 关节 (%s), 插值=%s, 步长上限=%.3f rad/拍", kPluginName, joint_names_.size(), explicit_joints ? "yaml 显式列表" : "URDF 全合并", interpolation_ == Interpolation::RUCKIG ? "ruckig" : "hold",
		step_limits_.empty() ? 0.0 : step_limits_[0]);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointStreamController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	for (const auto & name : joint_names_)
	{
		conf.names.push_back(name + "/position");
	}
	return conf;
}

controller_interface::InterfaceConfiguration
JointStreamController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	for (const auto & name : joint_names_)
	{
		conf.names.push_back(name + "/position");
	}
	return conf;
}

controller_interface::CallbackReturn JointStreamController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	const std::size_t n = joint_names_.size();
	if (state_interfaces_.size() != n || command_interfaces_.size() != n)
	{
		ULOG_ERROR("%s: 接口数不匹配 (state=%zu cmd=%zu 关节=%zu)", kPluginName, state_interfaces_.size(), command_interfaces_.size(), n);
		return controller_interface::CallbackReturn::ERROR;
	}
	// 命令 = 当前状态 (激活零跳变); 目标同置 (周期步进语义下不追陈旧目标)
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		cmd_[i] = state_interfaces_[i].get_value();
	}
	target_ = cmd_;
	if (interpolation_ == Interpolation::RUCKIG && otg_ready_)
	{
		std::array<double, unistackbot_interface::kMaxJoints> q{};
		for (std::size_t i = 0; i < n; ++i) {q[i] = cmd_[i];}
		otg_.reset(q);   // 从当前位重规划 (清旧轨迹)
	}
	ULOG_INFO("%s: 激活 (%zu 关节, 命令保持)", kPluginName, n);
	return controller_interface::CallbackReturn::SUCCESS;
}

// 命令消毒: NaN 门 + 限位 clamp (步长饱和由调用方按档位施加)
bool JointStreamController::sanitize(std::vector<double> & q) const
{
	const std::size_t n = joint_names_.size();
	for (std::size_t i = 0; i < n; ++i)
	{
		if (!std::isfinite(q[i]))
		{
			return false;
		}
	}
	for (std::size_t i = 0; i < n; ++i)
	{
		q[i] = std::clamp(q[i], q_min_[i], q_max_[i]);
	}
	return true;
}

controller_interface::return_type JointStreamController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	const std::size_t n = joint_names_.size();

	// update_rate 定源 (同 on_configure 注释): 首 16 拍中位数会被激活期 switch 服务
	// 同步 update 的 µs period 污染 —— 权威参数 (/** 通配节覆盖) 优先, 中位数兜底+交叉校验
	if (!rate_calibrated_)
	{
		const double p = period.seconds();
		if (p > 1e-9)
		{
			period_samples_[period_n_++] = p;
		}
		if (period_n_ >= kPeriodSamples)
		{
			rate_calibrated_ = true;
			std::sort(period_samples_, period_samples_ + kPeriodSamples);
			const double hz_med = 1.0 / period_samples_[kPeriodSamples / 2];
			double hz = hz_med;
			const unsigned int hz_auth = get_update_rate();
			if (hz_auth > 0)
			{
				hz = static_cast<double>(hz_auth);
				if (std::fabs(hz_med - hz) / hz > 0.2)
				{
					ULOG_WARN("%s: 实测中位数 %.0fHz 偏离权威 update_rate %.0fHz"
						" (激活期 period 污染), 以参数为准", kPluginName, hz_med, hz);
				}
			}
			for (std::size_t i = 0; i < step_limits_.size(); ++i)
			{
				step_limits_[i] = vmax_[i] / hz;
			}
			if (interpolation_ == Interpolation::RUCKIG)
			{
				// otg dt 校准 (Ruckig 定长数组, init 无堆分配)
				unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints>::Limits lim;
				for (std::size_t i = 0; i < lim.max_velocity.size(); ++i)
				{
					lim.max_velocity[i] = (i < vmax_.size()) ? vmax_[i] : 3.0;
				}
				lim.max_acceleration.fill(max_acceleration_);
				lim.max_jerk.fill(max_jerk_);
				otg_ready_ = otg_.init(1.0 / hz, lim, 0.3);
				// init 会清 initialized_ (OtgStream 契约: init 后必须 reset 才有安全基准)
				// —— 不重锚 = 校准后 update 恒 Hold = ruckig 永久冻结 (2026-09-23 A/B
				// 实测实锤, 激活首 32ms 后死)。以当前命令位重锚, v/a 归零 —— 32ms
				// 窗口内速度微小, 连续性无损。
				std::array<double, unistackbot_interface::kMaxJoints> q{};
				for (std::size_t i = 0; i < n; ++i)
				{
					q[i] = cmd_[i];
				}
				otg_.reset(q);
			}
			if (hz_auth > 0)
			{
				ULOG_INFO("%s: update_rate %.0f Hz (权威参数; 步长 %.4f rad/拍)", kPluginName, hz, step_limits_.empty() ? 0.0 : step_limits_[0]);
			}
			else
			{
				// 兜底路径必须显眼 (中位数可被激活期 µs period 污染 = 35 倍减速 bug 根)
				ULOG_WARN("%s: update_rate %.0f Hz (实测中位数兜底 —— yaml 缺 /**.update_rate;"
					" 步长 %.4f rad/拍)",
					kPluginName, hz, step_limits_.empty() ? 0.0 : step_limits_[0]);
			}
		}
	}

	// ① 取最新命令 (值通道; 撕裂/无发布 → 沿用旧值 = sp_latest 契约)。
	//    新消息只**采纳目标** —— 步进在 ② 每周期执行 (消息率与控制率解耦:
	//    2026-09-21 实测教训, 步进挂在收消息上时 100Hz 流的臂速被钳到
	//    step×消息率 = vmax/5, 历史上被 hz=1 的巨步长+write 层钳位掩盖)
	CmdSnapshot snap;
	uint64_t snap_seq = 0;
	if (rt_cmd_.read(snap, snap_seq) && snap_seq != cmd_seq_)
	{
		cmd_seq_ = snap_seq;
		std::vector<double> q(snap.position, snap.position + n);
		if (sanitize(q))
		{
			target_ = q;   // hold/ruckig 同: 采纳为当前目标
			if (cmd_seq_ % 25 == 0)
			{
				ULOG_INFO("%s: update 采纳 seq=%lu j4=%.3f", kPluginName, static_cast<unsigned long>(cmd_seq_), target_[3]);
			}
		}
		else
		{
			++dropped_mode_;   // NaN/限位拒绝计数 (复用观测通道)
			ULOG_WARN("%s: sanitize 拒绝 seq=%lu j4=%.3f", kPluginName, static_cast<unsigned long>(snap_seq), q[3]);
		}
	}

	// ② 每周期推进:
	//    hold 档 = 周期步长饱和 (速度界 vmax/update_rate, 与消息率无关);
	//    ruckig 档 = OtgStream 每拍向目标生成参考 (率失配填充 + v/a/j 整形;
	//    Ok → cmd_ 前进, Hold → 内部已自愈, cmd_ 保持上一拍输出)
	if (interpolation_ == Interpolation::HOLD)
	{
		for (std::size_t i = 0; i < n; ++i)
		{
			const double dv = target_[i] - cmd_[i];
			if (dv > step_limits_[i])
			{
				cmd_[i] += step_limits_[i];
			}
			else if (dv < -step_limits_[i])
			{
				cmd_[i] -= step_limits_[i];
			}
			else
			{
				cmd_[i] = target_[i];
			}
		}
	}
	else if (interpolation_ == Interpolation::RUCKIG && otg_ready_)
	{
		std::array<double, unistackbot_interface::kMaxJoints> tgt{};
		std::array<double, unistackbot_interface::kMaxJoints> out{};
		for (std::size_t i = 0; i < n; ++i) {tgt[i] = target_[i];}
		if (otg_.update(tgt, out) == unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints>::UpdateResult::Ok)
		{
			for (std::size_t i = 0; i < n; ++i) {cmd_[i] = out[i];}
			hold_run_ = 0;
		}
		else
		{
			// Hold 遥测 (2026-10-09 走廊冻结排查加): 静默 Hold 曾让链上无法归因
			++hold_run_;
			++hold_total_;
			if (hold_run_ == 1 || hold_run_ % 500 == 0)
			{
				ULOG_WARN("%s: ruckig Hold 连续 %lu 拍 (累计 %lu, lastError=%d, 目标j4=%.3f, 输出j4=%.3f)", kPluginName, static_cast<unsigned long>(hold_run_),
					static_cast<unsigned long>(hold_total_), otg_.lastError(), target_[3], cmd_[3]);
			}
		}
	}

	// ③ 写命令
	for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
	{
		command_interfaces_[i].set_value(cmd_[i]);
	}
	publishApplied();
	return controller_interface::return_type::OK;
}

void JointStreamController::publishApplied()
{
	if (!rt_applied_)
	{
		return;   // 观测关 (默认) —— 单虚调用开销, 零分配零锁
	}
	for (std::size_t i = 0; i < applied_msg_.position.size(); ++i)
	{
		applied_msg_.position[i] = cmd_[i];
	}
	rt_applied_->tryPublish(applied_msg_);   // trylock 非阻塞; 拿不到跳本拍
}

controller_interface::CallbackReturn JointStreamController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	ULOG_INFO("%s: 停用 (接口释放, 硬件保持)", kPluginName);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStreamController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
	cmd_sub_.reset();
	rt_applied_.reset();   // 观测面随清理卸载 (CM status 同款收尾)
	return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unistackbot_controller

// 插件注册 (缺失 = "no factory exists for it" —— 类加载器找不到工厂)
PLUGINLIB_EXPORT_CLASS(unistackbot_controller::JointStreamController, controller_interface::ControllerInterface)

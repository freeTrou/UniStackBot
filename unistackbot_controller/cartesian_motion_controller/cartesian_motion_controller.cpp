#include "cartesian_motion_controller/cartesian_motion_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>

#include <tinyxml2.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "ulog/ulog.hpp"

namespace unistackbot_controller
{

namespace
{

using CallbackReturn = controller_interface::CallbackReturn;

// 四元数点积 -> 姿态角误差 [rad] (测地距离 2·acos(|q·q'|); 20Hz 状态路径, 成本可忽略)
double rotationError(const CartesianPose & a, const CartesianPose & b)
{
	const double qd = std::fabs(a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz);
	return 2.0 * std::acos(std::clamp(qd, 0.0, 1.0));
}

// 解析 URDF <ros2_control> 块每关节 max_velocity → 单拍步长上限。
// 单一事实源: 与 BackendKinematic 同参数同缺省 (5 rad/s); 控制器拿不到
// HardwareInfo (那是硬件插件的), 只能自行解 XML
// 输出原始 max_velocity (vmax; 不做 /update_rate —— Humble 坑: configure 期
// get_update_rate() 取不到真实频率, 步长换算延后到首拍校准, 见 update())
bool parseStepLimits(const std::string & urdf, const std::vector<std::string> & chain_names,
	std::vector<double> & vmax_out, std::string & err)
{
	vmax_out.assign(chain_names.size(), 5.0);
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf.c_str(), urdf.size()) != tinyxml2::XML_SUCCESS)
	{
		err = "URDF XML 解析失败 (步长限幅)";
		return false;
	}
	const tinyxml2::XMLElement * root = doc.RootElement();
	if (root == nullptr)
	{
		err = "URDF 无根元素 (步长限幅)";
		return false;
	}
	for (const tinyxml2::XMLElement * rc = root->FirstChildElement("ros2_control"); rc != nullptr;
		rc = rc->NextSiblingElement("ros2_control"))
	{
		for (const tinyxml2::XMLElement * j = rc->FirstChildElement("joint"); j != nullptr;
			j = j->NextSiblingElement("joint"))
		{
			const char * jn = j->Attribute("name");
			if (jn == nullptr) {continue;}
			const auto it = std::find(chain_names.begin(), chain_names.end(), jn);
			if (it == chain_names.end()) {continue;}
			for (const tinyxml2::XMLElement * prm = j->FirstChildElement("param"); prm != nullptr;
				prm = prm->NextSiblingElement("param"))
			{
				const char * pn = prm->Attribute("name");
				if (pn == nullptr || std::strcmp(pn, "max_velocity") != 0 ||
					prm->GetText() == nullptr)
				{
					continue;
				}
				const double v = std::atof(prm->GetText());
				if (v > 0.0)
				{
					vmax_out[static_cast<std::size_t>(it - chain_names.begin())] = v;
				}
			}
		}
	}
	return true;
}

void poseToMsg(const CartesianPose & p, geometry_msgs::msg::Pose & out)
{
	out.position.x = p.x;
	out.position.y = p.y;
	out.position.z = p.z;
	out.orientation.w = p.qw;
	out.orientation.x = p.qx;
	out.orientation.y = p.qy;
	out.orientation.z = p.qz;
}

}  // namespace

controller_interface::CallbackReturn CartesianMotionController::on_init()
{
	// 参数面 (骨架期冻结; 名字与默认值即对外契约的一部分)
	auto_declare<std::string>("base_link", "");
	auto_declare<std::string>("tip_link", "");
	auto_declare<std::string>("seed_library_package", "unistackbot_description");
	auto_declare<std::string>("seed_library_relpath", "");
	auto_declare<int64_t>("update_timeout_ns", update_timeout_ns_);
	auto_declare<int64_t>("cold_timeout_ns", cold_timeout_ns_);
	auto_declare<int>("cold_after_fails", cold_after_fails_);
	// max_cart_step: 保留位, 本步未用 —— 笛卡尔预限幅实测制造不可解中间位姿
	// (2026-09-18 探针复现), 运动限幅由 jump_threshold + 关节步长饱和承担
	auto_declare<double>("max_cart_step", max_cart_step_);
	auto_declare<int>("degraded_n", degraded_n_);
	auto_declare<int>("ik_max_iterations", ik_max_iterations_);
	auto_declare<int>("worker_cpu", worker_cpu_);
	auto_declare<int>("worker_nice", worker_nice_);
	auto_declare<double>("converge_pos_tol", converge_pos_tol_);
	auto_declare<double>("converge_rot_tol", converge_rot_tol_);
	// 断流受控减速 (0c): 0=关闭 (默认, --once 单发目标语义); 流式跟踪场景 yaml 开启
	auto_declare<double>("stale_timeout_ms", 0.0);
	auto_declare<double>("stale_decel_ms", 200.0);
	// Humble: controller_manager 把自身 robot_description 以参数覆盖注入控制器节点
	auto_declare<std::string>("robot_description", "");
	return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianMotionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	// ulog 全工程统一打印 (2026-09-17 裁决): init 幂等 + 引用计数 —— 同进程与
	// SimControlHardware 并存时互不干扰; 终端 sink 恒开 (launch 捕获 stdout)
	unistackbot_common::ulog_config log_cfg;
	log_cfg.console = true;
	log_cfg.file_path = nullptr;
	log_cfg.level = unistackbot_common::ulog_level::info;
	log_cfg.max_bytes = 0;
	log_cfg.backups = 0;
	unistackbot_common::ulog_init(log_cfg);

	base_link_ = get_node()->get_parameter("base_link").as_string();
	tip_link_ = get_node()->get_parameter("tip_link").as_string();
	seed_lib_package_ = get_node()->get_parameter("seed_library_package").as_string();
	seed_lib_relpath_ = get_node()->get_parameter("seed_library_relpath").as_string();
	update_timeout_ns_ = get_node()->get_parameter("update_timeout_ns").as_int();
	cold_timeout_ns_ = get_node()->get_parameter("cold_timeout_ns").as_int();
	max_cart_step_ = get_node()->get_parameter("max_cart_step").as_double();
	degraded_n_ = get_node()->get_parameter("degraded_n").as_int();
	cold_after_fails_ = get_node()->get_parameter("cold_after_fails").as_int();
	ik_max_iterations_ = get_node()->get_parameter("ik_max_iterations").as_int();
	worker_cpu_ = get_node()->get_parameter("worker_cpu").as_int();
	worker_nice_ = get_node()->get_parameter("worker_nice").as_int();
	converge_pos_tol_ = get_node()->get_parameter("converge_pos_tol").as_double();
	converge_rot_tol_ = get_node()->get_parameter("converge_rot_tol").as_double();
	if (base_link_.empty() || tip_link_.empty())
	{
		ULOG_ERROR("cartesian_motion_controller: base_link/tip_link 必填");
		return CallbackReturn::ERROR;
	}

	// 运动学链 (update 线程私有实例; worker 侧第 3 步另建 —— KDL 实例不跨线程)
	std::string urdf;
	if (!get_node()->get_parameter("robot_description", urdf) || urdf.empty())
	{
		ULOG_ERROR("cartesian_motion_controller: robot_description 未注入 (CM 参数覆盖缺失)");
		return CallbackReturn::ERROR;
	}
	auto fk = std::make_unique<UrdfFk>();
	std::string msg;
	if (!fk->init(urdf, base_link_, tip_link_, msg))
	{
		ULOG_ERROR("cartesian_motion_controller: %s", msg.c_str());
		return CallbackReturn::ERROR;
	}
	auto ik = std::make_unique<DlsIk>();
	if (!ik->init(fk.get(), msg))
	{
		ULOG_ERROR("cartesian_motion_controller: %s", msg.c_str());
		return CallbackReturn::ERROR;
	}
	// 种子库 (可选; relpath 空 = 不加载, 走分支+随机阶梯)
	std::string lib;
	if (!seed_lib_relpath_.empty())
	{
		try
		{
			lib = ament_index_cpp::get_package_share_directory(seed_lib_package_) +
				"/" + seed_lib_relpath_;
		}
		catch (const std::exception &)
		{
			ULOG_ERROR("cartesian_motion_controller: 种子库包不可解析: %s", seed_lib_package_.c_str());
			return CallbackReturn::ERROR;
		}
		if (!ik->loadSeedLibrary(lib, msg))
		{
			ULOG_ERROR("cartesian_motion_controller: %s", msg.c_str());
			return CallbackReturn::ERROR;
		}
		ULOG_INFO("cartesian_motion_controller: 种子库 %zu 条",
			ik->seedLibrarySize());
	}
	fk_ = std::move(fk);
	ik_ = std::move(ik);
	// worker 原料快照 (第 3 步): worker 线程自建实例用, configure 期定死
	worker_urdf_ = urdf;
	worker_lib_ = lib;

	// 流式求解配置 (防线1: 墙钟预算 + 迭代上限; 预算内实测 max 32µs, 富余 15 倍)
	DlsIkConfig ikcfg;
	ikcfg.timeout_ns = static_cast<uint64_t>(update_timeout_ns_);
	ikcfg.max_iterations = ik_max_iterations_;
	ik_->setConfig(ikcfg);
	// 步长限幅: URDF <ros2_control> max_velocity 为单一事实源; /update_rate 的换算
	// 延后到首拍校准 (Humble 坑: configure 期 get_update_rate() 取不到真实频率,
	// 实测返回 1 —— 2026-09-21 排查 F6 断流不触发时实锤)。此处占位 = vmax
	if (!parseStepLimits(urdf, fk_->jointNames(), vmax_, msg))
	{
		ULOG_ERROR("cartesian_motion_controller: %s", msg.c_str());
		return CallbackReturn::ERROR;
	}
	step_limits_ = vmax_;   // 占位 (hz=1); 首拍校准为 vmax/真实hz
	// StaleWatch: 存 ms 配置, 周期数首拍校准
	stale_ms_cfg_ = get_node()->get_parameter("stale_timeout_ms").as_double();
	stale_decel_ms_cfg_ = get_node()->get_parameter("stale_decel_ms").as_double();
	stale_cycles_ = 0u;
	stale_decel_cycles_ = 1u;
	watch_ = unistackbot_common::StaleWatch(0u);
	rate_calibrated_ = false;
	period_n_ = 0;

	// 契约通道 (先装配通道再建订阅, 回调无未初始化竞态)。
	// rt_target_ 不 init: 零态 = "从未发布" (seq=0), read() 的 false 即"无目标" ——
	// 若 init(初值) 会把初值当已发布目标, IDLE 判定失效 (2026-09-17 实测踩中)
	// QoS 裁决 (2026-09-18): 值通道 = reliable + KeepLast(1) —— 可靠到达且只留最新
	// (排队送旧目标无意义, 深度 1 让新目标即时顶替)。best_effort 发布方将不兼容,
	// 这是控制命令通道的应有代价
	target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
		"~/target", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
		[this](const geometry_msgs::msg::PoseStamped::SharedPtr m)
		{
			// v1 只收 base 系; 跨系变换是上层职责 (RViz/规划器)。非本帧丢弃,
			// 每个帧名单次警告 (warned_frame_ 仅本回调线程读写, 无竞态)
			const std::string frame = m->header.frame_id;
			if (!frame.empty() && frame != base_link_)
			{
				if (frame != warned_frame_)
				{
					warned_frame_ = frame;
					ULOG_WARN("cm: 目标帧 %s != %s, 该帧消息将被丢弃 (v1 仅 base 系)",
						frame.c_str(), base_link_.c_str());
				}
				return;
			}
			// 四元数契约 = 单位模长; 归一化入口兜底 (实测踩坑: 消费方发两位舍入的
			// 非单位四元数, 位置到位后姿态误差地板 0.03 rad, converged 永假)
			CartesianPose p;
			p.x = m->pose.position.x;
			p.y = m->pose.position.y;
			p.z = m->pose.position.z;
			p.qw = m->pose.orientation.w;
			p.qx = m->pose.orientation.x;
			p.qy = m->pose.orientation.y;
			p.qz = m->pose.orientation.z;
			const double qn = std::sqrt(p.qw * p.qw + p.qx * p.qx + p.qy * p.qy + p.qz * p.qz);
			if (qn < 1e-9)
			{
				ULOG_WARN("cm: 目标四元数为零向量, 丢弃");
				return;
			}
			p.qw /= qn; p.qx /= qn; p.qy /= qn; p.qz /= qn;
			rt_target_.publish(p);   // 值通道: 覆盖写, seq 变更即"新目标"
		});
	control_sub_ = get_node()->create_subscription<unistackbot_interface::msg::CartesianControl>(
		"~/control", rclcpp::QoS(1).transient_local(),
		[this](const unistackbot_interface::msg::CartesianControl::SharedPtr m)
		{
			control_mode_.store(m->mode, std::memory_order_release);
		});
	status_pub_ = get_node()->create_publisher<unistackbot_interface::msg::CartesianMotionStatus>(
		"~/status", rclcpp::QoS(1).transient_local());
	rt_status_ = std::make_shared<realtime_tools::RealtimePublisher<
		unistackbot_interface::msg::CartesianMotionStatus>>(status_pub_);

	ULOG_INFO("cartesian_motion_controller: 链 %s -> %s (%u 关节, update 预算 %ldns)",
		base_link_.c_str(), tip_link_.c_str(), fk_->jointCount(),
		static_cast<long>(update_timeout_ns_));
	return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianMotionController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	// 只认领 FK 链关节 (臂) 的 position 命令 —— gripper/mimic 归 JTC 与硬件 mimic 语义
	if (fk_)
	{
		for (const auto & name : fk_->jointNames())
		{
			conf.names.push_back(name + "/position");
		}
	}
	return conf;
}

controller_interface::InterfaceConfiguration CartesianMotionController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	if (fk_)
	{
		for (const auto & name : fk_->jointNames())
		{
			conf.names.push_back(name + "/position");
		}
	}
	return conf;
}

controller_interface::CallbackReturn CartesianMotionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	const std::size_t n = fk_->jointCount();
	if (state_interfaces_.size() != n || command_interfaces_.size() != n)
	{
		ULOG_ERROR("cartesian_motion_controller: 接口数不匹配 (state=%zu cmd=%zu 链=%zu)",
			state_interfaces_.size(), command_interfaces_.size(), n);
		return CallbackReturn::ERROR;
	}
	// 按名映射: <ros2_control> 接口声明序 != URDF 链序 —— 真雷, 读写必须经此映射
	const auto & chain_names = fk_->jointNames();
	iface_joint_names_.clear();
	chain_from_iface_.clear();
	bool identity = true;
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		const std::string jn = state_interfaces_[i].get_prefix_name();
		const auto it = std::find(chain_names.begin(), chain_names.end(), jn);
		if (it == chain_names.end())
		{
			ULOG_ERROR("cartesian_motion_controller: 接口关节 %s 不在 FK 链", jn.c_str());
			return CallbackReturn::ERROR;
		}
		const std::size_t chain_idx = static_cast<std::size_t>(it - chain_names.begin());
		if (chain_idx != i) {identity = false;}
		iface_joint_names_.push_back(jn);
		chain_from_iface_.push_back(chain_idx);
	}
	if (!identity)
	{
		ULOG_WARN("cartesian_motion_controller: 接口序与链序不同, 已按名映射");
	}
	// 命令初始化 = 当前状态 (激活瞬间零跳变); RT 缓冲一次定容 (周期零分配)
	cmd_.assign(n, 0.0);
	q_meas_.assign(n, 0.0);
	q_out_.assign(n, 0.0);
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		cmd_[chain_from_iface_[i]] = state_interfaces_[i].get_value();
	}
	prev_cmd_ = cmd_;
	vel_.assign(n, 0.0);
	decel_rate_.assign(n, 0.0);
	watch_.reset();   // 重激活不继承断流态
	stream_stale_ = false;
	was_stale_ = false;
	// 预热: 进 RT 前触达全部首触路径 (缺页/惰性绑定/线程私有 syscall), 见设计 §3
	warmup();
	// worker (防线2): 低优冷启动线程 —— 激活起、停用收
	worker_run_.store(true, std::memory_order_release);
	worker_ = std::thread([this]() { workerLoop(); });
	last_status_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
	publishStatus(unistackbot_interface::msg::CartesianMotionStatus::IDLE);
	ULOG_INFO("cartesian_motion_controller: 激活 (命令保持)");
	return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMotionController::update(
	const rclcpp::Time & time, const rclcpp::Duration & period)
{
	// RT 线程调优: 我们经预留的 yaml 参数接口设置 (thread_priority/cpu_affinity,
	// 参数名是 ros2_control 暴露的接口, 值是我们定的 —— 见 <robot>_controllers.yaml);
	// 控制器层不自设 (撞车: update 与全部控制器共用 CM 的同一条 RT 线程)
	const auto wcet_t0 = std::chrono::steady_clock::now();
	const std::size_t n = cmd_.size();

	// update_rate 校准 (Humble 坑, 见 on_configure 注释): 首拍 period 非稳态,
	// 收 16 拍取中位数 (定长数组零分配; 16 拍 @500Hz = 32ms, 窗内由 write 层钳位兜底)
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
			const double hz = 1.0 / period_samples_[kPeriodSamples / 2];
			for (std::size_t i = 0; i < step_limits_.size(); ++i)
			{
				step_limits_[i] = vmax_[i] / hz;
			}
			stale_cycles_ = (stale_ms_cfg_ > 0.0)
				? static_cast<uint32_t>(stale_ms_cfg_ * hz / 1000.0) : 0u;
			stale_decel_cycles_ = (stale_decel_ms_cfg_ > 0.0)
				? static_cast<uint32_t>(std::max(stale_decel_ms_cfg_ * hz / 1000.0, 1.0)) : 1u;
			watch_ = unistackbot_common::StaleWatch(stale_cycles_);
			ULOG_INFO("cm: update_rate 校准 %.0f Hz (步长 %.4f rad/拍, 断流判定 %u 拍)",
				hz, step_limits_.empty() ? 0.0 : step_limits_[0], stale_cycles_);
		}
	}

	// ⓪ 断流看门狗 (~/target 值通道 seq 零拷贝轮询; stale_cycles_=0 → 恒 LIVE 零成本)。
	//    速度估计 = 上一完整周期的实际命令步长 (快照口径: 各分支零维护, 冻结拍自然归零)
	for (std::size_t i = 0; i < n; ++i) {vel_[i] = cmd_[i] - prev_cmd_[i];}
	prev_cmd_ = cmd_;
	const auto wst = watch_.tick(rt_target_.seq());
	stream_stale_ = (wst == unistackbot_common::StaleWatch::State::STALE);
	if (stream_stale_ && !was_stale_)
	{
		ULOG_WARN("cm: ~/target 断流 (静默 %u 拍), 受控减速刹停 (%u 拍线性窗)",
			watch_.cyclesSinceUpdate(), stale_decel_cycles_);
		for (std::size_t i = 0; i < n; ++i)
		{
			decel_rate_[i] = vel_[i] / static_cast<double>(stale_decel_cycles_);
		}
	}
	else if (!stream_stale_ && was_stale_)
	{
		ULOG_INFO("cm: ~/target 流恢复");
	}
	was_stale_ = stream_stale_;

	// ① 实测关节态 (接口序 → 链序); 误差基准 = 实测 FK
	//    (gz 链命令≠实际, 用命令算误差是自欺; mock 链两者相等)
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		q_meas_[chain_from_iface_[i]] = state_interfaces_[i].get_value();
	}
	CartesianPose meas;
	const bool meas_ok = fk_->fk(q_meas_, meas, fk_scratch_);

	// ②③ 取目标: 值通道快照 (撕裂/无发布 → read false, 沿用旧值 = sp_latest 契约);
	//     HOLD 期间新目标被忽略 (事件通道语义: 冻结就是冻结)
	const bool hold = control_mode_.load(std::memory_order_acquire) ==
		unistackbot_interface::msg::CartesianControl::HOLD;
	{
		CartesianPose tgt;
		uint64_t seq = 0;
		if (rt_target_.read(tgt, seq) && seq != target_seq_)
		{
			target_seq_ = seq;
			give_up_ = false;   // 新目标 → 重启求解尝试 (旧目标的"物理不可解"结论不继承)
			if (!hold)
			{
				has_target_ = true;
				target_pose_ = tgt;
			}
		}
	}
	const uint8_t mode = hold
		? unistackbot_interface::msg::CartesianMotionStatus::HOLD
		: (has_target_ ? unistackbot_interface::msg::CartesianMotionStatus::TRACKING
				: unistackbot_interface::msg::CartesianMotionStatus::IDLE);

	// 保持分支: HOLD / 无目标 / 实测 FK 异常 —— 都不伺服, 写上一拍命令
	if (hold || !has_target_ || !meas_ok)
	{
		for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
		{
			command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
		}
		recordWcet(wcet_t0);
		publishStatusRt(time, mode, meas, meas_ok);
		return controller_interface::return_type::OK;
	}

	// ③½ 断流受控减速 (0c; 仅流式跟踪场景启用): 上游流死 → 关节速度线性衰减到停。
	//     提前出口跳过 IK (= 省 update_timeout_ns 预算); 恢复后流式从 cmd_ (已减速位)
	//     无缝续解 —— 种子即命令, 连续性不破
	if (stream_stale_)
	{
		const auto & lo = fk_->qMin();
		const auto & hi = fk_->qMax();
		for (std::size_t i = 0; i < n; ++i)
		{
			if (std::abs(vel_[i]) <= std::abs(decel_rate_[i]))
			{
				vel_[i] = 0.0;
			}
			else
			{
				vel_[i] -= decel_rate_[i];
			}
			cmd_[i] = std::clamp(cmd_[i] + vel_[i], lo[i], hi[i]);
		}
		for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
		{
			command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
		}
		recordWcet(wcet_t0);
		publishStatusRt(time, mode, meas, true);
		return controller_interface::return_type::OK;
	}

	// ④ 几何预检: 原始目标超臂展上界 → UNREACHABLE 保持 (诚实失败, 不烧预算;
	//     步长限幅版曾把目标钳到 2cm 中间点, 反而绕过了这个检查 —— 一并修正)
	const double raw_dist = std::sqrt(
		target_pose_.x * target_pose_.x + target_pose_.y * target_pose_.y +
		target_pose_.z * target_pose_.z);
	if (raw_dist > fk_->maxReach())
	{
		++consecutive_fail_;
		last_result_ = static_cast<uint8_t>(unistackbot_algorithm::IkResult::UNREACHABLE);
		last_timed_out_ = false;
		last_min_sigma_ = -1.0;
		for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
		{
			command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
		}
		recordWcet(wcet_t0);
		publishStatusRt(time, mode, meas, true);
		return controller_interface::return_type::OK;
	}

	// 分流停重试 (第 4 步): NEAR_SINGULAR×非超时 = 物理跟不动 (playbook §7 标定),
	// 已降级后每拍烧 500µs 重试是无用功 —— 目标变化前短路 (保持命令, 事实码保留)
	if (give_up_)
	{
		for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
		{
			command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
		}
		recordWcet(wcet_t0);
		publishStatusRt(time, mode, meas, true);
		return controller_interface::return_type::OK;
	}

	// ⑤ 流式 IK: 全目标直解, 种子 = 上一拍命令 (连续性根), 预算 = update_timeout_ns_。
	//     不做笛卡尔预限幅 (2026-09-18 探针裁决): nlerp 中间姿态会制造 DLS 解不动
	//     的中间位姿 (实测把求解拖进腕奇异, 全目标反而 24 迭代收敛) —— 运动限幅
	//     由 STREAMING 的 jump_threshold (关节空间连续性) + ⑦ 的步长饱和承担
	DlsIkStats st;
	const RedundancyPreference preserve;
	const auto result = ik_->solve(target_pose_, cmd_, preserve, q_out_, &st,
		unistackbot_algorithm::SolveMode::STREAMING);
	last_result_ = static_cast<uint8_t>(result);
	last_timed_out_ = st.timed_out;
	last_min_sigma_ = st.min_sigma;

	if (result == unistackbot_algorithm::IkResult::OK)
	{
		if (applySolution(q_out_.data()))
		{
			consecutive_fail_ = 0;
		}
		else
		{
			++consecutive_fail_;   // NaN 门: 整拍保持 (dls_ik 契约已保证, 此为皮带扣)
		}
	}
	else
	{
		// 冷启动回灌 (防线2 收口): worker 为**当前目标** (seq 对账) 解出的分支解,
		// 采纳为本拍解走同一安全层 —— 步长饱和保证物理运动仍在速度界内 (无跳变);
		// 采纳后 cmd_ 已挪近解, 下拍流式从新 cmd_ 起通常自行接管
		ColdResult res;
		uint64_t res_seq = 0;
		if (cold_res_.read(res, res_seq) && res.ok && res.seq == target_seq_ &&
			res.n == static_cast<uint32_t>(n) && applySolution(res.q))
		{
			consecutive_fail_ = 0;
			last_result_ = static_cast<uint8_t>(unistackbot_algorithm::IkResult::OK);
			last_min_sigma_ = res.min_sigma;
			last_timed_out_ = false;
		}
		else
		{
			// 失败不改输出 (dls_ik 契约) → 保持上一拍命令
			++consecutive_fail_;
			// 分流停重试: 已降级 (consecutive_fail_ ≥ degraded_n_) 且失败原因是
			// NEAR_SINGULAR×非超时 (物理不可解, 冷启动也救不了) → 放弃直至目标变化
			if (consecutive_fail_ >= degraded_n_ &&
				result == unistackbot_algorithm::IkResult::NEAR_SINGULAR && !st.timed_out)
			{
				give_up_ = true;
				ULOG_WARN("cm: 物理不可解 (NEAR_SINGULAR, 冷启动亦无解), 停止重试直至目标变化");
			}
			// 升级 (防线2 触发): 流式连续失败达阈值 → 请求 worker 冷启动
			// (同目标只求一次, last_cold_req_seq_ 去重; 种子 = 实测关节)
			if (consecutive_fail_ >= cold_after_fails_ && target_seq_ != last_cold_req_seq_)
			{
				ColdRequest rq{};
				rq.target = target_pose_;
				rq.n = static_cast<uint32_t>(n);
				rq.seq = target_seq_;
				for (std::size_t i = 0; i < n; ++i)
				{
					rq.q[i] = q_meas_[i];
				}
				cold_req_.publish(rq);
				last_cold_req_seq_ = target_seq_;
				ULOG_INFO("cm: 流式连续失败 %d 拍, 请求冷启动 (预算 %ldns)",
					consecutive_fail_, static_cast<long>(cold_timeout_ns_));
			}
		}
	}

	// ⑧ 写命令 + 状态发布
	for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
	{
		command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
	}
	recordWcet(wcet_t0);
	publishStatusRt(time, mode, meas, true);
	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianMotionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	worker_run_.store(false, std::memory_order_release);
	if (worker_.joinable())
	{
		worker_.join();   // 循环 2ms 一拍, join 有界
	}
	// WCET 终报 (第 5 步验收证据): 预算 2ms, 防线1 的生效证据
	if (update_count_ > 0 && !wcet_samples_.empty())
	{
		std::vector<double> srt = wcet_samples_;
		std::sort(srt.begin(), srt.end());
		const double p50 = srt[srt.size() / 2];
		const double p99 = srt[srt.size() * 99 / 100];
		ULOG_INFO("cm: WCET 终报 %lu 拍: p50=%.1fµs p99=%.1fµs max=%.1fµs (预算 2000µs)",
			static_cast<unsigned long>(update_count_), p50, p99, wcet_max_us_);
	}
	wcet_samples_.clear();
	has_target_ = false;   // 重激活不追陈旧目标 (防跳变); 失败计数同步清零
	consecutive_fail_ = 0;
	give_up_ = false;
	publishStatus(unistackbot_interface::msg::CartesianMotionStatus::INACTIVE);
	ULOG_INFO("cartesian_motion_controller: 停用 (接口释放, 硬件保持)");
	return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianMotionController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
	rt_status_.reset();
	status_pub_.reset();
	control_sub_.reset();
	target_sub_.reset();
	ik_.reset();
	fk_.reset();
	return CallbackReturn::SUCCESS;
}

// WCET 记账: 每拍两次 steady_clock + 会话样本 (停用时算分位, 第 5 步验收工具)
void CartesianMotionController::recordWcet(
	const std::chrono::steady_clock::time_point & t0)
{
	const double us = std::chrono::duration<double, std::micro>(
		std::chrono::steady_clock::now() - t0).count();
	wcet_sum_us_ += us;
	if (us > wcet_max_us_)
	{
		wcet_max_us_ = us;
	}
	wcet_samples_.push_back(us);
	++update_count_;
}

// RT 周期状态发布: ~20Hz 节流, 消息锁外本地构建, tryPublish 单次拷贝 (零阻塞)
void CartesianMotionController::publishStatusRt(
	const rclcpp::Time & time, uint8_t mode, const CartesianPose & meas, bool meas_ok)
{
	if ((time - last_status_time_).seconds() < 0.05 || !rt_status_)
	{
		return;
	}
	last_status_time_ = time;
	unistackbot_interface::msg::CartesianMotionStatus m;
	m.header.stamp = time;
	m.header.frame_id = base_link_;
	m.mode = mode;
	const bool tracking = has_target_ &&
		mode == unistackbot_interface::msg::CartesianMotionStatus::TRACKING;
	if (tracking && meas_ok)
	{
		const double dx = meas.x - target_pose_.x;
		const double dy = meas.y - target_pose_.y;
		const double dz = meas.z - target_pose_.z;
		m.position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
		m.rotation_error = rotationError(meas, target_pose_);
		m.converged = m.position_error < converge_pos_tol_ && m.rotation_error < converge_rot_tol_;
	}
	else
	{
		m.position_error = 0.0;
		m.rotation_error = 0.0;
		m.converged = false;
	}
	m.min_sigma = last_min_sigma_;
	m.timed_out = last_timed_out_;
	m.stream_stale = stream_stale_;
	m.last_result = last_result_;
	if (meas_ok)
	{
		poseToMsg(meas, m.current_pose);
	}
	poseToMsg(tracking ? target_pose_ : meas, m.target_pose);
	rt_status_->tryPublish(m);
}

// IK 解走安全层 (流式/冷启动回灌共用): NaN 门 → 限位 clamp → 步长饱和。
// 「不论 IK 怎么算, 下发的命令必须合法保守」—— 与 IK 实现解耦的保证
bool CartesianMotionController::applySolution(const double * q)
{
	const std::size_t n = cmd_.size();
	for (std::size_t i = 0; i < n; ++i)
	{
		if (!std::isfinite(q[i]))
		{
			return false;   // NaN 门: 整拍保持
		}
	}
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	for (std::size_t i = 0; i < n; ++i)
	{
		double v = std::clamp(q[i], lo[i], hi[i]);
		const double dv = v - cmd_[i];
		if (dv > step_limits_[i])
		{
			v = cmd_[i] + step_limits_[i];
		}
		else if (dv < -step_limits_[i])
		{
			v = cmd_[i] - step_limits_[i];
		}
		cmd_[i] = v;
	}
	return true;
}

// worker 线程 (第 3 步, 防线2): 冷启动出环。2ms 轮询值通道 (冷路径延迟预算是
// ms 级, 轮询够用且免掉 condvar 复杂度); 自建一套 UrdfFk+DlsIk —— KDL 求解器
// 持有迭代暂存成员, 跨线程并发互踩 (urdf_fk.hpp 用法契约), 实例必须线程私有。
// 种子库在本线程加载: 解析线性读全文件, 页天然全触 (加载即预热)。
void CartesianMotionController::workerLoop()
{
	// worker 调优全参数化 (yaml: worker_cpu/worker_nice): 默认 nice+10 让路姿态
	unistackbot_common::rt_tune::apply(worker_cpu_, 0, worker_nice_, "cm_cold");
	UrdfFk fk;
	std::string msg;
	if (!fk.init(worker_urdf_, base_link_, tip_link_, msg))
	{
		ULOG_ERROR("cm worker: 建链失败 (%s)", msg.c_str());
		return;
	}
	DlsIk ik;
	if (!ik.init(&fk, msg))
	{
		ULOG_ERROR("cm worker: %s", msg.c_str());
		return;
	}
	if (!worker_lib_.empty() && !ik.loadSeedLibrary(worker_lib_, msg))
	{
		ULOG_WARN("cm worker: 种子库加载失败, 走分支+随机阶梯 (%s)", msg.c_str());
	}
	DlsIkConfig cfg;
	cfg.timeout_ns = static_cast<uint64_t>(cold_timeout_ns_);
	cfg.max_iterations = 200;   // 冷启动世界: 不限时档的配置 (阶梯自由磨)
	ik.setConfig(cfg);
	// 预热: 本线程首条日志 (cached_tid 是线程私有的) + 一次假冷启动
	{
		std::vector<double> q2(fk.jointCount(), 0.0);
		const auto & lo = fk.qMin();
		const auto & hi = fk.qMax();
		for (std::size_t i = 0; i < q2.size(); ++i)
		{
			q2[i] = std::clamp((lo[i] + hi[i]) / 2 + 0.1, lo[i], hi[i]);
		}
		CartesianPose t;
		if (fk.fk(q2, t))
		{
			std::vector<double> out = q2;
			// 结果有意丢弃: 就绪探测只求触达求解路径 (页/分支), 不消费解
			(void)ik.solve(t, q2, RedundancyPreference{}, out, nullptr,
				unistackbot_algorithm::SolveMode::COLD_START);
		}
		ULOG_INFO("cm worker: 就绪 (%u 关节, 库 %zu 条, 预算 %ldns)",
			fk.jointCount(), ik.seedLibrarySize(), static_cast<long>(cold_timeout_ns_));
	}
	ColdRequest req;
	uint64_t last = 0;
	uint64_t seq = 0;
	while (worker_run_.load(std::memory_order_acquire))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		if (!cold_req_.read(req, seq) || seq == last)
		{
			continue;
		}
		last = seq;
		std::vector<double> seed(req.q, req.q + req.n);
		std::vector<double> out = seed;
		DlsIkStats st;
		const auto r = ik.solve(req.target, seed, RedundancyPreference{}, out, &st,
			unistackbot_algorithm::SolveMode::COLD_START);
		ColdResult res{};
		res.seq = req.seq;
		res.n = req.n;
		res.ok = (r == unistackbot_algorithm::IkResult::OK);
		res.min_sigma = st.min_sigma;
		for (uint32_t i = 0; i < req.n && i < unistackbot_interface::kMaxJoints; ++i)
		{
			res.q[i] = out[i];
		}
		cold_res_.publish(res);
		ULOG_INFO("cm worker: 冷启动%s (σ=%.3f, %.0fµs)",
			res.ok ? "成功" : "失败", st.min_sigma, st.solve_us);
	}
	ULOG_INFO("cm worker: 退出");
}

// 激活末尾预热: 把首触成本全部留在非 RT 阶段 (RT 线程纪律 = 零意外)。
// 诚实边界: 失败阶梯 (重启路径/求解 WARN) 不预热 —— 仅失败后触达,
// 首现成本已被 timeout 封顶; 缺页验收 (1000 拍 Δminflt==0) 在第 5 步工具侧
void CartesianMotionController::warmup()
{
	// 1) 日志四级: 触达 cached_tid 首 syscall / 环首触 / 四级格式化路径
	//    (直接打 DEBUG 在 info 级下会被宏首句过滤, 什么也触不到 —— 先临时调级)
	unistackbot_common::ulog_set_level(unistackbot_common::ulog_level::debug);
	ULOG_DEBUG("cm warmup: debug 路径");
	ULOG_INFO("cm warmup: info 路径");
	ULOG_WARN("cm warmup: warn 路径");
	ULOG_ERROR("cm warmup: error 路径");
	unistackbot_common::ulog_set_level(unistackbot_common::ulog_level::info);

	// 2) 可达假目标真 solve: FK(cmd_+小扰动) 为目标 → 必经迭代/SVD/入口分配
	//    (拿 FK(cmd_) 原值作目标会在迭代前收敛, 触不到 SVD)。结果丢弃, 不改 cmd_
	{
		std::vector<double> q2 = cmd_;
		const auto & lo = fk_->qMin();
		const auto & hi = fk_->qMax();
		for (std::size_t i = 0; i < q2.size(); ++i)
		{
			q2[i] = std::clamp(q2[i] + 0.05, lo[i], hi[i]);
		}
		CartesianPose t;
		if (fk_->fk(q2, t, fk_scratch_))
		{
			std::vector<double> q_out = cmd_;
			const RedundancyPreference preserve;
			// 结果有意丢弃: 预热只求触达路径, 不消费解
			(void)ik_->solve(t, cmd_, preserve, q_out, nullptr,
				unistackbot_algorithm::SolveMode::STREAMING);
		}
	}
	// 3) 不可达假目标: 几何预检路径 (结果丢弃)
	{
		CartesianPose far;
		far.x = fk_->maxReach() + 0.5;
		far.qw = 1.0;
		std::vector<double> q_out = cmd_;
		const RedundancyPreference preserve;
		// 结果有意丢弃: 预热只求触达路径, 不消费解
		(void)ik_->solve(far, cmd_, preserve, q_out, nullptr,
			unistackbot_algorithm::SolveMode::STREAMING);
	}
	// 4) RealtimePublisher 首拍 (内部互斥与发布路径)
	{
		unistackbot_interface::msg::CartesianMotionStatus m;
		m.mode = unistackbot_interface::msg::CartesianMotionStatus::IDLE;
		rt_status_->tryPublish(m);
	}
}

void CartesianMotionController::publishStatus(uint8_t mode)
{
	if (!status_pub_)
	{
		return;
	}
	unistackbot_interface::msg::CartesianMotionStatus m;
	m.header.stamp = get_node()->now();
	m.header.frame_id = base_link_;
	m.mode = mode;
	CartesianPose cur;
	if (fk_ && fk_->fk(cmd_, cur, fk_scratch_))
	{
		poseToMsg(cur, m.current_pose);
		poseToMsg(cur, m.target_pose);
	}
	status_pub_->publish(m);
}

}  // namespace unistackbot_controller

PLUGINLIB_EXPORT_CLASS(unistackbot_controller::CartesianMotionController, controller_interface::ControllerInterface)

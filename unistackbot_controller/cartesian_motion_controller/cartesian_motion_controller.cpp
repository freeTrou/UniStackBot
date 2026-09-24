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

// 解析 URDF <ros2_control> 块每关节 max_velocity。
// 单一事实源: 与 BackendKinematic 同参数同缺省 (5 rad/s); 控制器拿不到
// HardwareInfo (那是硬件插件的), 只能自行解 XML。
// 用途 (§16.6 改版): OtgStream Limits 的 max_velocity 源 + 安全钳位步长基准。
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
	// 参数面 (名字即对外契约的一部分; §16.6 改版: 旧 RT 预算/worker 族参数退役,
	// OTG 机型资产 + 回调 IK 预算为必填无默认族)
	auto_declare<std::string>("base_link", "");
	auto_declare<std::string>("tip_link", "");
	auto_declare<std::string>("ik_solver", "");     // IkSolver 实现: 必填无默认 (算法选择显式手动)
	auto_declare<std::string>("seed_library_package", "unistackbot_description");
	auto_declare<std::string>("seed_library_relpath", "");
	auto_declare<double>("ik_timeout_ms", 0.0);     // 回调线程单次 IK 墙钟预算 (必填)
	auto_declare<double>("max_acceleration", 0.0);  // OTG 关节加速度界 (机型资产, 必填)
	auto_declare<double>("max_jerk", 0.0);          // OTG 关节加加速度界 (必填)
	auto_declare<int>("degraded_n", degraded_n_);
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
	ik_timeout_ms_ = get_node()->get_parameter("ik_timeout_ms").as_double();
	max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
	max_jerk_ = get_node()->get_parameter("max_jerk").as_double();
	degraded_n_ = get_node()->get_parameter("degraded_n").as_int();
	converge_pos_tol_ = get_node()->get_parameter("converge_pos_tol").as_double();
	converge_rot_tol_ = get_node()->get_parameter("converge_rot_tol").as_double();
	if (base_link_.empty() || tip_link_.empty())
	{
		ULOG_ERROR("cartesian_motion_controller: base_link/tip_link 必填");
		return CallbackReturn::ERROR;
	}
	if (ik_timeout_ms_ <= 0.0 || max_acceleration_ <= 0.0 || max_jerk_ <= 0.0)
	{
		ULOG_ERROR("cm: OTG 机型资产/IK 预算缺显式配置或非法 (ik_timeout_ms=%.3g, "
			"max_acceleration=%.3g, max_jerk=%.3g; 均须 > 0, yaml 显式声明)",
			ik_timeout_ms_, max_acceleration_, max_jerk_);
		return CallbackReturn::ERROR;
	}

	// 运动学 (实例线程归属 §16.6: fk_=RT 侧 status 误差, ik_=回调线程侧求解;
	// init 均在 configure 期单线程完成, 激活后两侧各用各的 —— KDL 暂存不跨线程)
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
	// 求解器选择 (IkSolver 接口): 数值类实现平级替换/AB 对比; 选择必须显式
	// (yaml 声明, 代码无默认 —— 算法↔机型对应永远是人手动定的, 2026-09-22)
	ik_solver_name_ = get_node()->get_parameter("ik_solver").as_string();
	if (ik_solver_name_.empty())
	{
		ULOG_ERROR("cm: 缺 ik_solver 显式配置 (算法选择必须显式手动; 当前可选: dls | analytic_piper)");
		return CallbackReturn::ERROR;
	}
	std::unique_ptr<IkSolver> ik;
	if (ik_solver_name_ == "dls")
	{
		ik = std::make_unique<DlsIk>();
	}
	else if (ik_solver_name_ == "analytic_piper")
	{
		ik = std::make_unique<AnalyticPiper>();
	}
	else
	{
		ULOG_ERROR("cm: 未知 ik_solver '%s' (可选: dls | analytic_piper)", ik_solver_name_.c_str());
		return CallbackReturn::ERROR;
	}
	ULOG_INFO("cm: IK 求解器 = '%s' (显式配置, 回调线程逐条解)", ik_solver_name_.c_str());
	if (!ik->init(fk.get(), msg))
	{
		ULOG_ERROR("cartesian_motion_controller: %s", msg.c_str());
		return CallbackReturn::ERROR;
	}
	// 种子库 (可选; relpath 空 = 不加载, 走分支+随机阶梯)
	if (!seed_lib_relpath_.empty())
	{
		std::string lib;
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
		ULOG_INFO("cartesian_motion_controller: 种子库 %zu 条", ik->seedLibrarySize());
	}
	// 回调线程求解配置 (旧 worker 档整编: 不限时档 —— 大预算 + 高迭代上限;
	// DlsIkConfig 是 DLS 专属, 经具体类施加)
	if (auto * dls = dynamic_cast<DlsIk *>(ik.get()))
	{
		DlsIkConfig ikcfg;
		ikcfg.timeout_ns = static_cast<uint64_t>(ik_timeout_ms_ * 1e6);
		ikcfg.max_iterations = 200;
		dls->setConfig(ikcfg);
	}
	fk_ = std::move(fk);
	ik_ = std::move(ik);

	// vmax (URDF max_velocity 单一事实源): OtgStream Limits 源 + 安全钳位基准;
	// /update_rate 换算延后到首拍定源 (权威参数优先, 见 update())
	if (!parseStepLimits(urdf, fk_->jointNames(), vmax_, msg))
	{
		ULOG_ERROR("cartesian_motion_controller: %s", msg.c_str());
		return CallbackReturn::ERROR;
	}
	step_limits_ = vmax_;   // 占位; 首拍定源后为 vmax/hz
	// StaleWatch: 存 ms 配置, 周期数首拍定源
	stale_ms_cfg_ = get_node()->get_parameter("stale_timeout_ms").as_double();
	stale_decel_ms_cfg_ = get_node()->get_parameter("stale_decel_ms").as_double();
	stale_cycles_ = 0u;
	watch_ = unistackbot_common::StaleWatch(0u);
	rate_calibrated_ = false;
	period_n_ = 0;
	otg_ready_ = false;

	// 契约通道 (先装配通道再建订阅, 回调无未初始化竞态)。
	// QoS 裁决 (2026-09-18): 值通道 = reliable + KeepLast(1) —— 可靠到达且只留最新。
	// §16.6: 回调内联一次 IK (种子=实测通道), 结果关节终点入值通道 —— RT 环零求解
	const std::uint32_t chain_n = fk_->jointCount();
	cb_seed_.assign(chain_n, 0.0);
	cb_q_.assign(chain_n, 0.0);
	target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
		"~/target", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
		[this, chain_n](const geometry_msgs::msg::PoseStamped::SharedPtr m)
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
			// 四元数契约 = 单位模长; 归一化入口兜底
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

			SolvedTarget sol{};
			sol.target = p;
			sol.n = chain_n;
			// 几何预检: 超臂展上界 → 诚实 UNREACHABLE (不烧求解预算)
			const double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
			if (dist > fk_->maxReach())
			{
				sol.ok = false;
				sol.result = static_cast<uint8_t>(unistackbot_algorithm::IkResult::UNREACHABLE);
				sol.min_sigma = -1.0;
				solved_ch_.publish(sol);
				return;
			}
			// 种子 = 最新实测 (RT 每拍覆盖写; 激活前通道空 → 零位回退) ——
			// 每条消息重解 = 闭环补偿 (物理链下垂按消息率追回)
			MeasJoints mj;
			uint64_t mseq = 0;
			for (std::size_t i = 0; i < cb_seed_.size(); ++i) {cb_seed_[i] = 0.0;}
			if (meas_ch_.read(mj, mseq) && mj.n == chain_n)
			{
				for (std::uint32_t i = 0; i < chain_n; ++i) {cb_seed_[i] = mj.q[i];}
			}
			cb_q_ = cb_seed_;
			DlsIkStats st;
			const RedundancyPreference preserve;
			const auto result = ik_->solve(p, cb_seed_, preserve, cb_q_, &st,
				unistackbot_algorithm::SolveMode::COLD_START);
			sol.ok = (result == unistackbot_algorithm::IkResult::OK);
			sol.result = static_cast<uint8_t>(result);
			sol.timed_out = st.timed_out;
			sol.min_sigma = st.min_sigma;
			for (std::uint32_t i = 0; i < chain_n && i < unistackbot_interface::kMaxJoints; ++i)
			{
				sol.q[i] = cb_q_[i];
			}
			solved_ch_.publish(sol);
			if (!sol.ok)
			{
				// 诚实拒绝要可见但不刷屏 (流式上游持续不可达时按条数限流)
				static thread_local uint64_t rej_n = 0;
				if (rej_n++ % 50 == 0)
				{
					ULOG_WARN("cm: IK 未解出 (result=%u, 第 %lu 条不可达类目标; 保持上一目标)",
						sol.result, static_cast<unsigned long>(rej_n));
				}
			}
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

	ULOG_INFO("cartesian_motion_controller: 链 %s -> %s (%u 关节, 回调 IK 预算 %.0fms, "
		"OTG a=%.1f j=%.1f)",
		base_link_.c_str(), tip_link_.c_str(), fk_->jointCount(),
		ik_timeout_ms_, max_acceleration_, max_jerk_);
	return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianMotionController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	// 只认领 FK 链关节 (臂) 的 position 命令 —— gripper/mimic 归硬件 mimic 语义
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
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		cmd_[chain_from_iface_[i]] = state_interfaces_[i].get_value();
	}
	watch_.reset();   // 重激活不继承断流态
	stream_stale_ = false;
	was_stale_ = false;
	// 重激活防重放 (同 OTG 门/worker 先例): SpLatest 留着上一次激活期的解算结果,
	// 预读残值记 seq 水位并弃用 —— 重激活不追陈旧目标 (臂可能已被别处动过)
	SolvedTarget residual;
	(void)solved_ch_.read(residual, target_seq_);
	has_target_ = false;
	have_goal_ = false;
	consecutive_fail_ = 0;
	goal_.fill(0.0);
	rate_calibrated_ = false;
	period_n_ = 0;
	otg_ready_ = false;
	// 预热: 进 RT 前触达全部首触路径 (缺页/惰性绑定/线程私有 syscall), 见设计 §3
	warmup();
	last_status_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
	publishStatus(unistackbot_interface::msg::CartesianMotionStatus::IDLE);
	ULOG_INFO("cartesian_motion_controller: 激活 (命令保持; 等待 update_rate 定源)");
	return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMotionController::update(
	const rclcpp::Time & time, const rclcpp::Duration & period)
{
	const auto wcet_t0 = std::chrono::steady_clock::now();
	const std::size_t n = cmd_.size();

	// ⓪ 实测关节态 (接口序 → 链序) —— 先读: 定源后 OTG 基准/回调种子/status 误差三用
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		q_meas_[chain_from_iface_[i]] = state_interfaces_[i].get_value();
	}
	// 回调 IK 种子通道 (每拍覆盖写, wait-free; 撕裂时回调沿用旧值 = sp_latest 契约)
	{
		MeasJoints mj{};
		mj.n = static_cast<uint32_t>(n);
		for (std::size_t i = 0; i < n && i < unistackbot_interface::kMaxJoints; ++i)
		{
			mj.q[i] = q_meas_[i];
		}
		meas_ch_.publish(mj);
	}

	// ⓪½ update_rate 定源 (2026-09-23, Humble 2.54.2 源码核实): yaml 的 /** 通配节把
	// update_rate 覆盖到控制器节点, configure 期即读入 → get_update_rate() 权威值。
	// 首 16 拍中位数降为交叉校验 + 无覆盖时兜底。定源完成同拍装配 OTG 输出级
	// (init 后必须 reset 才有安全基准 —— OtgStream 契约, JS 冻结 bug 同族防线)。
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
					ULOG_WARN("cm: 实测中位数 %.0fHz 偏离权威 update_rate %.0fHz"
						" (激活期 period 污染), 以参数为准", hz_med, hz);
				}
			}
			for (std::size_t i = 0; i < step_limits_.size(); ++i)
			{
				step_limits_[i] = vmax_[i] / hz;
			}
			stale_cycles_ = (stale_ms_cfg_ > 0.0)
				? static_cast<uint32_t>(stale_ms_cfg_ * hz / 1000.0) : 0u;
			watch_ = unistackbot_common::StaleWatch(stale_cycles_);
			// OTG 输出级装配 (§16.6): vmax=URDF 单一事实源; a/j=机型资产;
			// max_target_jump 0.3 = 关节目标跳变容忍 (每条消息一解, 分支翻转界)
			unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints>::Limits lim;
			for (std::size_t i = 0; i < lim.max_velocity.size(); ++i)
			{
				lim.max_velocity[i] = (i < vmax_.size()) ? vmax_[i] : 3.0;
			}
			lim.max_acceleration.fill(max_acceleration_);
			lim.max_jerk.fill(max_jerk_);
			if (!otg_.init(1.0 / hz, lim, 0.3))
			{
				ULOG_ERROR("cm: OtgStream init 失败 (限值/dt 非法)");
				return controller_interface::return_type::ERROR;
			}
			std::array<double, unistackbot_interface::kMaxJoints> q0{};
			for (std::size_t i = 0; i < n; ++i) {q0[i] = q_meas_[i];}
			otg_.reset(q0);   // 实测位锚定: 激活零跳变
			otg_ready_ = true;
			if (hz_auth > 0)
			{
				ULOG_INFO("cm: update_rate %.0f Hz (权威参数; OTG 就绪, 步长 %.4f rad/拍)",
					hz, step_limits_.empty() ? 0.0 : step_limits_[0]);
			}
			else
			{
				// 兜底路径必须显眼 (中位数可被激活期 µs period 污染 = 35 倍减速 bug 根)
				ULOG_WARN("cm: update_rate %.0f Hz (实测中位数兜底 —— yaml 缺 /**.update_rate;"
					" OTG 就绪, 步长 %.4f rad/拍)", hz, step_limits_.empty() ? 0.0 : step_limits_[0]);
			}
		}
	}

	// ① 实测 FK (status 误差基准; gz 链命令≠实际, 用命令算误差是自欺)
	CartesianPose meas;
	const bool meas_ok = fk_->fk(q_meas_, meas, fk_scratch_);

	// ② 断流看门狗 (解算通道 seq 零拷贝轮询; stale_cycles_=0 → 恒 LIVE)。
	//     断流处置 = OtgStream 自目标刹停 (C2; JS ruckig 档同款) —— 旧线性减速窗机制
	//     随速度估计族 (vel_/decel_rate_) 一并退役
	const auto wst = watch_.tick(solved_ch_.seq());
	stream_stale_ = (wst == unistackbot_common::StaleWatch::State::STALE);
	if (stream_stale_ && !was_stale_)
	{
		ULOG_WARN("cm: ~/target 断流 (静默 %u 拍), OtgStream 自目标刹停 (C2)",
			watch_.cyclesSinceUpdate());
	}
	else if (!stream_stale_ && was_stale_)
	{
		ULOG_INFO("cm: ~/target 流恢复");
	}
	was_stale_ = stream_stale_;

	// ③ 取最新解算 (值通道; 撕裂/无发布 → read false 沿用旧值)。HOLD 期间新目标被
	//     忽略 (事件通道语义: 冻结就是冻结); 失败载荷保持上一关节终点 (连续性根)
	const bool hold = control_mode_.load(std::memory_order_acquire) ==
		unistackbot_interface::msg::CartesianControl::HOLD;
	{
		SolvedTarget sol;
		uint64_t seq = 0;
		if (solved_ch_.read(sol, seq) && seq != target_seq_)
		{
			target_seq_ = seq;
			last_result_ = sol.result;
			last_timed_out_ = sol.timed_out;
			last_min_sigma_ = sol.min_sigma;
			if (sol.n == static_cast<uint32_t>(n) && sol.ok)
			{
				consecutive_fail_ = 0;
				for (std::size_t i = 0; i < n; ++i) {goal_[i] = sol.q[i];}
				have_goal_ = true;
			}
			else
			{
				++consecutive_fail_;
			}
			if (!hold)
			{
				has_target_ = true;
				target_pose_ = sol.target;
			}
		}
	}
	const uint8_t mode = hold
		? unistackbot_interface::msg::CartesianMotionStatus::HOLD
		: (!has_target_ ? unistackbot_interface::msg::CartesianMotionStatus::IDLE
			: (consecutive_fail_ >= degraded_n_
				? unistackbot_interface::msg::CartesianMotionStatus::DEGRADED
				: unistackbot_interface::msg::CartesianMotionStatus::TRACKING));

	// ④ 输出级: OTG 塑形 → 安全钳位 → 写接口。定源前 (32ms 窗) 保持命令。
	if (!otg_ready_)
	{
		for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
		{
			command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
		}
		recordWcet(wcet_t0);
		publishStatusRt(time, mode, meas, meas_ok);
		return controller_interface::return_type::OK;
	}
	{
		// 目标选择: HOLD/断流/无解算目标/FK 异常 → 自目标 (= 保持/刹停, C2 停在原地);
		// 正常 → 最新关节终点 goal_ (每拍重规划 = 可打断原生语义, §16.6)
		std::array<double, unistackbot_interface::kMaxJoints> tgt{};
		const bool servo = !hold && !stream_stale_ && have_goal_ && meas_ok;
		if (servo)
		{
			tgt = goal_;
		}
		else
		{
			for (std::size_t i = 0; i < n; ++i) {tgt[i] = cmd_[i];}
		}
		std::array<double, unistackbot_interface::kMaxJoints> out{};
		(void)otg_.update(tgt, out);   // Hold 时 out 恒有效 (OtgStream 契约)
		(void)applySolution(out.data());   // 皮带扣: NaN 门+限位+步长 (Limits 之上)
	}
	for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
	{
		command_interfaces_[i].set_value(cmd_[chain_from_iface_[i]]);
	}
	recordWcet(wcet_t0);
	publishStatusRt(time, mode, meas, meas_ok);
	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CartesianMotionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	// WCET 终报 (验收证据): RT 环只剩 OTG+FK+发布, 预算余量应显著大于旧 IK 环
	if (update_count_ > 0 && !wcet_samples_.empty())
	{
		std::vector<double> srt = wcet_samples_;
		std::sort(srt.begin(), srt.end());
		const double p50 = srt[srt.size() / 2];
		const double p99 = srt[srt.size() * 99 / 100];
		ULOG_INFO("cm: WCET 终报 %lu 拍: p50=%.1fµs p99=%.1fµs max=%.1fµs (OTG 输出级)",
			static_cast<unsigned long>(update_count_), p50, p99, wcet_max_us_);
	}
	wcet_samples_.clear();
	update_count_ = 0;   // 拍数随样本同清 (跨激活累计曾把 538+510 报成"1048 拍")
	has_target_ = false;   // 重激活不追陈旧目标 (防跳变); 失败计数同步清零
	have_goal_ = false;
	consecutive_fail_ = 0;
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

// WCET 记账: 每拍两次 steady_clock + 会话样本 (停用时算分位, 验收工具)
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

// 输出安全钳位 (皮带扣): NaN 门 → 限位 clamp → 步长饱和。「不论上游怎么算, 下发的
// 命令必须合法保守」—— OtgStream Limits 已是主约束 (vmax/a/j), 此层防通道撕裂
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

// 激活末尾预热: 把首触成本全部留在非 RT 阶段 (RT 线程纪律 = 零意外)。
// 注: IK 在回调线程 (与生命周期回调同 executor 线程串行, 无竞态); 预热触达 COLD_START
// 路径 (= 回调真实路径) + RealtimePublisher 首拍
void CartesianMotionController::warmup()
{
	// 1) 日志四级: 触达 cached_tid 首 syscall / 环首触 / 四级格式化路径
	unistackbot_common::ulog_set_level(unistackbot_common::ulog_level::debug);
	ULOG_DEBUG("cm warmup: debug 路径");
	ULOG_INFO("cm warmup: info 路径");
	ULOG_WARN("cm warmup: warn 路径");
	ULOG_ERROR("cm warmup: error 路径");
	unistackbot_common::ulog_set_level(unistackbot_common::ulog_level::info);

	// 2) 可达假目标真 solve (FK(cmd_+小扰动) 为目标 → 必经迭代/SVD); 结果丢弃
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
			(void)ik_->solve(t, cmd_, preserve, q_out, nullptr,
				unistackbot_algorithm::SolveMode::COLD_START);
		}
	}
	// 3) 不可达假目标: 几何预检路径 (结果丢弃)
	{
		CartesianPose far;
		far.x = fk_->maxReach() + 0.5;
		far.qw = 1.0;
		std::vector<double> q_out = cmd_;
		const RedundancyPreference preserve;
		(void)ik_->solve(far, cmd_, preserve, q_out, nullptr,
			unistackbot_algorithm::SolveMode::COLD_START);
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

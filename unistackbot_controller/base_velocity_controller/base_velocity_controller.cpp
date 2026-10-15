#include "base_velocity_controller/base_velocity_controller.hpp"

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "ulog/ulog.hpp"

namespace
{

constexpr char kPluginName[] = "base_velocity_controller";

}  // namespace

namespace unistackbot_controller
{

controller_interface::CallbackReturn BaseVelocityController::on_init()
{
	auto_declare<std::string>("base_kinematics", "");        // 运动学插件: 必填无默认 (算法选择显式手动)
	auto_declare<std::vector<std::string>>("kinematics_joints", std::vector<std::string>{});
	auto_declare<std::vector<double>>("kinematics_params", std::vector<double>{});
	auto_declare<double>("cmd_timeout", 0.5);                // 断流→零速阈值 (底盘安全语义)
	auto_declare<double>("max_lin_x", 0.0);                  // 机型资产 (必填>0)
	auto_declare<double>("max_lin_y", 0.0);
	auto_declare<double>("max_ang", 0.0);
	auto_declare<double>("max_lin_accel", 0.0);
	auto_declare<double>("max_lin_jerk", 0.0);
	auto_declare<double>("max_ang_accel", 0.0);
	auto_declare<double>("max_ang_jerk", 0.0);
	auto_declare<std::string>("odom_frame_id", "odom");
	auto_declare<std::string>("base_frame_id", "base_link");
	auto_declare<double>("publish_hz", 50.0);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseVelocityController::on_configure(const rclcpp_lifecycle::State &)
{
	unistackbot_common::ulog_config log_cfg;
	log_cfg.console = true;
	log_cfg.file_path = nullptr;
	log_cfg.level = unistackbot_common::ulog_level::info;
	log_cfg.max_bytes = 0;
	log_cfg.backups = 0;
	unistackbot_common::ulog_init(log_cfg);

	// ---- 运动学插件装配 (必填无默认 fail-fast 列可选项) ----
	const std::string kin_name = get_node()->get_parameter("base_kinematics").as_string();
	if (kin_name.empty())
	{
		std::string avail;
		for (const auto & n : unistackbot_algorithm::availableBaseKinematics()) { avail += n + " "; }
		ULOG_ERROR("%s: 缺 base_kinematics 显式配置 (可选: %s)", kPluginName, avail.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	kin_ = unistackbot_algorithm::createBaseKinematics(kin_name);
	if (kin_ == nullptr)
	{
		std::string avail;
		for (const auto & n : unistackbot_algorithm::availableBaseKinematics()) { avail += n + " "; }
		ULOG_ERROR("%s: 未知 base_kinematics '%s' (可选: %s)", kPluginName, kin_name.c_str(), avail.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	unistackbot_algorithm::BaseKinConfig cfg;
	cfg.joints = get_node()->get_parameter("kinematics_joints").as_string_array();
	cfg.params = get_node()->get_parameter("kinematics_params").as_double_array();
	std::string err;
	if (!kin_->init(cfg, err))
	{
		ULOG_ERROR("%s: %s", kPluginName, err.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	specs_ = kin_->specs();
	holonomic_ = kin_->holonomic();
	cmd_values_.assign(specs_.size(), 0.0);
	feedback_.assign(specs_.size(), 0.0);

	// ---- 机型资产校验 (限幅界必填正有限; VelocityStream 双保险的第二层) ----
	max_lin_x_ = get_node()->get_parameter("max_lin_x").as_double();
	max_lin_y_ = get_node()->get_parameter("max_lin_y").as_double();
	max_ang_ = get_node()->get_parameter("max_ang").as_double();
	max_lin_accel_ = get_node()->get_parameter("max_lin_accel").as_double();
	max_lin_jerk_ = get_node()->get_parameter("max_lin_jerk").as_double();
	max_ang_accel_ = get_node()->get_parameter("max_ang_accel").as_double();
	max_ang_jerk_ = get_node()->get_parameter("max_ang_jerk").as_double();
	cmd_timeout_ = get_node()->get_parameter("cmd_timeout").as_double();
	if (!(max_lin_x_ > 0.0) || !(max_lin_y_ > 0.0) || !(max_ang_ > 0.0) || !(max_lin_accel_ > 0.0) || !(max_lin_jerk_ > 0.0) || !(max_ang_accel_ > 0.0) || !(max_ang_jerk_ > 0.0) ||
		!(cmd_timeout_ > 0.0))
	{
		ULOG_ERROR("%s: 限幅/超时参数必须全为正 (max_lin_x/y, max_ang, *_accel, *_jerk, cmd_timeout)", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}

	// ---- 契约通道: /cmd_vel (reliable+KeepLast(1) 点流, 与 JS/CM 同款裁决) ----
	cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
		[this](const geometry_msgs::msg::Twist::SharedPtr m)
		{
			TwistSnapshot snap{};
			snap.vx = m->linear.x;
			snap.vy = m->linear.y;
			snap.wz = m->angular.z;
			rt_cmd_.publish(snap);
		});

	// ---- 发布面: /odom + /tf (预分配复用载体, RT 侧只写数值) ----
	const std::string odom_frame = get_node()->get_parameter("odom_frame_id").as_string();
	const std::string base_frame = get_node()->get_parameter("base_frame_id").as_string();
	publish_hz_ = get_node()->get_parameter("publish_hz").as_double();
	auto odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(50));
	rt_odom_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub);
	odom_msg_.header.frame_id = odom_frame;
	odom_msg_.child_frame_id = base_frame;
	auto tf_pub = get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(50));
	rt_tf_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub);
	tf_msg_.transforms.resize(1);
	tf_msg_.transforms[0].header.frame_id = odom_frame;
	tf_msg_.transforms[0].child_frame_id = base_frame;

	shaper_ready_ = false;
	period_n_ = 0;
	ULOG_INFO("%s: %s 运动学, %zu 通道, 限幅 lin=%.2f/%.2f m/s ang=%.2f rad/s, 断流 %.2fs→零速", kPluginName, kin_->name(), specs_.size(), max_lin_x_, max_lin_y_, max_ang_, cmd_timeout_);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BaseVelocityController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	for (const auto & s : specs_)
	{
		conf.names.push_back(s.joint + (s.type == unistackbot_algorithm::BaseActuatorType::Position ? "/position" : "/velocity"));
	}
	return conf;
}

controller_interface::InterfaceConfiguration BaseVelocityController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	for (const auto & s : specs_)
	{
		conf.names.push_back(s.joint + (s.type == unistackbot_algorithm::BaseActuatorType::Position ? "/position" : "/velocity"));
	}
	return conf;
}

controller_interface::CallbackReturn BaseVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
	const std::size_t n = specs_.size();
	if (state_interfaces_.size() != n || command_interfaces_.size() != n)
	{
		ULOG_ERROR("%s: 接口数不匹配 (state=%zu cmd=%zu 通道=%zu)", kPluginName, state_interfaces_.size(), command_interfaces_.size(), n);
		return controller_interface::CallbackReturn::ERROR;
	}
	// 命令 = 当前实测 (激活零跳变, JS 同款); 无命令流时超时机制自然减速到零
	for (std::size_t i = 0; i < n; ++i)
	{
		const double cur = state_interfaces_[i].get_value();
		command_interfaces_[i].set_value(std::isfinite(cur) ? cur : 0.0);
	}
	shaper_ready_ = false;   // 重激活重新校准周期 (JS 同款 16 拍中位数)
	period_n_ = 0;
	ULOG_INFO("%s: 激活 (命令=当前实测, 无流时 %.2fs 后减速停车)", kPluginName, cmd_timeout_);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BaseVelocityController::on_cleanup(const rclcpp_lifecycle::State &)
{
	cmd_sub_.reset();
	rt_odom_.reset();
	rt_tf_.reset();
	kin_.reset();
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BaseVelocityController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
	const double dt = period.seconds();

	// ---- 周期校准 (16 拍中位数; 校准前输出零命令 —— 安全静止) ----
	if (!shaper_ready_)
	{
		if (period_n_ < kPeriodSamples)
		{
			if (dt > 0.0 && dt < 1.0) { period_samples_[period_n_++] = dt; }
			if (period_n_ == kPeriodSamples)
			{
				std::sort(period_samples_, period_samples_ + kPeriodSamples);
				const double calib_dt = period_samples_[kPeriodSamples / 2];
				calib_dt_ = calib_dt;
				unistackbot_common::VelocityStream<3>::Limits lim;
				lim.max_velocity = {max_lin_x_, max_lin_y_, max_ang_};
				lim.max_acceleration = {max_lin_accel_, max_lin_accel_, max_ang_accel_};
				lim.max_jerk = {max_lin_jerk_, max_lin_jerk_, max_ang_jerk_};
				if (shaper_.init(calib_dt, lim))
				{
					std::array<double, 3> zero{};
					shaper_.reset(zero);
					shaper_ready_ = true;
					pub_div_ = static_cast<uint32_t>(std::max(1.0, 1.0 / (calib_dt * publish_hz_)));
					ULOG_INFO("%s: 速率校准 %.0fHz (dt=%.4fs), odom/TF @%.0fHz", kPluginName, 1.0 / calib_dt, calib_dt, 1.0 / (calib_dt * pub_div_));
				}
			}
		}
		if (!shaper_ready_)
		{
			return controller_interface::return_type::OK;
		}
	}

	// ---- 判活: seq 轮询 (断流>timeout → 零速, 底盘安全语义与臂域相反; 时钟域无关) ----
	const uint64_t seq_now = rt_cmd_.seq();
	if (seq_now != last_seq_)
	{
		last_seq_ = seq_now;
		age_periods_ = 0;
	}
	else if (age_periods_ < kMaxAge)
	{
		++age_periods_;
	}
	std::array<double, 3> target = last_target_;
	if (age_periods_ * calib_dt_ <= cmd_timeout_)
	{
		TwistSnapshot snap;
		uint64_t snap_seq = 0;
		if (rt_cmd_.read(snap, snap_seq) && std::isfinite(snap.vx) && std::isfinite(snap.vy) && std::isfinite(snap.wz))
		{
			if (!holonomic_ && std::fabs(snap.vy) > 1e-9) { ++dropped_vy_; }   // 非全向: 横移指令诚实钳零
			target = {std::clamp(snap.vx, -max_lin_x_, max_lin_x_), holonomic_ ? std::clamp(snap.vy, -max_lin_y_, max_lin_y_) : 0.0, std::clamp(snap.wz, -max_ang_, max_ang_)};
			last_target_ = target;
		}
	}
	else
	{
		target = {0.0, 0.0, 0.0};   // 断流: 目标零速 (VelocityStream 整形减速)
		last_target_ = target;
	}

	// ---- 整形 → 逆解 → 写命令 ----
	std::array<double, 3> shaped{0.0, 0.0, 0.0};
	if (shaper_.update(target, shaped) != unistackbot_common::VelocityStream<3>::UpdateResult::Ok)
	{
		shaped = {0.0, 0.0, 0.0};   // Hold 防御: VelocityStream 契约输出恒有效, 此处零速兜底 (宁停勿跑)
	}
	const unistackbot_algorithm::BodyTwist cmd_twist{shaped[0], shaped[1], shaped[2]};
	kin_->toActuators(cmd_twist, cmd_values_);
	for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
	{
		command_interfaces_[i].set_value(cmd_values_[i]);
	}

	// ---- 正解 → odom 积分 (裁决①: 控制器积分 = 轮里程计语义) ----
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		const double v = state_interfaces_[i].get_value();
		feedback_[i] = std::isfinite(v) ? v : 0.0;
	}
	unistackbot_algorithm::BodyTwist meas;
	kin_->fromActuators(feedback_, meas);
	const double cos_y = std::cos(odom_yaw_), sin_y = std::sin(odom_yaw_);
	odom_x_ += (meas.vx * cos_y - meas.vy * sin_y) * dt;
	odom_y_ += (meas.vx * sin_y + meas.vy * cos_y) * dt;
	odom_yaw_ += meas.wz * dt;
	if (odom_yaw_ > M_PI) { odom_yaw_ -= 2.0 * M_PI; }
	else if (odom_yaw_ < -M_PI) { odom_yaw_ += 2.0 * M_PI; }

	// ---- 发布 @分频 (trylock 非阻塞, 丢拍可接受) ----
	if (++pub_count_ % pub_div_ == 0)
	{
		if (rt_odom_ != nullptr && rt_odom_->trylock())
		{
			auto & m = rt_odom_->msg_;
			m = odom_msg_;   // 复用预分配载体 (frame ids 等静态字段回填)
			m.header.stamp = time;
			m.pose.pose.position.x = odom_x_;
			m.pose.pose.position.y = odom_y_;
			m.pose.pose.position.z = 0.0;
			m.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);
			m.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
			m.twist.twist.linear.x = meas.vx;
			m.twist.twist.linear.y = meas.vy;
			m.twist.twist.angular.z = meas.wz;
			rt_odom_->unlockAndPublish();
		}
		if (rt_tf_ != nullptr && rt_tf_->trylock())
		{
			auto & t = rt_tf_->msg_;
			t = tf_msg_;
			t.transforms[0].header.stamp = time;
			t.transforms[0].transform.translation.x = odom_x_;
			t.transforms[0].transform.translation.y = odom_y_;
			t.transforms[0].transform.translation.z = 0.0;
			t.transforms[0].transform.rotation.z = std::sin(odom_yaw_ * 0.5);
			t.transforms[0].transform.rotation.w = std::cos(odom_yaw_ * 0.5);
			rt_tf_->unlockAndPublish();
		}
	}
	return controller_interface::return_type::OK;
}

}  // namespace unistackbot_controller

PLUGINLIB_EXPORT_CLASS(unistackbot_controller::BaseVelocityController, controller_interface::ControllerInterface)

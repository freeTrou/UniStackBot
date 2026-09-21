#include "ee_state_broadcaster/ee_state_broadcaster.hpp"

#include <algorithm>
#include <cstdio>

#include "rclcpp/logging.hpp"
#include "ulog/ulog.hpp"

namespace unistackbot_controller
{

static constexpr char kPluginName[] = "ee_state_broadcaster";

controller_interface::CallbackReturn EeStateBroadcaster::on_init()
{
	auto_declare<std::string>("base_link", "");
	auto_declare<std::string>("tip_link", "");
	auto_declare<double>("publish_hz", publish_hz_);
	// Humble: controller_manager 把自身 robot_description 以参数覆盖注入控制器节点
	auto_declare<std::string>("robot_description", "");
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EeStateBroadcaster::on_configure(
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
	publish_hz_ = get_node()->get_parameter("publish_hz").as_double();
	if (base_link_.empty() || tip_link_.empty())
	{
		ULOG_ERROR("%s: base_link/tip_link 未配置 (yaml 需给)", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	if (publish_hz_ <= 0.0)
	{
		ULOG_ERROR("%s: publish_hz 非法 (%.1f)", kPluginName, publish_hz_);
		return controller_interface::CallbackReturn::ERROR;
	}

	std::string urdf;
	if (!get_node()->get_parameter("robot_description", urdf) || urdf.empty())
	{
		ULOG_ERROR("%s: robot_description 未注入", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	auto fk = std::make_unique<UrdfFk>();
	std::string msg;
	if (!fk->init(urdf, base_link_, tip_link_, msg))
	{
		ULOG_ERROR("%s: FK init 失败: %s", kPluginName, msg.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	fk_ = std::move(fk);

	pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
		get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
			"~/ee_state", rclcpp::QoS(10)));   // 状态流: 默认 reliable 即可 (50Hz 小包)
	ULOG_INFO("%s: 配置完成 (%s -> %s @%.0fHz)", kPluginName,
		base_link_.c_str(), tip_link_.c_str(), publish_hz_);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EeStateBroadcaster::command_interface_configuration()
const
{
	// 只读控制器: 不认领任何命令接口 (与 JS/CM 自由共存)
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::NONE;
	return conf;
}

controller_interface::InterfaceConfiguration EeStateBroadcaster::state_interface_configuration()
const
{
	// 认领 FK 链关节的 position 状态接口 (与 JSB 同款共享语义)
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

controller_interface::CallbackReturn EeStateBroadcaster::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	const std::size_t n = fk_ ? fk_->jointCount() : 0u;
	if (state_interfaces_.size() != n)
	{
		ULOG_ERROR("%s: 接口数不匹配 (state=%zu 链=%zu)",
			kPluginName, state_interfaces_.size(), n);
		return controller_interface::CallbackReturn::ERROR;
	}
	// 按名映射: 接口声明序 != FK 链序 (真雷, 与 CM 同款处理)
	const auto & chain_names = fk_->jointNames();
	chain_from_iface_.clear();
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		const std::string jn = state_interfaces_[i].get_prefix_name();
		const auto it = std::find(chain_names.begin(), chain_names.end(), jn);
		if (it == chain_names.end())
		{
			ULOG_ERROR("%s: 接口 %s 不在 FK 链上", kPluginName, jn.c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
		chain_from_iface_.push_back(static_cast<std::size_t>(it - chain_names.begin()));
	}
	last_pub_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
	ULOG_INFO("%s: 激活 (%zu 关节 FK)", kPluginName, n);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EeStateBroadcaster::update(
	const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
	// 50Hz 时间节流 (RT trylock 非阻塞; 拿不到锁跳过本拍 —— 下一拍再来)
	if ((time - last_pub_).seconds() < 1.0 / publish_hz_ || !pub_)
	{
		return controller_interface::return_type::OK;
	}
	last_pub_ = time;

	std::vector<double> q(fk_->jointCount(), 0.0);
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		q[chain_from_iface_[i]] = state_interfaces_[i].get_value();
	}
	unistackbot_algorithm::CartesianPose pose;
	if (!fk_->fk(q, pose, fk_scratch_))
	{
		return controller_interface::return_type::OK;   // FK 异常: 跳过本拍 (下拍重试)
	}

	if (pub_->trylock())
	{
		auto & m = pub_->msg_;
		m.header.stamp = time;
		m.header.frame_id = base_link_;
		m.pose.position.x = pose.x;
		m.pose.position.y = pose.y;
		m.pose.position.z = pose.z;
		m.pose.orientation.w = pose.qw;
		m.pose.orientation.x = pose.qx;
		m.pose.orientation.y = pose.qy;
		m.pose.orientation.z = pose.qz;
		pub_->unlockAndPublish();
	}
	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn EeStateBroadcaster::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	ULOG_INFO("%s: 停用", kPluginName);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EeStateBroadcaster::on_cleanup(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	pub_.reset();
	fk_.reset();
	return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unistackbot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unistackbot_controller::EeStateBroadcaster,
	controller_interface::ControllerInterface)

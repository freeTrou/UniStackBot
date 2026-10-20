#include "uni_joint_state_broadcaster/uni_joint_state_broadcaster.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "controller_common/urdf_command_joints.hpp"
#include "ulog/ulog.hpp"

namespace unistackbot_controller
{

static constexpr char kPluginName[] = "uni_joint_state_broadcaster";

namespace
{
constexpr std::array<const char *, UniJointStateBroadcaster::kIfaceCount> kIfaceNames{"position", "velocity", "effort"};

/*
 * 解析 URDF <ros2_control> 块: state 关节全集 (保序) + "name/iface" 认领清单。
 * 遍历骨架 = controller_common 的 forEachRos2ControlJoint (2026-10-20 单源化) ——
 * 与 command 侧 parseJointsFromUrdf 共享同一遍历, 排序契约由单源保证。
 * 不设 kMaxJoints 上界: 本表变长 (官方 JSB 同样无界, r1_pro 18 关节 + 轮系均容纳;
 * kMaxJoints 是命令关节定长数组的界, 不适用)。
 */
[[nodiscard]] bool parseStateJointsFromUrdf(const std::string & urdf, std::vector<std::string> & names,
	std::vector<std::string> & claims, std::string & err)
{
	const bool ok = forEachRos2ControlJoint(urdf, err,
		[&](const tinyxml2::XMLElement * j, const char * jn)
		{
			std::array<bool, kIfaceNames.size()> h{};
			for (const tinyxml2::XMLElement * si = j->FirstChildElement("state_interface"); si != nullptr;
				si = si->NextSiblingElement("state_interface"))
			{
				const char * nm = si->Attribute("name");
				if (nm == nullptr)
				{
					continue;
				}
				for (std::size_t k = 0; k < kIfaceNames.size(); ++k)
				{
					if (std::strcmp(nm, kIfaceNames[k]) == 0)
					{
						h[k] = true;
					}
				}
			}
			bool any = false;
			for (std::size_t k = 0; k < kIfaceNames.size(); ++k)
			{
				any = any || h[k];
			}
			if (!any)
			{
				return true;   // 无 state 接口的关节不广播
			}
			if (std::find(names.begin(), names.end(), jn) != names.end())
			{
				err = std::string("关节重复声明: ") + jn;   // 序契约不容歧义
				return false;
			}
			names.push_back(jn);
			for (std::size_t k = 0; k < kIfaceNames.size(); ++k)
			{
				if (h[k])
				{
					claims.push_back(std::string(jn) + "/" + kIfaceNames[k]);
				}
			}
			return true;
		});
	if (!ok)
	{
		return false;
	}
	if (names.empty())
	{
		err = "URDF <ros2_control> 无 state 接口关节";
		return false;
	}
	return true;
}

}  // namespace

controller_interface::CallbackReturn UniJointStateBroadcaster::on_init()
{
	// publish_hz 不走 auto_declare<double>: yaml 写整数 (publish_hz: 50) 时 override 已按
	// int64 auto-declare, get_value<double> 抛 InvalidParameterTypeException 且 Humble CM
	// 不 catch → CM 进程终止 (2026-10-20 审查 A1 实机复现)。条件声明 + configure 期按
	// 类型分支读取; 未配置时走默认 0.0 (全率)。
	if (!get_node()->has_parameter("publish_hz"))
	{
		get_node()->declare_parameter<double>("publish_hz", publish_hz_);
	}
	// Humble: controller_manager 把自身 robot_description 以参数覆盖注入控制器节点
	auto_declare<std::string>("robot_description", "");
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UniJointStateBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	unistackbot_common::ulog_init_console();

	const auto prm = get_node()->get_parameter("publish_hz");
	if (prm.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
	{
		publish_hz_ = static_cast<double>(prm.as_int());   // yaml 整数容错 (A1)
	}
	else if (prm.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
	{
		publish_hz_ = prm.as_double();
	}
	else
	{
		ULOG_ERROR("%s: publish_hz 类型非法 (需数值)", kPluginName);
		return controller_interface::CallbackReturn::ERROR;
	}
	if (publish_hz_ < 0.0)
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
	names_.clear();
	state_claims_.clear();
	std::string msg;
	if (!parseStateJointsFromUrdf(urdf, names_, state_claims_, msg))
	{
		ULOG_ERROR("%s: URDF 解析失败: %s", kPluginName, msg.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}

	// 顶层绝对名 (官方 JSB 同款, EEB /ee_state 同款注释: 框架认定的标准位优先于节点
	// 命名空间; "~" 会展开成 /joint_state_broadcaster/joint_states —— 2026-10-20 实测)
	pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
		get_node()->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10)));
	// 消息骨架预填 (RT 环零分配): name 序 = 输出契约, 数组定长
	auto & m = pub_->msg_;
	m.name = names_;
	m.position.assign(names_.size(), 0.0);
	m.velocity.assign(names_.size(), 0.0);
	m.effort.assign(names_.size(), 0.0);
	ULOG_INFO("%s: 配置完成 (%zu 关节 URDF 序, %s)", kPluginName, names_.size(),
		publish_hz_ > 0.0 ? "限频发布" : "全率发布");
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration UniJointStateBroadcaster::command_interface_configuration()
const
{
	// 只读控制器: 不认领任何命令接口 (与 JS/CM/EEB 自由共存)
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::NONE;
	return conf;
}

controller_interface::InterfaceConfiguration UniJointStateBroadcaster::state_interface_configuration()
const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	conf.names = state_claims_;   // on_configure 单源清单, 认领即清单 (无二次推导)
	return conf;
}

controller_interface::CallbackReturn UniJointStateBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	if (state_interfaces_.size() != state_claims_.size())
	{
		ULOG_ERROR("%s: 接口数不匹配 (state=%zu 期望=%zu)", kPluginName, state_interfaces_.size(), state_claims_.size());
		return controller_interface::CallbackReturn::ERROR;
	}
	// 按名映射: 接口到手序不可信 (ResourceManager 无序容器), 输出序只认 URDF 契约
	chan_.assign(names_.size(), Channels{});
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		const std::string jn = state_interfaces_[i].get_prefix_name();
		const auto it = std::find(names_.begin(), names_.end(), jn);
		if (it == names_.end())
		{
			ULOG_ERROR("%s: 接口 %s 不在 URDF state 关节表", kPluginName, jn.c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
		const std::string in = state_interfaces_[i].get_interface_name();
		std::size_t k = 0;
		while (k < kIfaceNames.size() && in != kIfaceNames[k])
		{
			++k;
		}
		if (k == kIfaceNames.size())
		{
			ULOG_ERROR("%s: 未知接口类型 %s/%s", kPluginName, jn.c_str(), in.c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
		Channels & c = chan_[it - names_.begin()];
		if (c.idx[k] != -1)
		{
			ULOG_ERROR("%s: 接口重复 %s/%s", kPluginName, jn.c_str(), in.c_str());
			return controller_interface::CallbackReturn::ERROR;
		}
		c.idx[k] = static_cast<int>(i);
	}
	last_pub_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
	ULOG_INFO("%s: 激活 (%zu 关节, %zu 接口)", kPluginName, names_.size(), state_interfaces_.size());
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UniJointStateBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
	if (!pub_)
	{
		return controller_interface::return_type::OK;
	}
	if (time < last_pub_)
	{
		// 仿真时间回卷 (mujoco reset 含时间回卷, controller 保持 active): 重锚防节流门
		// 永久关闭 (last_pub_ 冻结在回卷前, elapsed 恒负 → 静默整段回卷前时长)
		last_pub_ = time;
	}
	if (publish_hz_ > 0.0 && (time - last_pub_).seconds() < 1.0 / publish_hz_)
	{
		return controller_interface::return_type::OK;
	}
	last_pub_ = time;
	if (pub_->trylock())   // RT 非阻塞; 拿不到锁跳本拍
	{
		auto & m = pub_->msg_;
		m.header.stamp = time;
		const std::array<std::vector<double> *, kIfaceCount> dst{&m.position, &m.velocity, &m.effort};
		for (std::size_t i = 0; i < chan_.size(); ++i)
		{
			for (std::size_t k = 0; k < kIfaceCount; ++k)
			{
				const int idx = chan_[i].idx[k];
				(*dst[k])[i] = idx >= 0 ? state_interfaces_[static_cast<std::size_t>(idx)].get_value() : 0.0;
			}
		}
		pub_->unlockAndPublish();
	}
	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn UniJointStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	ULOG_INFO("%s: 停用", kPluginName);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UniJointStateBroadcaster::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
	pub_.reset();
	names_.clear();
	state_claims_.clear();
	chan_.clear();
	return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unistackbot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unistackbot_controller::UniJointStateBroadcaster, controller_interface::ControllerInterface)

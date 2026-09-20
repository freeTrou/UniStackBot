#include "joint_stream_controller/joint_stream_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <tinyxml2.h>

#include <pluginlib/class_list_macros.hpp>

#include "ulog/ulog.hpp"

namespace
{

constexpr char kPluginName[] = "joint_stream_controller";

// 解析 URDF <ros2_control> 块: position 命令关节表 + min/max/max_velocity。
// 单一事实源: 限位与 BackendKinematic / CM 控制器同参数同缺省。
bool parseJointsFromUrdf(
	const std::string & urdf, std::vector<std::string> & names,
	std::vector<double> & qmin, std::vector<double> & qmax,
	std::vector<double> & vmax, std::string & err)
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
			double mn = 0.0, mx = 0.0, vm = 5.0;
			for (const tinyxml2::XMLElement * prm = j->FirstChildElement("param"); prm != nullptr;
				prm = prm->NextSiblingElement("param"))
			{
				const char * pn = prm->Attribute("name");
				const char * txt = prm->GetText();
				if (pn == nullptr || txt == nullptr)
				{
					continue;
				}
				if (std::strcmp(pn, "min") == 0) {mn = std::atof(txt);}
				else if (std::strcmp(pn, "max") == 0) {mx = std::atof(txt);}
				else if (std::strcmp(pn, "max_velocity") == 0) {vm = std::atof(txt);}
			}
			for (const tinyxml2::XMLElement * ci = j->FirstChildElement("command_interface"); ci != nullptr;
				ci = ci->NextSiblingElement("command_interface"))
			{
				const char * nm = ci->Attribute("name");
				if (nm != nullptr && std::strcmp(nm, "position") == 0)
				{
					has_cmd = true;
					break;
				}
			}
			if (!has_cmd)
			{
				continue;   // mimic / 无 position 命令的关节不归本控制器
			}
			names.push_back(jn);
			qmin.push_back(mn);
			qmax.push_back(mx);
			vmax.push_back(vm > 0.0 ? vm : 5.0);
		}
	}
	if (names.empty())
	{
		err = "URDF <ros2_control> 无 position 命令关节";
		return false;
	}
	return true;
}

}  // namespace

namespace unistackbot_controller
{

controller_interface::CallbackReturn JointStreamController::on_init()
{
	auto_declare<std::string>("interpolation", "hold");   // hold | ruckig
	auto_declare<double>("max_acceleration", max_acceleration_);
	auto_declare<double>("max_jerk", max_jerk_);
	auto_declare<std::string>("robot_description", "");
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStreamController::on_configure(
	const rclcpp_lifecycle::State & /*previous_state*/)
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
	if (!parseJointsFromUrdf(urdf, joint_names_, q_min_, q_max_, vmax_, err))
	{
		ULOG_ERROR("%s: %s", kPluginName, err.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}
	const double hz = std::max<double>(static_cast<double>(get_update_rate()), 1.0);
	step_limits_.clear();
	for (double v : vmax_)
	{
		step_limits_.push_back(v / hz);   // 单拍步长 = max_velocity / update_rate
	}
	cmd_.assign(joint_names_.size(), 0.0);
	target_.assign(joint_names_.size(), 0.0);

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
	cmd_sub_ = get_node()->create_subscription<unistackbot_interface::msg::JointCommand>(
		"~/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
		[this](const unistackbot_interface::msg::JointCommand::SharedPtr m)
		{
			if (m->mode != unistackbot_interface::msg::JointCommand::MODE_CSP)
			{
				++dropped_mode_;   // 仿真链仅 CSP; 其余模式计数 (真机 MIT 批次启用)
				return;
			}
			if (m->joint_names.size() != joint_names_.size() ||
				m->position.size() < joint_names_.size())
			{
				++dropped_len_;
				return;
			}
			// 按名映射: 消息关节序 → 本控制器关节序 (缺关节整条丢弃), 产出 POD 快照
			CmdSnapshot snap{};
			snap.seq = ++cmd_seq_pub_;
			for (std::size_t i = 0; i < joint_names_.size(); ++i)
			{
				const auto it = std::find(m->joint_names.begin(), m->joint_names.end(),
					joint_names_[i]);
				if (it == m->joint_names.end())
				{
					++dropped_len_;
					return;
				}
				snap.position[i] = m->position[static_cast<std::size_t>(it - m->joint_names.begin())];
			}
			rt_cmd_.publish(snap);
		});

	ULOG_INFO("%s: %zu 关节, 插值=%s, 步长上限=%.3f rad/拍",
		kPluginName, joint_names_.size(),
		interpolation_ == Interpolation::RUCKIG ? "ruckig" : "hold",
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

controller_interface::CallbackReturn JointStreamController::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	const std::size_t n = joint_names_.size();
	if (state_interfaces_.size() != n || command_interfaces_.size() != n)
	{
		ULOG_ERROR("%s: 接口数不匹配 (state=%zu cmd=%zu 关节=%zu)",
			kPluginName, state_interfaces_.size(), command_interfaces_.size(), n);
		return controller_interface::CallbackReturn::ERROR;
	}
	// 命令 = 当前状态 (激活零跳变)
	for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
	{
		cmd_[i] = state_interfaces_[i].get_value();
	}
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

controller_interface::return_type JointStreamController::update(
	const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	const std::size_t n = joint_names_.size();

	// ① 取最新命令 (值通道; 撕裂/无发布 → 沿用旧值 = sp_latest 契约)
	CmdSnapshot snap;
	uint64_t snap_seq = 0;
	if (rt_cmd_.read(snap, snap_seq) && snap_seq != cmd_seq_)
	{
		cmd_seq_ = snap_seq;
		std::vector<double> q(snap.position, snap.position + n);
		if (sanitize(q))
		{
			if (interpolation_ == Interpolation::HOLD)
			{
				// hold 档: 步长饱和 (速度界; 上层应为平滑流)
				for (std::size_t i = 0; i < n; ++i)
				{
					const double dv = q[i] - cmd_[i];
					if (dv > step_limits_[i])
					{
						q[i] = cmd_[i] + step_limits_[i];
					}
					else if (dv < -step_limits_[i])
					{
						q[i] = cmd_[i] - step_limits_[i];
					}
				}
				cmd_ = q;
			}
			else
			{
				// ruckig 档: 目标直达 (整形由 OtgStream 的 v/a/j 极限承担, 无步长饱和)
				target_ = q;
			}
		}
		else
		{
			++dropped_mode_;   // NaN 命令计数 (复用观测通道)
		}
	}

	// ③ ruckig 档: 每拍从当前态向最新目标生成参考 (率失配填充 + v/a/j 整形)。
	//    Ok → cmd_ 前进; Hold → OtgStream 内部已自愈, cmd_ 保持上一拍输出
	if (interpolation_ == Interpolation::RUCKIG && otg_ready_)
	{
		std::array<double, unistackbot_interface::kMaxJoints> tgt{};
		std::array<double, unistackbot_interface::kMaxJoints> out{};
		for (std::size_t i = 0; i < n; ++i) {tgt[i] = target_[i];}
		if (otg_.update(tgt, out) == unistackbot_common::OtgStream<unistackbot_interface::kMaxJoints>::UpdateResult::Ok)
		{
			for (std::size_t i = 0; i < n; ++i) {cmd_[i] = out[i];}
		}
	}

	// ④ 写命令
	for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
	{
		command_interfaces_[i].set_value(cmd_[i]);
	}
	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn JointStreamController::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	ULOG_INFO("%s: 停用 (接口释放, 硬件保持)", kPluginName);
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStreamController::on_cleanup(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	cmd_sub_.reset();
	return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace unistackbot_controller

// 插件注册 (缺失 = "no factory exists for it" —— 类加载器找不到工厂)
PLUGINLIB_EXPORT_CLASS(unistackbot_controller::JointStreamController,
	controller_interface::ControllerInterface)

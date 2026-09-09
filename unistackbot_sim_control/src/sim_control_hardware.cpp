#include "unistackbot_sim_control/sim_control_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ulog/ulog.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistackbot_sim_control/sim_backend_factory.hpp"

namespace unistackbot_sim_control
{


namespace
{

constexpr double kDefaultMaxVelocity = 5.0;   // 未声明 max_velocity 时的最终执行器速度上限 (关节上限见 sim_command_queue.hpp 的 kMaxJoints)

// 边界兜底: std::stod 会抛 (invalid_argument/out_of_range), 在此接住并翻译成返回值
// —— 我们的代码不抛, 只接系统明确会抛的 (规范 27)
[[nodiscard]] bool toDouble(const std::string & text, double & out)
{
	try
	{
		size_t consumed = 0;
		out = std::stod(text, &consumed);
		return consumed == text.size();   // "1.5abc" 只消费一半, 判为非法
	}
	catch (const std::exception &)
	{
		return false;
	}
}

// 装配期转储: 全量打印 CM 解析进来的 HardwareInfo (仅 on_init 调用, 非 RT 路径, 允许分配)
void dumpHardwareInfo(const hardware_interface::HardwareInfo & info)
{
	ULOG_INFO("===== HardwareInfo dump: name=%s type=%s =====", info.name.c_str(), info.type.c_str());
	for (const auto & param : info.hardware_parameters)
	{
		ULOG_INFO("[hardware] %s = %s", param.first.c_str(), param.second.c_str());
	}
	ULOG_INFO("[counts] joints=%zu, sensors=%zu, gpios=%zu", info.joints.size(), info.sensors.size(), info.gpios.size());
	for (const auto & joint : info.joints)
	{
		std::string cmd_names;
		for (const auto & ci : joint.command_interfaces)
		{
			cmd_names += ci.name + ", ";
		}
		std::string state_names;
		for (const auto & si : joint.state_interfaces)
		{
			state_names += si.name + ", ";
		}
		ULOG_INFO("[joint] %s | cmd: %s| state: %s", joint.name.c_str(), cmd_names.c_str(), state_names.c_str());
		for (const auto & param : joint.parameters)
		{
			ULOG_INFO("[joint %s] %s = %s", joint.name.c_str(), param.first.c_str(), param.second.c_str());
		}
	}
	ULOG_INFO("===== HardwareInfo dump end =====");
}

}  // namespace

hardware_interface::CallbackReturn SimControlHardware::on_init(const hardware_interface::HardwareInfo & info)
{
	// 调用框架函数检测
	if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	// ulog 接入: 终端 sink (launch 捕获 stdout); 可选文件 sink 经 <param name="ulog_file">
	unistackbot_common::ulog_config ulog_cfg;
	ulog_cfg.console = true;
	if (info_.hardware_parameters.count("ulog_file"))
	{
		ulog_cfg.file_path = info_.hardware_parameters.at("ulog_file").c_str();
	}
	unistackbot_common::ulog_init(ulog_cfg);

	ULOG_INFO("SimControlHardware on_init: %zu joints", info_.joints.size());
	dumpHardwareInfo(info_); // 打印所有的信息

	const size_t joint_count = info_.joints.size();
	if (joint_count > kMaxJoints) // 检查长度是否小于上限
	{
		ULOG_ERROR("joint_count %zu > kMaxJoints=%u", joint_count, kMaxJoints);
		return hardware_interface::CallbackReturn::ERROR;
	}

	// 提前分配内存 
	joints_.resize(joint_count);
	cmd_position_.assign(joint_count, 0.0);
	state_position_.assign(joint_count, 0.0);
	state_velocity_.assign(joint_count, 0.0);
	state_effort_.assign(joint_count, 0.0);

	// 第一遍: 名字索引 / 限位 / 速度上限 / 接口契约校验
	std::unordered_map<std::string, size_t> joint_index;
	for (size_t i = 0; i < joint_count; ++i)
	{
		const auto & joint = info_.joints[i];
		joint_index[joint.name] = i;
		joints_[i].name = joint.name;

		joints_[i].min = 0.0;
		joints_[i].max = 0.0;
		if (joint.parameters.count("min") && !toDouble(joint.parameters.at("min"), joints_[i].min))
		{
			ULOG_ERROR("Joint '%s': invalid double '%s' for param 'min'", joint.name.c_str(), joint.parameters.at("min").c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (joint.parameters.count("max") && !toDouble(joint.parameters.at("max"), joints_[i].max))
		{
			ULOG_ERROR("Joint '%s': invalid double '%s' for param 'max'", joint.name.c_str(), joint.parameters.at("max").c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		joints_[i].max_velocity = kDefaultMaxVelocity;
		if (joint.parameters.count("max_velocity") && !toDouble(joint.parameters.at("max_velocity"), joints_[i].max_velocity))
		{
			ULOG_ERROR("Joint '%s': invalid double '%s' for param 'max_velocity'", joint.name.c_str(), joint.parameters.at("max_velocity").c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		// 命令接口契约: 恰好一个 position
		if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
		{
			ULOG_ERROR("Joint '%s' must declare exactly one position command interface", joint.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		// 状态接口契约: position 必须有, 其余只能是 velocity/effort (effort 恒 0)
		bool has_position = false;
		for (const auto & si : joint.state_interfaces)
		{
			if (si.name == hardware_interface::HW_IF_POSITION)
			{
				has_position = true;
			}
			else if (si.name != hardware_interface::HW_IF_VELOCITY && si.name != hardware_interface::HW_IF_EFFORT)
			{
				ULOG_ERROR("Joint '%s' declares unsupported state interface '%s'", joint.name.c_str(), si.name.c_str());
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		if (!has_position)
		{
			ULOG_ERROR("Joint '%s' must declare a position state interface", joint.name.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
	}

	// 第二遍: mimic 关系解析
	for (size_t i = 0; i < joint_count; ++i)
	{
		const auto & params = info_.joints[i].parameters;
		auto mimic_it = params.find("mimic");
		if (mimic_it == params.end() || mimic_it->second.empty())
		{
			continue;
		}
		auto src_it = joint_index.find(mimic_it->second);
		if (src_it == joint_index.end())
		{
			ULOG_ERROR("Joint '%s' mimics unknown joint '%s'", info_.joints[i].name.c_str(), mimic_it->second.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		joints_[i].mimic_source = static_cast<int>(src_it->second);
		joints_[i].mimic_multiplier = 1.0;
		joints_[i].mimic_offset = 0.0;
		if (params.count("multiplier") && !toDouble(params.at("multiplier"), joints_[i].mimic_multiplier))
		{
			ULOG_ERROR("Joint '%s': invalid double '%s' for mimic param 'multiplier'", joints_[i].name.c_str(), params.at("multiplier").c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		if (params.count("offset") && !toDouble(params.at("offset"), joints_[i].mimic_offset))
		{
			ULOG_ERROR("Joint '%s': invalid double '%s' for mimic param 'offset'", joints_[i].name.c_str(), params.at("offset").c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}
		ULOG_INFO("Joint '%s' mimics '%s' (multiplier %.3f, offset %.3f)",
			joints_[i].name.c_str(), mimic_it->second.c_str(),
			joints_[i].mimic_multiplier, joints_[i].mimic_offset);
	}

	// 后端分类 (对内): "backend" 为 <hardware> 级参数, 工厂按名实例化
	std::string backend_name = "kinematic";
	if (info_.hardware_parameters.count("backend"))
	{
		backend_name = info_.hardware_parameters.at("backend");
	}
	std::string message;
	backend_ = createBackend(backend_name, message);
	if (!backend_)
	{
		ULOG_ERROR("%s", message.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	if (!backend_->init(joints_, message))
	{
		ULOG_ERROR("Backend '%s' init failed: %s", backend_name.c_str(), message.c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	ULOG_INFO("Backend '%s' ready (%s)", backend_name.c_str(), message.c_str());

	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SimControlHardware::export_state_interfaces()
{
	// 镜像 URDF 声明: 声明了什么状态接口就导出什么 (effort 恒 0)
	std::vector<hardware_interface::StateInterface> interfaces;
	for (size_t i = 0; i < info_.joints.size(); ++i)
	{
		for (const auto & si : info_.joints[i].state_interfaces)
		{
			if (si.name == hardware_interface::HW_IF_POSITION)
			{
				interfaces.emplace_back(info_.joints[i].name, si.name, &state_position_[i]);
			}
			else if (si.name == hardware_interface::HW_IF_VELOCITY)
			{
				interfaces.emplace_back(info_.joints[i].name, si.name, &state_velocity_[i]);
			}
			else if (si.name == hardware_interface::HW_IF_EFFORT)
			{
				interfaces.emplace_back(info_.joints[i].name, si.name, &state_effort_[i]);
			}
		}
	}
	return interfaces;
}

std::vector<hardware_interface::CommandInterface> SimControlHardware::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> interfaces;
	interfaces.reserve(cmd_position_.size());
	for (size_t i = 0; i < cmd_position_.size(); ++i)
	{
		interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_position_[i]);
	}
	return interfaces;
}

hardware_interface::CallbackReturn SimControlHardware::on_configure(const rclcpp_lifecycle::State &)
{
	// /sim_control 服务: 独立节点 + 独立 executor 线程 (非实时)
	service_node_ = rclcpp::Node::make_shared("sim_control");
	server_ = std::make_unique<SimControlServer>(
		service_node_,
		[this](const SimCommand & cmd, std::string & message)
		{
			return enqueueSink(cmd, message);
		},
		[this](const std::vector<std::string> & names,
			const std::vector<double> & positions,
			const std::vector<double> & velocities,
			SimCommand & command,
			std::string & message)
		{
			return validateSetState(names, positions, velocities, command, message);
		});
	service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	service_executor_->add_node(service_node_);
	service_running_ = true;
	service_thread_ = std::thread([this]()
	{
		while (service_running_.load() && rclcpp::ok())
		{
			service_executor_->spin_once(std::chrono::milliseconds(50));
		}
	});

	ULOG_INFO("/sim_control services started");
	return hardware_interface::CallbackReturn::SUCCESS;
}

// 停收 /sim_control 服务线程 (幂等; on_cleanup 与 on_shutdown 共用)。
// 踩坑记录: 进程退出走 deactivate → shutdown 直接进 FINALIZED, 不经过 on_cleanup ——
// 清理只写在 on_cleanup 时 service_thread_ 无人 join, joinable 线程析构 → std::terminate → abort
void SimControlHardware::stopServices()
{
	service_running_.store(false);
	if (service_thread_.joinable())
	{
		service_thread_.join();
	}
	if (service_executor_ && service_node_)
	{
		service_executor_->remove_node(service_node_);
	}
	server_.reset();
	service_node_.reset();
}

hardware_interface::CallbackReturn SimControlHardware::on_activate(const rclcpp_lifecycle::State &)
{
	// 后端生命周期转发: 异步后端在此起 plant, 失败则不开 RT 循环
	if (!activateBackend())
	{
		ULOG_ERROR("Backend '%s' failed to activate", backend_->name().c_str());
		return hardware_interface::CallbackReturn::ERROR;
	}
	// 默认语义: mock 无需首拍同步 (cmd/state 同从 0 起); 真机驱动重写此处时先 read 再 cmd=state
	ULOG_INFO("SimControlHardware activated (INACTIVE -> ACTIVE), RT loop starting");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimControlHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
	// 后端生命周期转发: 异步后端在此停 plant (有界 join)
	deactivateBackend();
	// 默认语义: 停用后接口收回、RT 循环停止, 命令数组保持最后值 (无人再消化)
	ULOG_INFO("SimControlHardware deactivated (ACTIVE -> INACTIVE), RT loop stopped");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimControlHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
	stopServices();
	ULOG_INFO("/sim_control services stopped (cleanup)");
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SimControlHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
	stopServices();
	ULOG_INFO("/sim_control services stopped (shutdown)");
	return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- 后端访问收口: backend_ 只在 on_init(创建处) 与以下收口方法中出现 ----

//起 plant (异步后端在此起线程; 失败则激活失败, RT 循环不启动)
bool SimControlHardware::activateBackend()
{
	return backend_->activate();
}

//停 plant (异步后端在此停线程, 有界 join)
void SimControlHardware::deactivateBackend()
{
	backend_->deactivate();
}

//后端一拍驱动 (read() 内, 全类唯一 step 调用点); dt=0 且 integrate=false = 只重算 mimic 派生量
void SimControlHardware::stepBackend(double dt, bool integrate)
{
	backend_->step(cmd_position_, state_position_, state_velocity_, dt, integrate);
}

//发命令 (write() 转发): 同步后端 = 空操作 (命令已在 step 中消化); 异步后端 = cmd 下发 plant
void SimControlHardware::writeBackend()
{
	backend_->transmit(cmd_position_);
}

//状态直写 (瞬移/回零转发): 同步后端宿主数组即 plant 状态 = 空操作; 异步后端 = 下发 plant
void SimControlHardware::stateBackend()
{
	backend_->requestState(state_position_, state_velocity_);
}

void SimControlHardware::drainCommands()
{
	SimCommand cmd;
	while (cmd_queue_.pop(cmd))
	{
		switch (cmd.type)
		{
			case SimCmdType::RESET:
			{
				for (size_t i = 0; i < joints_.size(); ++i)
				{
					if (joints_[i].is_mimic())
					{
						continue;
					}
					state_position_[i] = 0.0;
					state_velocity_[i] = 0.0;
				}
				// 回零: 先直写下发 plant (异步后端; 同步后端 = 空操作), 再同步 mimic 派生量。
				// 顺序不可反: 异步后端的 step 会取旧快照回写宿主数组, 但 plant 侧 override 已入队,
				// 下一拍快照即恢复正确值 (瞬移在 1~2 个宿主周期内生效)
				stateBackend();
				stepBackend(0.0, false);
			}
			break;
			case SimCmdType::SET_STATE:
			{
				for (uint32_t k = 0; k < cmd.count && k < kMaxJoints; ++k)
				{
					if (cmd.mask[k] != 0)
					{
						state_position_[k] = cmd.positions[k];
						state_velocity_[k] = cmd.velocities[k];
					}
				}
				// 瞬移: 先直写下发 plant (异步后端; 同步后端 = 空操作), 再同步 mimic 派生量 (顺序同 RESET)
				stateBackend();
				stepBackend(0.0, false);
			}
			break;
			case SimCmdType::PAUSE:
			{
				paused_.store(true);
			}
			break;
			case SimCmdType::RESUME:
			{
				paused_.store(false);
			}
			break;
			case SimCmdType::STEP:
			{
				step_requests_.fetch_add(1);
			}
			break;
			default:
			{
				// 未处理的命令类型 (枚举值已全覆盖, 此分支不可达)
			}
			break;
		}
	}
}

hardware_interface::return_type SimControlHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	drainCommands();

	const double dt = period.seconds();
	bool integrate = dt > 0.0;
	if (paused_.load())
	{
		if (step_requests_.load() > 0)
		{
			step_requests_.fetch_sub(1);   // 消费一次单步请求
		}
		else
		{
			integrate = false;        // 冻结 (mimic 仍按源推导)
		}
	}

	stepBackend(dt, integrate);
	return hardware_interface::return_type::OK;
}

hardware_interface::return_type SimControlHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	// 同步后端: transmit 为空操作 (命令已在 read 的 step 中消化); 异步后端: 此处发 cmd 给 plant
	writeBackend();
	return hardware_interface::return_type::OK;
}

bool SimControlHardware::validateSetState(const std::vector<std::string> & names, const std::vector<double> & positions,
	const std::vector<double> & velocities, SimCommand & command, std::string & message)
{
	if (names.empty())
	{
		message = "joint_names is empty";
		return false;
	}
	if (positions.size() != names.size() || (!velocities.empty() && velocities.size() != names.size()))
	{
		message = "positions/velocities size mismatch with joint_names";
		return false;
	}

	std::unordered_map<std::string, size_t> index;
	for (size_t i = 0; i < joints_.size(); ++i)
	{
		index[joints_[i].name] = i;
	}

	// 展开为按关节索引填充的命令; 未点名的关节 mask=0, 保持原状态
	command = SimCommand{};
	command.type = SimCmdType::SET_STATE;
	command.count = static_cast<uint32_t>(joints_.size());
	for (size_t k = 0; k < names.size(); ++k)
	{
		auto it = index.find(names[k]);
		if (it == index.end())
		{
			message = "unknown joint '" + names[k] + "'";
			return false;
		}
		const JointMeta & joint = joints_[it->second];
		if (joint.is_mimic())
		{
			message = "joint '" + names[k] + "' is a mimic joint (derived, not settable)";
			return false;
		}
		if (positions[k] < joint.min || positions[k] > joint.max)
		{
			message = "position for '" + names[k] + "' out of limits [" +
				std::to_string(joint.min) + ", " + std::to_string(joint.max) + "]";
			return false;
		}
		command.mask[it->second] = 1;
		command.positions[it->second] = positions[k];
		command.velocities[it->second] = velocities.empty() ? 0.0 : velocities[k];
	}
	return true;
}

bool SimControlHardware::enqueueSink(const SimCommand & cmd, std::string & message)
{
	if (!cmd_queue_.push(cmd))
	{
		message = "sim command queue full";
		return false;
	}
	return true;
}

}  // namespace unistackbot_sim_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unistackbot_sim_control::SimControlHardware, hardware_interface::SystemInterface)

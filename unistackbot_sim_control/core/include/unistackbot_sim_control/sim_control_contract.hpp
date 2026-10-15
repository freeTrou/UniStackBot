#ifndef UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_CONTRACT_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_CONTRACT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unistackbot_interface/joint_capacity.hpp"
#include "unistackbot_sim_control/srv/set_joint_state.hpp"

namespace unistackbot_sim_control
{

// 容量常量住在 unistackbot_interface (与反馈/指令帧共用); 本命名空间内直接使用
using unistackbot_interface::kMaxJoints;

/*
 * /sim_control 统一仿真控制契约 —— 归属统一仿真控制层本包 (2026-09-17 回迁:
 * gz 适配器纯 ROS 化后 ign 依赖消失, 契约与两个后端同包; interface 只留机器人级契约)。
 * 实现方: 本包插件 (mock 链, 命令入队交 RT 循环)、本包 sim_control_gz_node
 *         (gz 链, 经桥接的 ControlWorld 服务下发)。
 */

// 命令帧容量上限 kMaxJoints 已上收至 joint_capacity.hpp (反馈帧/指令帧共用同一常量)

enum class SimCmdType : uint8_t
{
	RESET = 0,       // 全部非 mimic 关节回零, 速度清零
	SET_STATE = 1,   // 瞬移到指定状态 (mask 标记生效关节)
	PAUSE = 2,       // 冻结状态推进
	RESUME = 3,      // 恢复推进
	STEP = 4,        // 暂停状态下推进一步
};

// 定长 POD 命令 —— 无锁队列要求可平凡拷贝 (队列本体 sp_ring.hpp 处另有元素级断言)。
// mask[i]=1 表示关节 i 生效 (由服务端校验器按关节索引填好, mimic 关节不会出现)
struct SimCommand
{
	SimCmdType type{SimCmdType::RESET};
	uint32_t count{0};    // 关节总数快照 (圈定索引范围, 生效与否看 mask)
	std::array<uint8_t, kMaxJoints> mask{};
	std::array<double, kMaxJoints> positions{};
	std::array<double, kMaxJoints> velocities{};
};
static_assert(std::is_trivially_copyable_v<SimCommand>, "SimCommand must be trivially copyable for the lock-free queue");

/*
 * 操作态通道·暂停 (2026-10-15): kPausedTopic = /sim_control/paused
 * (std_msgs/Bool, @kPausedTopicHz 心跳式发布)。
 *
 * 动机: 暂停期间全链源静默 (js/ee/cm 断流) 是暂停的正常形态, 不是故障 ——
 * supervisor 若按静默=断流裁决会把每次 set_joint_state/reset 的标准前置
 * pause 变成假 FAULT (告警疲劳, 实测 2026-10-15)。
 *
 * 生产者 = 各链 /sim_control 适配器:
 *   gz     /stats (WorldStatistics.paused) 真值翻译 —— gz server ground truth,
 *          含 GUI 手点暂停
 *   mujoco 自持命令镜像 (SetPause 命令的状态机) —— MuJoCo 窗口 GUI 暂停不可见
 *          (off-contract 用法, 盲区入档)
 *   mock   不发布 —— 它的 pause 不停 CM 循环 (js 照流, 无假警报问题);
 *          话题缺席 = 消费端默认"运行", 零配置
 *
 * 消费纪律 (supervisor): 心跳式而非 latched —— 流断即回落正常裁决
 * (误报方向安全, 顺带把适配器死亡暴露出来)。
 *
 * 【挂账 2026-10-15】第一期 = 仿真暂停专用。通用操作态框架 (真机软停/维护
 * 模式 = 第二、三个生产者) 待需求落地再定形状, 不提前造 —— 用户标记待更优想法。
 */
inline constexpr const char * kPausedTopic = "/sim_control/paused";
inline constexpr double kPausedTopicHz = 10.0;

/*
 * /sim_control 系列服务的统一门面。
 * 在给定节点上创建 5 个绝对名服务:
 *   /sim_control/reset            std_srvs/srv/Trigger
 *   /sim_control/set_joint_state  unistackbot_interface/srv/SetJointState
 *   /sim_control/pause            std_srvs/srv/Trigger
 *   /sim_control/resume           std_srvs/srv/Trigger
 *   /sim_control/step             std_srvs/srv/Trigger
 * 五个服务挂同一个 MutuallyExclusiveCallbackGroup —— 回调串行化,
 * 保证命令队列的严格单生产者前提。
 *
 * 应答语义: success=true 表示命令已被后端"接受"(入队 / ign 请求已发出),
 * 不代表执行完成。
 */
class SimControlServer
{
public:
	/*
	 * 命令下沉函数: 实现方决定命令去向 (mock=入队, gz=ign 服务调用)。
	 * 返回 false + 原因 = 拒绝 (队列满 / 后端不支持), 会透传进服务应答。
	 */
	using Sink = std::function<bool(const SimCommand &, std::string & message)>;

	/*
	 * set_joint_state 校验器: 关节存在性 / 非 mimic / 限位, 实现方特有。
	 * 校验通过后由实现方把请求展开成命令 (mask/positions/velocities 按关节索引填充)。
	 * 返回 false + 原因时命令不会入队。
	 */
	using SetStateValidator = std::function<bool(const std::vector<std::string> & names, const std::vector<double> & positions, const std::vector<double> & velocities, SimCommand & command,
		std::string & message)>;

	//创建五个 /sim_control/* 服务 (同一互斥组, 回调串行)
	SimControlServer(const rclcpp::Node::SharedPtr & node, Sink sink, SetStateValidator validate_set_state)
	{
		callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		auto make_trigger_service = [&](const std::string & name, SimCmdType type)
		{
			auto cb = [this, sink, type](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
			{
				SimCommand cmd;
				cmd.type = type;
				response->success = sink(cmd, response->message);
			};
			services_.push_back(node->create_service<std_srvs::srv::Trigger>(name, cb, rmw_qos_profile_services_default, callback_group_));
		};

		make_trigger_service("/sim_control/reset", SimCmdType::RESET);
		make_trigger_service("/sim_control/pause", SimCmdType::PAUSE);
		make_trigger_service("/sim_control/resume", SimCmdType::RESUME);
		make_trigger_service("/sim_control/step", SimCmdType::STEP);

		auto set_state_cb = [this, sink, validate_set_state](const std::shared_ptr<srv::SetJointState::Request> request, std::shared_ptr<srv::SetJointState::Response> response)
		{
			SimCommand cmd;
			cmd.type = SimCmdType::SET_STATE;
			std::string message;
			// 校验通过才允许入队; 拒绝原因原样透传给客户端
			if (!validate_set_state(request->joint_names, request->positions, request->velocities, cmd, message))
			{
				response->success = false;
				response->message = message;
				return;
			}
			response->success = sink(cmd, response->message);
		};
		services_.push_back(node->create_service<srv::SetJointState>("/sim_control/set_joint_state", set_state_cb, rmw_qos_profile_services_default, callback_group_));
	}

private:
	rclcpp::CallbackGroup::SharedPtr callback_group_;    // 互斥组: 五服务串行 → 队列单生产者保证
	std::vector<rclcpp::ServiceBase::SharedPtr> services_;   // 服务句柄 (成员保活, 析构即下线)
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_CONTROL_CONTRACT_HPP_

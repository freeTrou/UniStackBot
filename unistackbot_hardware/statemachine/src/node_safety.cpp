#include "unistackbot_statemachine/node_safety.hpp"

namespace unistackbot_statemachine
{

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;

void NodeSafety::update(const NodeFeedback &fb)
{
	last_fb_ = fb;

	// 状态推进优先级: FAULT > 超时 > 模式（故障是最重事实）
	if (fb.error_raw != 0u)
	{
		state_ = NeutralState::kFault;
		// 故障进入时清使能请求（不允许带故障使能）
		enable_req_ = false;
		return;
	}

	if (fb.timeout_triggered)
	{
		state_ = NeutralState::kQuickStop;
		return;
	}

	if (fb.mode == NodeMode::kRun)
	{
		state_ = NeutralState::kEnabled;
		return;
	}

	// mode=0 且无故障无超时
	state_ = NeutralState::kReady;

	// 如果有使能请求且守卫满足 → 标记为待使能（泵会取 enable_command()）
	// 注意: 实际的状态转换要等反馈确认 mode=1 才算完成
	// 这里只是预检——真正的 ENERGIZED 由下一拍 update() 确认
}

bool NodeSafety::can_enable() const
{
	// 守卫: 有反馈（非 OFFLINE）+ 无故障 + 非超时态
	return state_ == NeutralState::kReady;
}

bool NodeSafety::can_clear_fault() const
{
	return state_ == NeutralState::kFault;
}

NodeCommand NodeSafety::enable_command(double lock_kp, double lock_kd) const
{
	NodeCommand cmd;
	// 核心安全: position = 实测位（防跳变）
	// 如果在 DE-ENERGIZED 期间臂因重力下垂了, last_fb_.position 就是新位置
	// 使能命令锁在这里 → 电机原地锁定 → 零跳变
	cmd.mode = NodeMode::kRun;
	cmd.position = last_fb_.position;
	cmd.kp = lock_kp;
	cmd.kd = lock_kd;
	cmd.speed = 0.0;
	cmd.tau = 0.0;
	cmd.watchdog_enable = true;
	return cmd;
}

NodeCommand NodeSafety::disable_command() const
{
	NodeCommand cmd;
	cmd.mode = NodeMode::kStop;
	cmd.position = 0.0;
	cmd.kp = 0.0;
	cmd.kd = 0.0;
	cmd.speed = 0.0;
	cmd.tau = 0.0;
	cmd.watchdog_enable = true;   // 断使能也开着门狗——总线断时电机自停
	return cmd;
}

void NodeSafety::request_enable()
{
	if (can_enable())
	{
		enable_req_ = true;
		disable_req_ = false;
	}
}

void NodeSafety::request_disable()
{
	// 任何非 OFFLINE 状态都可以请求断使能
	if (state_ != NeutralState::kUnknown)
	{
		disable_req_ = true;
		enable_req_ = false;
	}
}

void NodeSafety::request_fault_clear()
{
	if (can_clear_fault())
	{
		fault_clear_req_ = true;
	}
}

void NodeSafety::clear_requests()
{
	enable_req_ = false;
	disable_req_ = false;
}

}  // namespace unistackbot_statemachine

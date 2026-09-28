#include "unistackbot_statemachine/node_safety.hpp"

namespace unistackbot_statemachine
{

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;

void NodeSafety::update(const NodeFeedback &fb)
{
	last_fb_ = fb;

	// ── 标准状态机: switch(当前态) × 输入事件 → 下一态 ──
	// 每个 case 明确列出合法迁移; 非法迁移不发生 (状态保持)。
	// FAULT 是粘性的: 一旦进入, 只有 request_fault_clear() + error=0 确认才能离开。

	switch (state_)
	{
	// ── OFFLINE: 无反馈, 等第一帧 ──
	case NeutralState::kUnknown:
		if (fb.error_raw != 0u)
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.mode == NodeMode::kRun)
		{
			state_ = NeutralState::kEnabled;
		}
		else
		{
			state_ = NeutralState::kReady;
		}
		break;

	// ── DE-ENERGIZED: mode=0, 电机自由 ──
	case NeutralState::kReady:
		if (fb.error_raw != 0u)
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.timeout_triggered)
		{
			state_ = NeutralState::kQuickStop;
		}
		else if (fb.mode == NodeMode::kRun)
		{
			state_ = NeutralState::kEnabled;
		}
		// mode=0 无异常 → 保持 kReady
		break;

	// ── ENERGIZED: mode=1, 执行命令 ──
	case NeutralState::kEnabled:
		if (fb.error_raw != 0u)
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.timeout_triggered)
		{
			state_ = NeutralState::kQuickStop;
		}
		else if (fb.mode == NodeMode::kStop)
		{
			state_ = NeutralState::kReady;
		}
		// mode=1 无异常 → 保持 kEnabled
		break;

	// ── QUICK_STOP: 看门狗触发, 受控减速 ──
	case NeutralState::kQuickStop:
		if (fb.error_raw != 0u)
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (!fb.timeout_triggered && fb.mode == NodeMode::kRun)
		{
			// 超时已清 + FOC 确认 → 回使能
			state_ = NeutralState::kEnabled;
		}
		else if (!fb.timeout_triggered && fb.mode == NodeMode::kStop)
		{
			// 超时已清 + 停机确认 → 回安全态
			state_ = NeutralState::kReady;
		}
		// 超时未清 → 保持 kQuickStop
		break;

	// ── FAULT: 粘性——不自动离开, 须显式清 ──
	case NeutralState::kFault:
		// 即使 error 瞬间回零也不离开——间歇性故障不能自动恢复 (防复撞)。
		// 离开路径: request_fault_clear() → 外部发清故障帧 → error 持续为 0
		//          → confirm_fault_cleared() 由外部在确认后调用。
		break;

	default:
		break;
	}
}

void NodeSafety::confirm_fault_cleared()
{
	// 只在 FAULT 态且 error 确实为 0 时生效 (由外部在发送清故障帧后调用)
	if (state_ == NeutralState::kFault && last_fb_.error_raw == 0u)
	{
		state_ = NeutralState::kReady;
		fault_clear_req_ = false;
	}
}

bool NodeSafety::can_enable() const
{
	return state_ == NeutralState::kReady;
}

bool NodeSafety::can_clear_fault() const
{
	return state_ == NeutralState::kFault;
}

NodeCommand NodeSafety::enable_command(double lock_kp, double lock_kd) const
{
	NodeCommand cmd;
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
	cmd.watchdog_enable = true;
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

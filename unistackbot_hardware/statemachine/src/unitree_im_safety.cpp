#include "unistackbot_statemachine/unitree_im_safety.hpp"

#include "unistackbot_protocol/unitree_im_core.hpp"

namespace unistackbot_statemachine
{

void UnitreeImSafety::update(const unistackbot_protocol::NodeFeedback & fb)
{
	last_fb_ = fb;

	// ── 标准状态机: switch(当前态) × 输入事件 → 下一态 ──
	// 每个 case 明确列出合法迁移; 非法迁移不发生 (状态保持)。
	// FAULT 是粘性的: 一旦进入, 只有 confirm_fault_cleared() 握手才能离开。

	switch (state_)
	{
	// ── OFFLINE: 无反馈, 等第一帧 ──
	case NeutralState::kUnknown:
		if (fb.error_raw != 0u)
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.mode == unistackbot_protocol::NodeMode::kRun)
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
		else if (fb.mode == unistackbot_protocol::NodeMode::kRun)
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
		else if (fb.mode == unistackbot_protocol::NodeMode::kStop)
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
		else if (!fb.timeout_triggered && fb.mode == unistackbot_protocol::NodeMode::kRun)
		{
			// 超时已清 + FOC 确认 → 回使能
			state_ = NeutralState::kEnabled;
		}
		else if (!fb.timeout_triggered && fb.mode == unistackbot_protocol::NodeMode::kStop)
		{
			// 超时已清 + 停机确认 → 回安全态
			state_ = NeutralState::kReady;
		}
		// 超时未清 → 保持 kQuickStop
		break;

	// ── FAULT: 粘性——不自动离开, 须显式清 ──
	// 即使 error 瞬间回零也不离开——间歇性故障不能自动恢复 (防复撞)。
	// 离开路径: request_fault_clear() → 泵发 kClear 帧 → error 持续为 0
	//          → confirm_fault_cleared() 由泵在确认后调用。
	case NeutralState::kFault:
		break;

	default:
		break;
	}

	// 请求生命周期机器自持: 命令被设备确认即清 (CiA402 链式使能同款纪律;
	// 泵不再代清——链式机器的请求须活到链走完)
	if (fb.mode == unistackbot_protocol::NodeMode::kRun)
	{
		enable_req_ = false;
	}
	if (fb.mode == unistackbot_protocol::NodeMode::kStop)
	{
		disable_req_ = false;
	}
}

unistackbot_protocol::NodeCommand UnitreeImSafety::enable_command() const
{
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kRun;
	cmd.position = last_fb_.position;   // 锚定实测位 (防跳变核心)
	cmd.kp = unistackbot_protocol::unitree_im::kPriorKp;   // 增益先验 (协议轴单一事实源)
	cmd.kd = unistackbot_protocol::unitree_im::kPriorKd;
	cmd.speed = 0.0;
	cmd.tau = 0.0;
	cmd.watchdog_enable = true;
	return cmd;
}

unistackbot_protocol::NodeCommand UnitreeImSafety::disable_command() const
{
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kStop;
	cmd.position = 0.0;
	cmd.kp = 0.0;
	cmd.kd = 0.0;
	cmd.speed = 0.0;
	cmd.tau = 0.0;
	cmd.watchdog_enable = true;   // 安全怠速: 停机帧 + 看门狗位
	return cmd;
}

}  // namespace unistackbot_statemachine

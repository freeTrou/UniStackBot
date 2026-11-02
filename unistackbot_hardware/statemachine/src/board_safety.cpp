#include "unistackbot_statemachine/board_safety.hpp"

namespace unistackbot_statemachine
{

void BoardSafety::update(const unistackbot_protocol::NodeFeedback & fb)
{
	last_fb_ = fb;
	if (fb.error_raw != 0u)
	{
		state_ = NeutralState::kFault;   // 粘性: confirm_fault_cleared 握手离开
		return;
	}
	if (fb.timeout_triggered)
	{
		state_ = NeutralState::kUnknown;
		return;
	}
	state_ = (fb.state_raw & 0x100u) != 0u ? NeutralState::kEnabled : NeutralState::kReady;
}

unistackbot_protocol::NodeCommand BoardSafety::enable_command() const
{
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kRun;
	cmd.watchdog_enable = true;
	cmd.position = last_fb_.position;   // 锚定实测位 (先例: 使能不产生运动)
	return cmd;
}

unistackbot_protocol::NodeCommand BoardSafety::disable_command() const
{
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kStop;
	return cmd;
}

unistackbot_protocol::NodeCommand BoardSafety::safety_stop_command() const
{
	return disable_command();
}

unistackbot_protocol::NodeCommand BoardSafety::idle_command() const
{
	return disable_command();
}

}  // namespace unistackbot_statemachine

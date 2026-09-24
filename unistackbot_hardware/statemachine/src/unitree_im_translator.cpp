#include "unistackbot_statemachine/unitree_im_translator.hpp"

namespace unistackbot_statemachine
{

NeutralState UnitreeImTranslator::map_state(
	const unistackbot_protocol::NodeFeedback &fb) const
{
	if (fb.error_raw != 0u)
	{
		return NeutralState::kFault;
	}
	if (fb.timeout_triggered)
	{
		return NeutralState::kQuickStop;
	}
	if (fb.mode == unistackbot_protocol::NodeMode::kRun)
	{
		return NeutralState::kEnabled;
	}
	return NeutralState::kReady;
}

}  // namespace unistackbot_statemachine

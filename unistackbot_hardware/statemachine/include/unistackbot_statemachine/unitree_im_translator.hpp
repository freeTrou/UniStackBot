#ifndef UNISTACKBOT_STATEMACHINE__UNITREE_IM_TRANSLATOR_HPP_
#define UNISTACKBOT_STATEMACHINE__UNITREE_IM_TRANSLATOR_HPP_

#include "unistackbot_statemachine/state_translator.hpp"

// UnitreeImTranslator —— 宇树 IM 系反馈位 → 中立四态的翻译器。
// 位语义 (协议文档): mode 0=停机 1=FOC; timeout 位=超时保护触发; MError≠0=故障。
// 优先级: 故障 > 超时 > 模式 (故障位码触发即自动停机, 是最重事实)。

namespace unistackbot_statemachine
{

class UnitreeImTranslator final : public StateTranslator
{
public:
	[[nodiscard]] NeutralState map_state(
		const unistackbot_protocol::NodeFeedback &fb) const override;
};

}  // namespace unistackbot_statemachine

#endif

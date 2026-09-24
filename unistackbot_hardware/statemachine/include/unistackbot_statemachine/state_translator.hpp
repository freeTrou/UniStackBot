#ifndef UNISTACKBOT_STATEMACHINE__STATE_TRANSLATOR_HPP_
#define UNISTACKBOT_STATEMACHINE__STATE_TRANSLATOR_HPP_

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/neutral_state.hpp"

// StateTranslator —— 状态机翻译轴的父类 (2026-09-24 拆包, 同协议轴组织:
// 父类定接口 · 子类实现 · 工厂按名创建 · 型号↔翻译器由机器配置显式声明)。
//
// 职责: 统一反馈 (NodeFeedback, 各协议 decode 的共同产物) → 中立状态 (NeutralState)。
// 消费方: 总线骨架的 health (粗态恒真层)。
//
// 延后项 (架构文档 §2.1/§2.3 加法地图): plan() 动作序列 (中立命令 → 协议动作链,
// 等 CiA402 三步使能链出现) / 细分格 STANDBY·瞬态终态 / 影子状态机 (裸协议推断)。

namespace unistackbot_statemachine
{

class StateTranslator
{
public:
	virtual ~StateTranslator() = default;

	// 观察方向: 反馈 → 粗态。"无反馈"由骨架 health 管, 不在此。
	[[nodiscard]] virtual NeutralState map_state(
		const unistackbot_protocol::NodeFeedback &fb) const = 0;
};

}  // namespace unistackbot_statemachine

#endif

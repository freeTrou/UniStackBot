#ifndef UNISTACKBOT_STATEMACHINE__UNITREE_IM_SAFETY_HPP_
#define UNISTACKBOT_STATEMACHINE__UNITREE_IM_SAFETY_HPP_

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/node_safety.hpp"

// UnitreeImSafety —— 宇树 IM 系的安全状态机子类 (2026-10-10 从 NodeSafety 具体
// 类下放; MIT 协议语义与中立骨架分离——三面同变纪律)。
//
// 映射规则 (协议知识): error_raw≠0 → FAULT / timeout 位 → QUICK_STOP /
// mode=FOC → ENABLED / mode=停机 → READY; FAULT 粘性 (父类握手退出)。
//
// 使能命令: MIT 锚定实测位 (防跳变核心) + 增益先验 kPriorKp/kPriorKd (协议轴
// 单一事实源)——"使能即锁定"是宇树 MIT 特有语义, CiA402 无此概念。
// 断使能命令: mode 0 + 看门狗位 (安全怠速, 驱动器看门狗为安全终点)。
// 清故障: request → 泵发 kClear 帧 (mode 6 + tor_raw=-256 魔数) → error=0
// 确认 → confirm_fault_cleared()。

namespace unistackbot_statemachine
{

class UnitreeImSafety final : public NodeSafety
{
public:
	void update(const unistackbot_protocol::NodeFeedback & fb) override;

	[[nodiscard]] unistackbot_protocol::NodeCommand enable_command() const override;
	[[nodiscard]] unistackbot_protocol::NodeCommand disable_command() const override;
};

}  // namespace unistackbot_statemachine
#endif  // UNISTACKBOT_STATEMACHINE__UNITREE_IM_SAFETY_HPP_

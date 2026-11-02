#ifndef UNISTACKBOT_STATEMACHINE__BOARD_SAFETY_HPP_
#define UNISTACKBOT_STATEMACHINE__BOARD_SAFETY_HPP_
/*
 * BoardSafety —— Lumos UDP 电控板节点安全机 (2026-11-02 总线/协议拆分随建;
 * UnitreeImSafety/Cia402Safety 同款组织: 父类骨架 + 协议子类语义 + 工厂注册)。
 *
 * 映射规则 (udp_board 协议知识; state_raw 打包见 board_protocol.hpp):
 *   error_raw != 0          → kFault (父类粘性, 清故障走 reset 拍握手)
 *   timeout_triggered        → kUnknown (信念撤销)
 *   enable 位 (state_raw bit8) → kEnabled
 *   其余                     → kReady
 *
 * 命令形态: 板无独立使能/停机帧——使能位就在 Control 帧内, 由聚合编解码按
 * NodeMode 落位 (kRun/kEnable→1, 其余→0)。故本类命令生成 = 纯模式词:
 *   enable  = kRun   (下一拍 enable 位拉起)
 *   disable = kStop  (三相释放; P20 臂自重下坠语义 = 板侧保护兜底, 真机批次复核)
 *   safety_stop/idle = kStop (同 disable; 板看门狗由帧节拍本身维持)
 *
 * 清故障: ctrl_word=Reset(0x1) 特殊拍 (主站发) → error 持续 0 → confirm_fault_cleared。
 * M4 真板批次验证该语义 (mos 侧无实测证据, provisional)。
 */
#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/node_safety.hpp"

namespace unistackbot_statemachine
{

class BoardSafety final : public NodeSafety
{
public:
	void update(const unistackbot_protocol::NodeFeedback & fb) override;
	[[nodiscard]] unistackbot_protocol::NodeCommand enable_command() const override;
	[[nodiscard]] unistackbot_protocol::NodeCommand disable_command() const override;
	[[nodiscard]] unistackbot_protocol::NodeCommand safety_stop_command() const override;
	[[nodiscard]] unistackbot_protocol::NodeCommand idle_command() const override;
};

}  // namespace unistackbot_statemachine
#endif  // UNISTACKBOT_STATEMACHINE__BOARD_SAFETY_HPP_

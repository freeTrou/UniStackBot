#ifndef UNISTACKBOT_STATEMACHINE__NODE_SAFETY_HPP_
#define UNISTACKBOT_STATEMACHINE__NODE_SAFETY_HPP_

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/neutral_state.hpp"

// NodeSafety —— 单节点安全状态机父类 (2026-10-10 重构归位, 同 protocol/bus 组织:
// 父类中立骨架 · 协议子类实现 · 工厂按名创建 · 总线内部调用)。
//
// **总线内部组件** (用户硬约束): 由总线主站经工厂自建自持, 住泵 RT 循环——每时隙
// 喂一帧反馈 (update), 内联消费其命令生成 (resolveSlotCmd)。非上层/ROS 侧驱动。
// RT 纪律: update/命令生成零分配零锁零 syscall; 实例在 start 期创建, 循环期不换。
//
// 父类持有 (协议无关骨架):
//   - 状态词汇 (NeutralState) 与最近反馈缓存
//   - 粘性原则: FAULT 不自动离开, 走 request → 协议清帧 → confirm 握手
//   - 意图请求机制: enable/disable 请求标志 (泵下一拍消费)
//   - 守卫结构: can_enable / can_clear_fault (按粗态)
// 子类持有 (协议语义, 随协议同变——三面同变纪律):
//   - 反馈字段 → 粗态的映射规则 (update)
//   - 使能/断使能命令生成 (MIT 锚定 kp 等)
//
// 历史注: 2026-09-24 首版为 StateTranslator (无状态 map_state) 轴; 2026-09-28
// 并入有状态 NodeSafety 具体类; 2026-10-10 审核定形为父类+子类+工厂 (map_state
// 双机并存退役——每协议一个映射一份测试)。

#include <cstdint>

namespace unistackbot_statemachine
{

class NodeSafety
{
public:
	virtual ~NodeSafety() = default;

	// ── 协议语义 (子类实现) ──

	// 喂一帧反馈, 推进状态机 (映射规则 = 协议知识)。
	virtual void update(const unistackbot_protocol::NodeFeedback & fb) = 0;

	// 使能命令: 协议各自的安全使能形态 (MIT=锚定实测位 + 先验 kp/kd)。
	// 返回值可直接交给 ProtocolCore::encode。
	[[nodiscard]] virtual unistackbot_protocol::NodeCommand enable_command() const = 0;

	// 断使能命令: 协议各自的安全停机形态 (unitree = mode 0 + 看门狗位)。
	[[nodiscard]] virtual unistackbot_protocol::NodeCommand disable_command() const = 0;

	// ── 中立骨架 (协议无关) ──

	// 当前粗态 (快照层可在其上按 last_seen 超陈覆盖为 kUnknown——信念撤销, 不改本机)。
	[[nodiscard]] NeutralState state() const { return state_; }

	// 最近反馈 (命令生成/锚定的数据源)。
	[[nodiscard]] const unistackbot_protocol::NodeFeedback & last_feedback() const
	{
		return last_fb_;
	}

	// 能否使能: 守卫 = 在 READY 态 (有新鲜反馈 + 无故障由状态机映射保证)。
	[[nodiscard]] bool can_enable() const { return state_ == NeutralState::kReady; }

	// 能否清故障: 只有 FAULT 态需要清。
	[[nodiscard]] bool can_clear_fault() const { return state_ == NeutralState::kFault; }

	// ── 意图请求 (泵下一拍消费) ──

	// 请求使能 (仅在 READY 守卫下接受)。
	void request_enable()
	{
		if (can_enable())
		{
			enable_req_ = true;
			disable_req_ = false;
		}
	}

	// 请求断使能 (OFFLINE 无意义不设)。
	void request_disable()
	{
		if (state_ != NeutralState::kUnknown)
		{
			disable_req_ = true;
			enable_req_ = false;
		}
	}

	// 请求清故障 (仅 FAULT 态; 过热/过流类不可自动恢复由上层判断后再请求)。
	void request_fault_clear()
	{
		if (can_clear_fault())
		{
			fault_clear_req_ = true;
		}
	}

	[[nodiscard]] bool enable_requested() const { return enable_req_; }
	[[nodiscard]] bool disable_requested() const { return disable_req_; }
	[[nodiscard]] bool fault_clear_requested() const { return fault_clear_req_; }
	void acknowledge_fault_clear() { fault_clear_req_ = false; }

	void clear_requests()
	{
		enable_req_ = false;
		disable_req_ = false;
	}

	// 确认故障已清 (外部发清故障帧且 error 持续为 0 后调用 → FAULT → READY)。
	// 守卫内建: 非 FAULT 或 error 仍非零时调用无效。
	void confirm_fault_cleared()
	{
		if (state_ == NeutralState::kFault && last_fb_.error_raw == 0u)
		{
			state_ = NeutralState::kReady;
			fault_clear_req_ = false;
		}
	}

protected:
	NeutralState state_ = NeutralState::kUnknown;
	unistackbot_protocol::NodeFeedback last_fb_ = {};
	bool enable_req_ = false;
	bool disable_req_ = false;
	bool fault_clear_req_ = false;
};

}  // namespace unistackbot_statemachine
#endif  // UNISTACKBOT_STATEMACHINE__NODE_SAFETY_HPP_

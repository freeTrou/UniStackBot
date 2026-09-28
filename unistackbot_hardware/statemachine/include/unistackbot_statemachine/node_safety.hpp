#ifndef UNISTACKBOT_STATEMACHINE__NODE_SAFETY_HPP_
#define UNISTACKBOT_STATEMACHINE__NODE_SAFETY_HPP_

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/neutral_state.hpp"

// NodeSafety —— 单节点安全状态机 (2026-09-28, 四态保护模型)。
//
// 职责: 跟踪节点状态 + 执行安全流转守卫 + 生成安全命令。
// 它是**有状态的**（与 StateTranslator 的纯观察不同）——泵每拍喂反馈，
// NodeSafety 维护状态并回答"能不能使能""怎么安全使能"。
//
// 四态模型:
//   OFFLINE      无反馈（总线断/电机断电）
//   DE-ENERGIZED 电机自由（mode=0，人可碰）
//   ENERGIZED    电机施力（mode=1，按命令走）
//   FAULT        故障停机（error≠0，需人工清）
//
// 安全规则:
//   ①使能必须锚定实测位（防跳变）
//   ②故障不可自动清除（防复撞）
//   ③看不着就不使能（OFFLINE 禁止→ENERGIZED）
//   ④断使能期间位置追踪不断流（泵持续发停机帧，反馈持续回来）

namespace unistackbot_statemachine
{

// 流转结果（显式错误流）
enum class TransitionResult : std::uint8_t
{
	kOk = 0,
	kRejected,       // 守卫不满足（状态不对/反馈陈旧/有故障）
	kNoChange,       // 已在目标态
};

class NodeSafety
{
public:
	// 每拍更新（泵线程调用；喂最新反馈，内部推进状态）
	void update(const unistackbot_protocol::NodeFeedback &fb);

	// 当前状态
	[[nodiscard]] NeutralState state() const { return state_; }

	// 最近反馈（使能命令用它锚定位置）
	[[nodiscard]] const unistackbot_protocol::NodeFeedback &last_feedback() const
	{
		return last_fb_;
	}

	// ── 安全守卫 ──

	// 能否安全使能？守卫: 有新鲜反馈 + 无故障
	[[nodiscard]] bool can_enable() const;

	// 能否清除故障？守卫: 当前在 FAULT 态（只有 FAULT 态才需要清）
	[[nodiscard]] bool can_clear_fault() const;

	// ── 安全命令生成 ──

	// 使能命令: 锚定实测位（防跳变的核心）
	// 返回的 NodeCommand 可直接交给 ProtocolCore::encode
	[[nodiscard]] unistackbot_protocol::NodeCommand enable_command(
		double lock_kp, double lock_kd) const;

	// 断使能命令: mode=0 + 看门狗位（安全怠速）
	[[nodiscard]] unistackbot_protocol::NodeCommand disable_command() const;

	// 故障清除请求标志（泵线程看到后发清故障帧——协议特有操作由协议层处理）
	[[nodiscard]] bool fault_clear_requested() const { return fault_clear_req_; }
	void acknowledge_fault_clear() { fault_clear_req_ = false; }

	// ── 状态推进（总线线程或外部触发） ──

	// 请求使能（设置意图标志; 下一拍总线循环消费请求并生成安全命令）
	void request_enable();
	// 请求断使能
	void request_disable();
	// 请求清故障（只在 FAULT 态有效; 过热/过流类不可自动恢复由上层判断）
	void request_fault_clear();
	// 确认故障已清（外部发清故障帧且 error 持续为 0 后调用 → FAULT → READY）
	void confirm_fault_cleared();

	// ── 内部 ──
	[[nodiscard]] bool enable_requested() const { return enable_req_; }
	[[nodiscard]] bool disable_requested() const { return disable_req_; }
	void clear_requests();

private:
	NeutralState state_ = NeutralState::kUnknown;
	unistackbot_protocol::NodeFeedback last_fb_ = {};
	bool enable_req_ = false;
	bool disable_req_ = false;
	bool fault_clear_req_ = false;
};

}  // namespace unistackbot_statemachine

#endif

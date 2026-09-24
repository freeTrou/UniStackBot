#ifndef UNISTACKBOT_PROTOCOL__NEUTRAL_STATE_HPP_
#define UNISTACKBOT_PROTOCOL__NEUTRAL_STATE_HPP_

#include <cstdint>

namespace unistackbot_statemachine
{

// 中立语义状态 —— 粗态恒真层 (架构文档 real_hardware_architecture.md §2.1)。
// 细分格 (STANDBY/瞬态终态) 与影子推断为显式延后项: 等第二个协议出现按能力位加,
// 加法只加不改 (接口只加不改纪律)。
enum class NeutralState : std::uint8_t
{
	kUnknown = 0,   // 尚无反馈 (骨架 health 的"无数据"态, 不由 map_state 产生)
	kReady,         // 停机/可使能
	kEnabled,       // 执行中
	kQuickStop,     // 超时保护/急停触发
	kFault,         // 故障 (错误位码非零)
};

[[nodiscard]] inline const char *toText(NeutralState s)
{
	switch (s)
	{
	case NeutralState::kUnknown: return "UNKNOWN";
	case NeutralState::kReady: return "READY";
	case NeutralState::kEnabled: return "ENABLED";
	case NeutralState::kQuickStop: return "QUICK_STOP";
	case NeutralState::kFault: return "FAULT";
	}
	return "?";
}

}  // namespace unistackbot_statemachine

#endif

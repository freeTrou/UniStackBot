#include "unistackbot_statemachine/cia402_safety.hpp"

namespace unistackbot_statemachine
{

namespace
{

// controlword 标准命令 (0x6040) —— 链上各步
constexpr std::uint32_t kCwShutdown = 0x06;         // SOS→RTSO (也 SO→RTSO)
constexpr std::uint32_t kCwSwitchOn = 0x07;         // RTSO→SO (也 OE→SO)
constexpr std::uint32_t kCwEnableOp = 0x0F;         // SO→OE
constexpr std::uint32_t kCwDisableVoltage = 0x00;   // 任意→SOS (安全怠速)
constexpr std::uint32_t kCwFaultReset = 0x80;       // Fault→SOS (上升沿, 泵经 kClear 发)

// statusword 七态标准解码 (mask 0x6F, IEC 61800-7-200 表)
[[nodiscard]] bool decodeCia402(std::uint32_t sw, Cia402State & out)
{
	switch (sw & 0x6Fu)
	{
		case 0x40u: out = Cia402State::kSwitchOnDisabled; return true;
		case 0x21u: out = Cia402State::kReadyToSwitchOn; return true;
		case 0x23u: out = Cia402State::kSwitchedOn; return true;
		case 0x27u: out = Cia402State::kOperationEnabled; return true;
		case 0x07u: out = Cia402State::kQuickStopActive; return true;
		case 0x0Fu: out = Cia402State::kFaultReactionActive; return true;
		case 0x08u: out = Cia402State::kFault; return true;
		default: return false;   // NotReady/厂商扩展位期 —— 原生态保持不变
	}
}

}  // namespace

const char *cia402ToText(Cia402State s)
{
	switch (s)
	{
		case Cia402State::kNotReady: return "NOT_READY";
		case Cia402State::kSwitchOnDisabled: return "SWITCH_ON_DISABLED";
		case Cia402State::kReadyToSwitchOn: return "READY_TO_SWITCH_ON";
		case Cia402State::kSwitchedOn: return "SWITCHED_ON";
		case Cia402State::kOperationEnabled: return "OPERATION_ENABLED";
		case Cia402State::kQuickStopActive: return "QUICK_STOP_ACTIVE";
		case Cia402State::kFaultReactionActive: return "FAULT_REACTION_ACTIVE";
		case Cia402State::kFault: return "FAULT";
	}
	return "?";
}

void Cia402Safety::update(const unistackbot_protocol::NodeFeedback & fb)
{
	last_fb_ = fb;
	sw_ = fb.state_raw;

	// 原生层: 七态解码 (未知组合保持原态 —— statusword 过渡期是正常现象)
	Cia402State n = native_;
	const bool decoded = decodeCia402(fb.state_raw, n);
	if (decoded)
	{
		native_ = n;
	}

	// 决策层: 七态 → 粗态投影 (设备是权威, 纯 statusword 驱动)
	switch (native_)
	{
		case Cia402State::kOperationEnabled:
			state_ = NeutralState::kEnabled;
			break;
		case Cia402State::kQuickStopActive:
			state_ = NeutralState::kQuickStop;
			break;
		case Cia402State::kFaultReactionActive:
		case Cia402State::kFault:
			state_ = NeutralState::kFault;
			enable_req_ = false;   // 故障即撤使能意图 (骨架纪律同款)
			break;
		default:   // SOS / RTSO / SO / NotReady → 未出力/未上电 = 可使能面
			state_ = NeutralState::kReady;
			break;
	}

	// 请求生命周期机器自持 (链式使能的关键: 泵不清请求, 确认即清)
	if (native_ == Cia402State::kOperationEnabled)
	{
		enable_req_ = false;                 // 链走完
	}
	if (native_ == Cia402State::kSwitchOnDisabled)
	{
		disable_req_ = false;                // 已到安全怠速
	}
	if (fault_clear_req_ && native_ != Cia402State::kFault &&
		native_ != Cia402State::kFaultReactionActive)
	{
		fault_clear_req_ = false;            // 设备已离开故障 (statusword 为证)
	}
}

unistackbot_protocol::NodeCommand Cia402Safety::enable_command() const
{
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kRun;
	cmd.watchdog_enable = true;
	cmd.position = last_fb_.position;      // 目标位置锚实测 (防跳变, 同骨架纪律)

	// 链上下一步 (每次调用一步, 设备确认后 update() 推进原生状态)
	switch (native_)
	{
		case Cia402State::kSwitchOnDisabled:
			cmd.cmd_raw = kCwShutdown;         // → RTSO
			break;
		case Cia402State::kReadyToSwitchOn:
			cmd.cmd_raw = kCwSwitchOn;         // → SO
			break;
		case Cia402State::kSwitchedOn:
			cmd.cmd_raw = kCwEnableOp;         // → OE
			break;
		default:   // OE=已使能(保持帧) / 故障急停态=不该被要求使能 → 安全怠速兜底
			cmd.cmd_raw = native_ == Cia402State::kOperationEnabled
				? kCwEnableOp : kCwDisableVoltage;
			break;
	}
	return cmd;
}

unistackbot_protocol::NodeCommand Cia402Safety::disable_command() const
{
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kStop;
	cmd.watchdog_enable = true;
	cmd.cmd_raw = kCwDisableVoltage;   // → SOS (人可碰; 受控减速变体属驱动器 0x605A 配置)
	return cmd;
}

}  // namespace unistackbot_statemachine

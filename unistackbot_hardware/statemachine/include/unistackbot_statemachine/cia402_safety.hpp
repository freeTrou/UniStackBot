#ifndef UNISTACKBOT_STATEMACHINE__CIA402_SAFETY_HPP_
#define UNISTACKBOT_STATEMACHINE__CIA402_SAFETY_HPP_

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/node_safety.hpp"

// Cia402Safety —— CiA402 (IEC 61800-7-200) 官方状态机子类 (2026-10-10, 第二个
// 安全机 = 重构解耦能力的验收现场)。
//
// **两层词汇在此子类完整成形**:
//   原生层 (驱动官方七态, 细分格): Cia402State 全信息, 经 fb.state_raw (statusword
//     0x6041 原样) 驱动, 需要细节的消费者读 native_state()/statusword();
//   决策层 (粗态投影, 对上承诺): 七态 → NeutralState 五值 (见 project)。
//
// 使能 = 三步链 (controlword 0x06→0x07→0x0F, 每步等 statusword 确认): 泵每时隙
// 调 enable_command() 拿链上下一步, 机器内部记链进度, 设备确认到位即自行推进
// —— 序列性住在机器内部, 不泄漏到接口 (泵代码零改动)。请求生命周期机器自持:
// 链走完 (OE 确认) 自清 enable_req_, 到 SOS 自清 disable_req_。
//
// 清故障: request → 泵发 kClear (cia402 codec 将映射 controlword 0x80 上升沿)
// → 设备离开 Fault (statusword 变化) → 投影自动回 kReady (设备是权威, 本子类
// update 纯 statusword 驱动)。

namespace unistackbot_statemachine
{

// CiA402 官方七态 (细分格层原生态) —— 顺序即 power/state machine 标准排布
enum class Cia402State : std::uint8_t
{
	kNotReady = 0,          // 上电初始化未就绪 (statusword 低位随机期)
	kSwitchOnDisabled,      // Switch On Disabled —— 电压未上, 可自由转动
	kReadyToSwitchOn,       // Ready To Switch On —— 主电未上, 使能链第一步落点
	kSwitchedOn,            // Switched On —— 主电已上, 未出力
	kOperationEnabled,      // Operation Enabled —— 出力中
	kQuickStopActive,       // Quick Stop Active —— 急停减速中
	kFaultReactionActive,   // Fault Reaction Active —— 故障反应减速中
	kFault,                 // Fault —— 故障停机 (设备自粘, 须 0x80 复位)
};

[[nodiscard]] const char *cia402ToText(Cia402State s);

class Cia402Safety final : public NodeSafety
{
public:
	// statusword (fb.state_raw) 驱动: 七态解码 → 原生记录 → 粗态投影 → 请求生命周期
	void update(const unistackbot_protocol::NodeFeedback & fb) override;

	// 三步使能链: 每次调用返回链上下一步 controlword (cmd_raw), 到 OE 后返回保持帧
	[[nodiscard]] unistackbot_protocol::NodeCommand enable_command() const override;

	// 安全怠速: Disable Voltage (0x00) → SOS (人可碰)
	[[nodiscard]] unistackbot_protocol::NodeCommand disable_command() const override;

	// ── 细分格查询 (原生态一等遥测) ──
	[[nodiscard]] Cia402State native_state() const { return native_; }
	[[nodiscard]] std::uint32_t statusword() const { return sw_; }

private:
	Cia402State native_ = Cia402State::kNotReady;
	std::uint32_t sw_ = 0;
};

}  // namespace unistackbot_statemachine
#endif  // UNISTACKBOT_STATEMACHINE__CIA402_SAFETY_HPP_

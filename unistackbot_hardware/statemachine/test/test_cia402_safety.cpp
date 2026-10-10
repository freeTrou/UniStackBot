/*
 * test_cia402_safety —— CiA402 安全机测试 (2026-10-10, 第二个安全机)。
 *
 * 覆盖:
 *   ⓪工厂: cia402 可创建, 注册表双条目
 *   ①七态解码 + 粗态投影 (statusword 标准 mask 0x6F)
 *   ②三步使能链: SOS→0x06→RTSO→0x07→SO→0x0F→OE, 确认即推进, 到 OE 请求自清
 *   ③故障: FRA/Fault→kFault, 设备自粘; 复位链 (request→statusword 变化→kReady+ack)
 *   ④断使能: 0x00→SOS, 到 SOS 请求自清
 *   ⑤原生层一等遥测: native_state()/statusword()
 *   ⑥未知 statusword 组合: 原生态保持 (过渡期正常)
 *
 * 编译运行:
 *   cd unistackbot_hardware/statemachine/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_cia402_safety.cpp \
 *       ../src/unitree_im_safety.cpp ../src/cia402_safety.cpp ../src/node_safety_factory.cpp \
 *       -I../include -I../../protocol/include -o /tmp/test_cia402 && /tmp/test_cia402
 */
#include <cstdio>

#include "unistackbot_statemachine/cia402_safety.hpp"
#include "unistackbot_statemachine/node_safety_factory.hpp"

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::Cia402Safety;
using unistackbot_statemachine::Cia402State;
using unistackbot_statemachine::NeutralState;

namespace
{

int g_fail = 0;
int g_case = 0;

void check(bool ok, const char *what)
{
	++g_case;
	if (!ok)
	{
		++g_fail;
		std::printf("FAIL [%d] %s\n", g_case, what);
	}
}

// statusword 帧 (CiA402 反馈: state_raw = 0x6041 原样; error_raw = 0x603F)
NodeFeedback swFb(std::uint32_t sw, std::uint32_t err = 0u, double pos = 0.0)
{
	NodeFeedback fb;
	fb.state_raw = sw;
	fb.error_raw = err;
	fb.position = pos;
	return fb;
}

// 标准 statusword 值 (低 7 位组合)
constexpr std::uint32_t kSwSOS = 0x40;    // Switch On Disabled
constexpr std::uint32_t kSwRTSO = 0x21;   // Ready To Switch On
constexpr std::uint32_t kSwSO = 0x23;     // Switched On
constexpr std::uint32_t kSwOE = 0x27;     // Operation Enabled
constexpr std::uint32_t kSwQSA = 0x07;    // Quick Stop Active
constexpr std::uint32_t kSwFRA = 0x0F;    // Fault Reaction Active
constexpr std::uint32_t kSwFault = 0x08;  // Fault

}  // namespace

int main()
{
	// ── ⓪ 工厂 ──
	std::unique_ptr<unistackbot_statemachine::NodeSafety> ns =
		unistackbot_statemachine::createNodeSafety("cia402");
	check(ns != nullptr, "factory creates cia402");
	if (!ns)
	{
		return 1;
	}
	const auto names = unistackbot_statemachine::availableNodeSafetyMachines();
	check(names.size() == 2, "registry has two machines");
	bool has_u = false, has_c = false;
	for (const auto &n : names)
	{
		has_u = has_u || n == "unitree_im";
		has_c = has_c || n == "cia402";
	}
	check(has_u && has_c, "registry lists both");
	Cia402Safety &s = static_cast<Cia402Safety &>(*ns);

	// ── ① 七态解码 + 粗态投影 ──
	s.update(swFb(kSwSOS));
	check(s.native_state() == Cia402State::kSwitchOnDisabled, "SOS native");
	check(s.state() == NeutralState::kReady, "SOS → kReady (人可碰)");
	check(s.statusword() == kSwSOS, "statusword telemetry");

	s.update(swFb(kSwRTSO));
	check(s.native_state() == Cia402State::kReadyToSwitchOn, "RTSO native");
	check(s.state() == NeutralState::kReady, "RTSO → kReady");

	s.update(swFb(kSwSO));
	check(s.native_state() == Cia402State::kSwitchedOn, "SO native");
	check(s.state() == NeutralState::kReady, "SO → kReady (主电未出力)");

	s.update(swFb(kSwOE));
	check(s.native_state() == Cia402State::kOperationEnabled, "OE native");
	check(s.state() == NeutralState::kEnabled, "OE → kEnabled");

	s.update(swFb(kSwQSA));
	check(s.state() == NeutralState::kQuickStop, "QSA → kQuickStop");

	s.update(swFb(kSwFRA));
	check(s.native_state() == Cia402State::kFaultReactionActive, "FRA native");
	check(s.state() == NeutralState::kFault, "FRA → kFault");

	s.update(swFb(kSwFault));
	check(s.native_state() == Cia402State::kFault, "Fault native");
	check(s.state() == NeutralState::kFault, "Fault → kFault");

	// ── ② 三步使能链 ──
	s.update(swFb(kSwSOS, 0u, 1.25));   // 回 SOS, 实测位 1.25
	check(s.can_enable(), "SOS can enable");
	s.request_enable();
	check(s.enable_requested(), "enable requested");

	const NodeCommand c1 = s.enable_command();
	check(c1.cmd_raw == 0x06, "chain step1: Shutdown (0x06)");
	check(c1.mode == NodeMode::kRun, "chain step1 mode=RUN");
	check(c1.position == 1.25, "target pos anchored to measured");

	s.update(swFb(kSwRTSO));            // 设备确认第一步
	check(s.native_state() == Cia402State::kReadyToSwitchOn, "chain: RTSO reached");
	check(s.enable_requested(), "request alive mid-chain (机器自持)");

	const NodeCommand c2 = s.enable_command();
	check(c2.cmd_raw == 0x07, "chain step2: Switch On (0x07)");

	s.update(swFb(kSwSO));
	const NodeCommand c3 = s.enable_command();
	check(c3.cmd_raw == 0x0F, "chain step3: Enable Operation (0x0F)");

	s.update(swFb(kSwOE));
	check(s.state() == NeutralState::kEnabled, "chain done → kEnabled");
	check(!s.enable_requested(), "request self-cleared at OE (机器自持)");

	const NodeCommand c4 = s.enable_command();
	check(c4.cmd_raw == 0x0F, "post-enable hold frame");

	// ── ③ 故障与复位链 ──
	s.update(swFb(kSwFault, 0x1234u));   // 故障 + 错误寄存器非零
	check(s.state() == NeutralState::kFault, "Fault → kFault");
	s.update(swFb(kSwFault));            // 错误寄存器清了, statusword 仍 Fault
	check(s.state() == NeutralState::kFault, "device-sticky: statusword 仍 Fault");
	check(!s.can_enable(), "FAULT cannot enable");

	s.request_fault_clear();
	check(s.fault_clear_requested(), "clear requested");
	s.update(swFb(kSwSOS));              // 复位生效 (codec 将发 0x80), statusword 离开 Fault
	check(s.state() == NeutralState::kReady, "device left fault → kReady");
	check(!s.fault_clear_requested(), "clear request acknowledged by statusword");

	// ── ④ 断使能 ──
	s.update(swFb(kSwOE));
	s.request_disable();
	check(s.disable_requested(), "disable requested");
	const NodeCommand d1 = s.disable_command();
	check(d1.cmd_raw == 0x00, "disable = Disable Voltage (0x00)");
	s.update(swFb(kSwSOS));
	check(s.state() == NeutralState::kReady, "SOS → kReady");
	check(!s.disable_requested(), "disable request self-cleared at SOS");

	// ── ⑥ 未知 statusword 组合: 原生态保持 ──
	s.update(swFb(kSwRTSO));
	s.update(swFb(0x5555u));   // 过渡期杂值 (高位告警等)
	check(s.native_state() == Cia402State::kReadyToSwitchOn, "unknown sw → native held");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

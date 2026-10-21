/*
 * test_dm_safety —— DmSafety 安全状态机测试 (2026-10-21, stationlite 真机接入 A2)。
 *
 * 覆盖: 工厂 (dm4310/dm4340 两 key 同类, 未知名拒, 清单) / 四态推进
 * (Unknown→Ready|Enabled|Fault, Ready↔Enabled) / 全故障码表 (3/4/5/8/9/A/B/C/D/E
 * + 未定义 2/6/7 保守按故障) / FAULT 粘性 (err 回 0/1 不自动离开) / 清故障握手
 * (request→confirm, err 未清时 confirm 无效) / 使能守卫与意图请求守卫 /
 * enable|disable 命令形态 (kEnable 独立帧 / kStop 失能帧) / 请求生命周期自持
 * (err 确认信号清请求) / 经 DmMotorCore 端到端字节验证 (命令→真实帧字节)。
 *
 * 编译运行 (零 ROS, g++ 直编):
 *   cd unistackbot_hardware/statemachine/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_dm_safety.cpp \
 *       ../src/dm_safety.cpp ../src/unitree_im_safety.cpp ../src/cia402_safety.cpp ../src/node_safety_factory.cpp \
 *       ../../protocol/src/dm_motor_core.cpp -I../include -I../../protocol/include \
 *       -o /tmp/test_dm_safety && /tmp/test_dm_safety
 */
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "unistackbot_protocol/dm_motor_core.hpp"
#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/dm_safety.hpp"
#include "unistackbot_statemachine/node_safety_factory.hpp"

using unistackbot_protocol::DmMotorCore;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_protocol::dm_motor::DmMotorType;
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

// 反馈构造 (达妙: 状态住 error_raw, 无模式字段)
NodeFeedback fbErr(std::uint32_t err)
{
	NodeFeedback fb;
	fb.error_raw = err;
	fb.position = 0.5;
	return fb;
}

}  // namespace

int main()
{
	// ---- 工厂 ----
	auto s4310 = unistackbot_statemachine::createNodeSafety("dm4310");
	auto s4340 = unistackbot_statemachine::createNodeSafety("dm4340");
	check(s4310 != nullptr && s4340 != nullptr, "工厂 dm4310/dm4340 创建");
	check(unistackbot_statemachine::createNodeSafety("dm_motor") == nullptr, "未知名拒");
	bool listed = false;
	for (const auto &n : unistackbot_statemachine::availableNodeSafetyMachines())
	{
		listed = listed || n == "dm4310";
	}
	check(listed, "清单含 dm4310");

	unistackbot_statemachine::NodeSafety &sm = *s4310;   // 全程基类指针

	// ---- Unknown 起始三分支 ----
	check(sm.state() == NeutralState::kUnknown, "起始 Unknown");
	sm.update(fbErr(0));
	check(sm.state() == NeutralState::kReady, "Unknown+err0 → Ready");
	sm.update(fbErr(8));
	check(sm.state() == NeutralState::kFault, "Ready+err8 → Fault");

	auto s2 = unistackbot_statemachine::createNodeSafety("dm4340");
	s2->update(fbErr(1));
	check(s2->state() == NeutralState::kEnabled, "Unknown+err1 → Enabled");
	auto s3 = unistackbot_statemachine::createNodeSafety("dm4310");
	s3->update(fbErr(0xE));
	check(s3->state() == NeutralState::kFault, "Unknown+errE → Fault");

	// ---- Ready ↔ Enabled ----
	sm.update(fbErr(0));
	check(sm.state() == NeutralState::kFault, "FAULT 粘性 (err0 不自动离开)");
	sm.confirm_fault_cleared();   // err 已是 0 → 握手成立
	check(sm.state() == NeutralState::kReady, "confirm (err=0) → Ready");
	sm.update(fbErr(1));
	check(sm.state() == NeutralState::kEnabled, "Ready+err1 → Enabled");
	sm.update(fbErr(0));
	check(sm.state() == NeutralState::kReady, "Enabled+err0 → Ready");

	// ---- 全故障码表 (手册 V1.3/V1.4 + 未定义保守) ----
	const std::uint32_t faults[] = {2, 3, 4, 5, 6, 7, 8, 9, 0xA, 0xB, 0xC, 0xD, 0xE};
	for (std::uint32_t err : faults)
	{
		auto sf = unistackbot_statemachine::createNodeSafety("dm4310");
		sf->update(fbErr(err));
		check(sf->state() == NeutralState::kFault, "全码表 → FAULT");
	}

	// ---- FAULT 粘性 + 握手细节 ----
	{
		auto sf = unistackbot_statemachine::createNodeSafety("dm4310");
		sf->update(fbErr(0xB));
		sf->update(fbErr(0));   // 故障间歇回 0
		check(sf->state() == NeutralState::kFault, "粘性: 间歇回 0 不自动离开");
		// 正道: request (FAULT 受理) → 泵发清帧 → err 回 0 → confirm
		sf->request_fault_clear();
		check(sf->fault_clear_requested(), "fault_clear 请求受理");
		sf->confirm_fault_cleared();
		check(sf->state() == NeutralState::kReady, "清故障握手完成 → Ready");
		check(!sf->fault_clear_requested(), "confirm 后请求清");
	}
	{
		auto sf = unistackbot_statemachine::createNodeSafety("dm4310");
		sf->update(fbErr(9));
		sf->request_fault_clear();
		sf->update(fbErr(9));   // 清帧未生效 (err 仍故障)
		sf->confirm_fault_cleared();
		check(sf->state() == NeutralState::kFault, "err 未清 confirm 无效");
	}

	// ---- 使能守卫 + 意图请求 ----
	{
		auto sf = unistackbot_statemachine::createNodeSafety("dm4310");
		sf->update(fbErr(0));   // Ready
		check(sf->can_enable() && !sf->can_clear_fault(), "Ready 守卫");
		sf->request_enable();
		check(sf->enable_requested(), "enable 请求受理");
		sf->update(fbErr(1));   // 使能帧生效
		check(sf->state() == NeutralState::kEnabled && !sf->enable_requested(), "err=1 确认清请求 (生命周期自持)");
		sf->request_enable();
		check(!sf->enable_requested(), "Enabled 态拒 enable 请求");
		sf->request_disable();
		check(sf->disable_requested(), "disable 请求受理");
		sf->update(fbErr(0));
		check(sf->state() == NeutralState::kReady && !sf->disable_requested(), "err=0 确认清 disable 请求");
		// 故障自动撤使能请求
		sf->request_enable();
		sf->update(fbErr(0xA));
		check(sf->state() == NeutralState::kFault && !sf->enable_requested(), "故障撤 enable 请求");
		check(!sf->can_enable(), "FAULT 拒 can_enable");
	}

	// ---- 命令形态 + 端到端字节 (经 DmMotorCore) ----
	{
		auto sf = unistackbot_statemachine::createNodeSafety("dm4310");
		DmMotorCore core(DmMotorType::kDm4310);
		std::uint8_t f[8] = {};
		const NodeCommand en = sf->enable_command();
		check(en.mode == NodeMode::kEnable, "enable = kEnable 独立帧");
		check(core.encode(f, sizeof f, en, 1, 1.0) == 8 && f[7] == 0xFC && f[0] == 0xFF, "enable 端到端 FF×7+FC");
		const NodeCommand dis = sf->disable_command();
		check(dis.mode == NodeMode::kStop, "disable = kStop 失能帧");
		check(core.encode(f, sizeof f, dis, 1, 1.0) == 8 && f[7] == 0xFD, "disable 端到端 FF×7+FD");
		// 清故障帧由泵在 fault_clear_requested 时发 kClear
		NodeCommand clr;
		clr.mode = NodeMode::kClear;
		check(core.encode(f, sizeof f, clr, 1, 1.0) == 8 && f[7] == 0xFB, "clear 端到端 FF×7+FB");
		// 反馈闭环: 状态机吃 DmMotorCore decode 产物
		const std::uint8_t fbframe[8] = {0x11, 0x7F, 0xFF, 0x00, 0x00, 0x00, 30, 31};
		NodeFeedback fb;
		check(core.decode(fbframe, 8, fb, 1.0), "反馈 decode");
		auto sf2 = unistackbot_statemachine::createNodeSafety("dm4310");
		sf2->update(fb);
		check(sf2->state() == NeutralState::kEnabled, "decode 产物驱动状态机 (D0=0x11 → err=1 → Enabled)");
	}

	std::printf("%s: %d cases, %d fail\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

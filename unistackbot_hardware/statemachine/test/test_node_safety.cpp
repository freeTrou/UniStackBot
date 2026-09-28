/*
 * test_node_safety —— 单节点安全状态机测试 (2026-09-28, 四态保护模型)。
 *
 * 覆盖:
 *   ①四态推进: 反馈帧驱动状态 (OFFLINE→DE-ENERGIZED→ENERGIZED→FAULT)
 *   ②使能守卫: 锚定实测位 / OFFLINE 禁使能 / FAULT 禁使能
 *   ③断使能: 安全怠速命令 (mode=0 + 看门狗)
 *   ④故障: 清请求 / 带故障禁使能请求自动清除
 *   ⑤防跳变: 断使能期间位置变化 → 使能命令用最新实测位
 *
 * 编译运行:
 *   cd unistackbot_hardware/statemachine/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_node_safety.cpp \
 *       ../src/node_safety.cpp -I../include -I../../protocol/include \
 *       -o /tmp/test_safety && /tmp/test_safety
 */
#include <cmath>
#include <cstdio>

#include "unistackbot_statemachine/node_safety.hpp"

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::NodeSafety;
using NeutralState = unistackbot_statemachine::NeutralState;

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

NodeFeedback fbWith(NodeMode mode, double pos = 0.0, bool timeout = false,
	unsigned err = 0u)
{
	NodeFeedback fb;
	fb.mode = mode;
	fb.position = pos;
	fb.timeout_triggered = timeout;
	fb.error_raw = err;
	return fb;
}

}  // namespace

int main()
{
	// ── ① 四态推进 ──
	NodeSafety ns;

	check(ns.state() == NeutralState::kUnknown, "initial = OFFLINE");

	ns.update(fbWith(NodeMode::kStop));
	check(ns.state() == NeutralState::kReady, "mode=0 → DE-ENERGIZED");

	ns.update(fbWith(NodeMode::kRun));
	check(ns.state() == NeutralState::kEnabled, "mode=1 → ENERGIZED");

	ns.update(fbWith(NodeMode::kRun, 0.0, true));
	check(ns.state() == NeutralState::kQuickStop, "timeout → QUICK_STOP");

	ns.update(fbWith(NodeMode::kStop, 0.0, false, 0x10u));
	check(ns.state() == NeutralState::kFault, "error → FAULT");

	// 故障粘性: error 消失也不自动离开 (间歇故障防复撞)
	ns.update(fbWith(NodeMode::kStop));
	check(ns.state() == NeutralState::kFault, "FAULT sticky (error gone but stays)");

	// 正确恢复路径: request_clear → confirm → READY
	ns.request_fault_clear();
	check(ns.fault_clear_requested(), "fault clear requested");
	ns.confirm_fault_cleared();
	check(ns.state() == NeutralState::kReady, "confirmed clear → DE-ENERGIZED");

	// ── ② 使能守卫 ──
	NodeSafety ns2;
	ns2.update(fbWith(NodeMode::kStop, 1.5));   // DE-ENERGIZED, pos=1.5

	check(ns2.can_enable(), "DE-ENERGIZED can enable");
	ns2.request_enable();
	check(ns2.enable_requested(), "enable request accepted");

	// 使能命令: 锚定实测位
	NodeCommand ec = ns2.enable_command(20.0, 1.5);
	check(ec.mode == NodeMode::kRun, "enable cmd mode=RUN");
	check(std::fabs(ec.position - 1.5) < 1e-12, "enable cmd pos = measured (no jump)");
	check(std::fabs(ec.kp - 20.0) < 1e-12, "enable cmd kp = lock_kp");
	check(ec.watchdog_enable, "enable cmd watchdog on");

	// OFFLINE 禁使能
	NodeSafety ns3;
	check(!ns3.can_enable(), "OFFLINE cannot enable");
	ns3.request_enable();
	check(!ns3.enable_requested(), "OFFLINE enable request rejected");

	// FAULT 禁使能
	NodeSafety ns4;
	ns4.update(fbWith(NodeMode::kStop, 0.0, false, 0x10u));
	check(!ns4.can_enable(), "FAULT cannot enable");
	ns4.request_enable();
	check(!ns4.enable_requested(), "FAULT enable request rejected");

	// 带故障时已有的使能请求被自动清除
	NodeSafety ns5;
	ns5.update(fbWith(NodeMode::kStop));         // DE-ENERGIZED
	ns5.request_enable();
	check(ns5.enable_requested(), "enable requested before fault");
	ns5.update(fbWith(NodeMode::kRun, 0.0, false, 0x100u));  // 故障来了
	check(ns5.state() == NeutralState::kFault, "now in FAULT");
	check(!ns5.enable_requested(), "enable request auto-cleared on fault");

	// ── ③ 断使能命令 ──
	NodeSafety ns6;
	ns6.update(fbWith(NodeMode::kRun, 2.0));
	NodeCommand dc = ns6.disable_command();
	check(dc.mode == NodeMode::kStop, "disable cmd mode=STOP");
	check(dc.watchdog_enable, "disable cmd watchdog on");
	check(dc.kp == 0.0 && dc.kd == 0.0, "disable cmd zero gains");

	// ── ④ 故障清除 ──
	NodeSafety ns7;
	ns7.update(fbWith(NodeMode::kStop, 0.0, false, 0x100u));
	check(ns7.can_clear_fault(), "FAULT can clear");
	ns7.request_fault_clear();
	check(ns7.fault_clear_requested(), "fault clear requested");

	NodeSafety ns8;
	ns8.update(fbWith(NodeMode::kStop));   // DE-ENERGIZED
	check(!ns8.can_clear_fault(), "non-FAULT cannot clear");

	// ── ⑤ 防跳变: 断使能期间位置变化 ──
	NodeSafety ns9;

	// 断使能时位置 1.0
	ns9.update(fbWith(NodeMode::kStop, 1.0));

	// 断使能期间臂下垂到 1.3（重力）
	ns9.update(fbWith(NodeMode::kStop, 1.3));

	// 使能命令应该用 1.3（最新实测），不是 1.0（旧位置）也不是 0（零位）
	NodeCommand ec2 = ns9.enable_command(15.0, 1.0);
	check(std::fabs(ec2.position - 1.3) < 1e-12,
		"enable pos = latest measured (1.3), not old (1.0) or zero");

	// ── ⑥ 批量更新 (EC: 一次全推进) ──
	NodeSafety batch[3];
	NodeFeedback bfs[3];
	bfs[0] = fbWith(NodeMode::kStop, 0.1);
	bfs[1] = fbWith(NodeMode::kRun, 0.5);
	bfs[2] = fbWith(NodeMode::kStop, 0.0, false, 0x10u);
	NodeSafety::update_batch(batch, bfs, 3);
	check(batch[0].state() == NeutralState::kReady, "batch[0] READY");
	check(batch[1].state() == NeutralState::kEnabled, "batch[1] ENABLED");
	check(batch[2].state() == NeutralState::kFault, "batch[2] FAULT");

	// ── 请求清理 ──
	ns9.clear_requests();
	check(!ns9.enable_requested() && !ns9.disable_requested(), "requests cleared");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

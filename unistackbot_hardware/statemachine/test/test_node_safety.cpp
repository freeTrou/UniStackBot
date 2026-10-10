/*
 * test_node_safety —— 安全状态机测试 (2026-09-28 立测; 2026-10-10 随父类+子类+
 * 工厂重构改经工厂/基类指针——map_state 双机并存退役, 每协议一个映射一份测试)。
 *
 * 覆盖:
 *   ⓪工厂: 未知名 nullptr / 可选清单 / 基类指针多态
 *   ①四态推进: 反馈帧驱动状态 (OFFLINE→DE-ENERGIZED→ENERGIZED→QUICK_STOP→FAULT)
 *   ②使能守卫: 锚定实测位 + 协议先验 kp/kd / OFFLINE 禁使能 / FAULT 禁使能
 *   ③断使能: 安全怠速命令 (mode=0 + 看门狗)
 *   ④故障粘性 + 清握手 (request → confirm; 非 FAULT 请求无效)
 *   ⑤防跳变: 断使能期间位置变化 → 使能命令用最新实测位
 *
 * 编译运行:
 *   cd unistackbot_hardware/statemachine/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_node_safety.cpp \
 *       ../src/unitree_im_safety.cpp ../src/cia402_safety.cpp ../src/node_safety_factory.cpp \
 *       -I../include -I../../protocol/include \
 *       -o /tmp/test_safety && /tmp/test_safety
 */
#include <cmath>
#include <cstdio>

#include "unistackbot_statemachine/node_safety_factory.hpp"
#include "unistackbot_protocol/unitree_im_core.hpp"

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::NodeSafety;
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
	// ── ⓪ 工厂 ──
	check(unistackbot_statemachine::createNodeSafety("no_such") == nullptr,
		"unknown name -> nullptr (fail-fast)");
	const auto names = unistackbot_statemachine::availableNodeSafetyMachines();
	check(names.size() == 2, "registry has unitree_im + cia402");
	std::unique_ptr<NodeSafety> ns = unistackbot_statemachine::createNodeSafety("unitree_im");
	check(ns != nullptr, "factory creates unitree_im");
	if (!ns)
	{
		std::printf("%s: %d cases, %d failed\n", "FAIL", g_case, g_fail);
		return 1;
	}
	NodeSafety &s = *ns;   // 全程只握基类指针

	// ── ① 四态推进 ──
	check(s.state() == NeutralState::kUnknown, "initial = OFFLINE");

	s.update(fbWith(NodeMode::kStop));
	check(s.state() == NeutralState::kReady, "mode=0 → DE-ENERGIZED");

	s.update(fbWith(NodeMode::kRun));
	check(s.state() == NeutralState::kEnabled, "mode=1 → ENERGIZED");

	s.update(fbWith(NodeMode::kRun, 0.0, true));
	check(s.state() == NeutralState::kQuickStop, "timeout → QUICK_STOP");

	s.update(fbWith(NodeMode::kStop, 0.0, false, 0x10u));
	check(s.state() == NeutralState::kFault, "error → FAULT");

	// 故障粘性: error 消失也不自动离开 (间歇故障防复撞)
	s.update(fbWith(NodeMode::kStop));
	check(s.state() == NeutralState::kFault, "FAULT sticky (error gone but stays)");

	// 正确恢复路径: request_clear → confirm → READY
	s.request_fault_clear();
	check(s.fault_clear_requested(), "fault clear requested");
	s.confirm_fault_cleared();
	check(s.state() == NeutralState::kReady, "confirmed clear → DE-ENERGIZED");

	// ── ② 使能守卫 ──
	std::unique_ptr<NodeSafety> ns2 = unistackbot_statemachine::createNodeSafety("unitree_im");
	ns2->update(fbWith(NodeMode::kStop, 1.5));   // DE-ENERGIZED, pos=1.5

	check(ns2->can_enable(), "DE-ENERGIZED can enable");
	ns2->request_enable();
	check(ns2->enable_requested(), "enable request accepted");

	// 使能命令: 锚定实测位 + 协议先验 kp/kd (单一事实源在协议轴)
	NodeCommand ec = ns2->enable_command();
	check(ec.mode == NodeMode::kRun, "enable cmd mode=RUN");
	check(std::fabs(ec.position - 1.5) < 1e-12, "enable cmd pos = measured (no jump)");
	check(std::fabs(ec.kp - unistackbot_protocol::unitree_im::kPriorKp) < 1e-12,
		"enable cmd kp = prior");
	check(std::fabs(ec.kd - unistackbot_protocol::unitree_im::kPriorKd) < 1e-12,
		"enable cmd kd = prior");
	check(ec.watchdog_enable, "enable cmd watchdog on");

	// OFFLINE 禁使能
	std::unique_ptr<NodeSafety> ns3 = unistackbot_statemachine::createNodeSafety("unitree_im");
	check(!ns3->can_enable(), "OFFLINE cannot enable");
	ns3->request_enable();
	check(!ns3->enable_requested(), "OFFLINE enable request rejected");

	// FAULT 禁使能
	std::unique_ptr<NodeSafety> ns4 = unistackbot_statemachine::createNodeSafety("unitree_im");
	ns4->update(fbWith(NodeMode::kStop, 0.0, false, 0x10u));
	check(!ns4->can_enable(), "FAULT cannot enable");
	ns4->request_enable();
	check(!ns4->enable_requested(), "FAULT enable request rejected");

	// 带故障时已有的使能请求被自动清除
	std::unique_ptr<NodeSafety> ns5 = unistackbot_statemachine::createNodeSafety("unitree_im");
	ns5->update(fbWith(NodeMode::kStop));         // DE-ENERGIZED
	ns5->request_enable();
	check(ns5->enable_requested(), "enable requested before fault");
	ns5->update(fbWith(NodeMode::kRun, 0.0, false, 0x100u));  // 故障来了
	check(ns5->state() == NeutralState::kFault, "now in FAULT");
	check(!ns5->enable_requested(), "enable request auto-cleared on fault");

	// ── ③ 断使能命令 ──
	std::unique_ptr<NodeSafety> ns6 = unistackbot_statemachine::createNodeSafety("unitree_im");
	ns6->update(fbWith(NodeMode::kRun, 2.0));
	NodeCommand dc = ns6->disable_command();
	check(dc.mode == NodeMode::kStop, "disable cmd mode=STOP");
	check(dc.watchdog_enable, "disable cmd watchdog on");
	check(dc.kp == 0.0 && dc.kd == 0.0, "disable cmd zero gains");

	// ── ④ 故障清除 ──
	std::unique_ptr<NodeSafety> ns7 = unistackbot_statemachine::createNodeSafety("unitree_im");
	ns7->update(fbWith(NodeMode::kStop, 0.0, false, 0x100u));
	check(ns7->can_clear_fault(), "FAULT can clear");
	ns7->request_fault_clear();
	check(ns7->fault_clear_requested(), "fault clear requested");
	// 非 FAULT 态请求无效
	std::unique_ptr<NodeSafety> ns8 = unistackbot_statemachine::createNodeSafety("unitree_im");
	ns8->update(fbWith(NodeMode::kStop));   // DE-ENERGIZED
	check(!ns8->can_clear_fault(), "non-FAULT cannot clear");
	ns8->request_fault_clear();
	check(!ns8->fault_clear_requested(), "non-FAULT clear request rejected");
	// confirm 守卫: 非 FAULT 或 error 未清时无效
	ns7->update(fbWith(NodeMode::kStop, 0.0, false, 0x100u));   // 仍带故障
	ns7->confirm_fault_cleared();
	check(ns7->state() == NeutralState::kFault, "confirm ignored while error set");
	ns7->update(fbWith(NodeMode::kStop));                       // error 清了
	ns7->confirm_fault_cleared();
	check(ns7->state() == NeutralState::kReady, "confirm with clean fb → READY");
	check(!ns7->fault_clear_requested(), "confirm consumes request");

	// ── ⑤ 防跳变: 断使能期间位置变化 ──
	std::unique_ptr<NodeSafety> ns9 = unistackbot_statemachine::createNodeSafety("unitree_im");

	// 断使能时位置 1.0, 随后臂下垂到 1.3 (重力)
	ns9->update(fbWith(NodeMode::kStop, 1.0));
	ns9->update(fbWith(NodeMode::kStop, 1.3));

	// 使能命令应该用 1.3 (最新实测), 不是 1.0 (旧位置) 也不是 0 (零位)
	NodeCommand ec2 = ns9->enable_command();
	check(std::fabs(ec2.position - 1.3) < 1e-12,
		"enable pos = latest measured (1.3), not old (1.0) or zero");

	// ── 请求清理 ──
	ns9->clear_requests();
	check(!ns9->enable_requested() && !ns9->disable_requested(), "requests cleared");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

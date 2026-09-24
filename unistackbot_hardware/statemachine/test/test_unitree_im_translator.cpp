/*
 * test_unitree_im_translator —— 状态翻译器测试 (2026-09-24, 状态机轴拆包)。
 *
 * 翻译器消费统一 NodeFeedback 结构 (非帧), 测试直接构造结构体——与 codec 解耦。
 * 覆盖: 四态映射 + 优先级 (故障>超时>模式) / 工厂 (按名创建/未知拒绝/列表) /
 * 经基类指针多态消费 (骨架 health 将来就这么用)。
 *
 * 编译运行 (零 ROS, g++ 直编):
 *   cd unistackbot_hardware/statemachine/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_unitree_im_translator.cpp \
 *       ../src/unitree_im_translator.cpp ../src/state_translator_factory.cpp \
 *       -I../include -I../../protocol/include \
 *       -o /tmp/test_translator && /tmp/test_translator
 */
#include <cstdio>

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/state_translator_factory.hpp"
#include "unistackbot_statemachine/unitree_im_translator.hpp"

using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::NeutralState;
using unistackbot_statemachine::StateTranslator;
using unistackbot_statemachine::createStateTranslator;
using unistackbot_statemachine::availableStateTranslators;

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

NodeFeedback fbWith(NodeMode mode, bool timeout = false, unsigned err = 0u)
{
	NodeFeedback fb;
	fb.mode = mode;
	fb.timeout_triggered = timeout;
	fb.error_raw = err;
	return fb;
}

}  // namespace

int main()
{
	const auto tr = createStateTranslator("unitree_im");
	check(tr != nullptr, "factory creates unitree_im");
	check(createStateTranslator("nope") == nullptr, "unknown name -> null");
	const auto names = availableStateTranslators();
	check(names.size() == 1 && names[0] == "unitree_im", "registry lists");
	if (tr == nullptr)
	{
		std::printf("FAIL: no translator\n");
		return 1;
	}

	StateTranslator &poly = *tr;   // 全程只握基类指针 (骨架 health 的消费形态)

	// 四态 + 优先级: 故障 > 超时 > 模式
	check(poly.map_state(fbWith(NodeMode::kRun)) == NeutralState::kEnabled, "enabled");
	check(poly.map_state(fbWith(NodeMode::kStop)) == NeutralState::kReady, "ready");
	check(poly.map_state(fbWith(NodeMode::kStop, true)) == NeutralState::kQuickStop, "quickstop");
	check(poly.map_state(fbWith(NodeMode::kRun, true)) == NeutralState::kQuickStop, "timeout beats run");
	check(poly.map_state(fbWith(NodeMode::kStop, false, 0x10u)) == NeutralState::kFault, "fault");
	check(poly.map_state(fbWith(NodeMode::kRun, true, 0x10u)) == NeutralState::kFault, "fault beats all");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

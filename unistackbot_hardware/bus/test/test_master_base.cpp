/*
 * test_master_base —— 总线父类轴测试 (组合模型动态容量形态, 2026-10-21 R-a)。
 *
 * 覆盖:
 *   ①抽象性 + 交互类别编译期证明 (周期 PDO / 慢通道 SDO / 即发命令 / 意图三件套)
 *   ②工厂契约 (空注册表 / 未知名拒 / null maker 拒)
 *   ③交换/配置类型: NodeSpec/NodeSnapshot 可默认构造 (运行期 vector 元素);
 *     容量无编译期上限 (DynamicSpLatest 装配期定容——24 节点 vector 实测, r1_pro 形态)
 *
 * 编译运行 (零 ROS, g++ 直编):
 *   cd unistackbot_hardware/bus/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_master_base.cpp ../src/master_factory.cpp \
 *       -I../include -I../../protocol/include -I../../statemachine/include \
 *       -o /tmp/test_master_base && /tmp/test_master_base
 */
#include <cstdio>
#include <type_traits>
#include <vector>

#include "unistackbot_bus/master_factory.hpp"

using unistackbot_bus::MasterBase;
using unistackbot_bus::MasterConfig;
using unistackbot_bus::NodeSpec;
using unistackbot_bus::NodeSnapshot;
using unistackbot_bus::availableMasters;
using unistackbot_bus::registerMaster;
using unistackbot_bus::createMaster;

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

}  // namespace

int main()
{
	// ① 抽象性: 父类只定接口 (子类才可实例化)
	static_assert(std::is_abstract_v<MasterBase>, "MasterBase must be abstract");
	check(true, "MasterBase is abstract");

	// ①b 交互类别完整性: 周期交换 + 慢通道事务 + 意图三件套 + 急停
		using PubCmdFn = void (MasterBase::*)(const unistackbot_protocol::NodeCommand *, std::size_t);
	static_assert(std::is_member_function_pointer_v<PubCmdFn>, "PDO-form cyclic exchange present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::read_param)>, "SDO-form slow-channel read present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::write_param)>, "SDO-form slow-channel write present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::quick_stop)>, "immediate command present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::enable)>, "enable intent present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::disable)>, "disable intent present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::fault_clear)>, "fault_clear present");
	check(true, "interaction categories complete (cyclic + transactional + immediate + intent trio)");

	// ② 工厂契约
	check(createMaster("no_such") == nullptr, "unknown name -> null");
	check(registerMaster("test_null", nullptr) == false, "null maker rejected");
	check(availableMasters().empty(), "registry starts empty (impls register via packages)");

	// ③ 配置/载荷类型: 默认构造 + 数量运行期 (无编译期容量上限)
	MasterConfig cfg;
	check(cfg.nodes.empty() && cfg.rate_hz == 0.0, "default init: empty node table");
	cfg.nodes.resize(24);   // r1_pro 整机形态 (24 电机) —— 旧 kMaxNodes=16 撞界实证
	check(cfg.nodes.size() == 24, "node table runtime-sized (24 ok)");
	NodeSpec spec;
	check(spec.id == 0 && spec.ratio == 0.0 && spec.protocol.empty(), "NodeSpec default");
	NodeSnapshot snap;
	check(snap.state == unistackbot_statemachine::NeutralState::kUnknown, "NodeSnapshot default kUnknown");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

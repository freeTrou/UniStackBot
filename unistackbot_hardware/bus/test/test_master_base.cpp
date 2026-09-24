/*
 * test_master_base —— 总线父类轴测试 (2026-09-24 立轴)。
 *
 * 当前为接口定形测试 (无实现注册——SerialMaster 随切片1步4 落地):
 *   ①MasterBase 是抽象类 + 交互类别完整 (周期交换 PDO 形 / 慢通道事务 SDO 形 / 即发命令
 *     ——2026-09-24 用户纠偏补 SDO 类别后的编译期证明)
 *   ②工厂契约: 未知名字拒绝 + 跨包注册机制 (registerMaster 由实现包组合根调用)
 *   ③交换类型/配置结构完整可默认构造 (POD 定长数组, 零分配语义)
 *
 * 编译运行 (零 ROS, g++ 直编):
 *   cd unistackbot_hardware/bus/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_master_base.cpp \
 *       ../src/master_factory.cpp -I../include -I../../protocol/include -I../../statemachine/include \
 *       -o /tmp/test_master_base && /tmp/test_master_base
 */
#include <cstdio>
#include <type_traits>

#include "unistackbot_bus/master_factory.hpp"

using unistackbot_bus::BusCommand;
using unistackbot_bus::BusState;
using unistackbot_bus::MasterBase;
using unistackbot_bus::MasterConfig;
using unistackbot_bus::availableMasters;
using unistackbot_bus::registerMaster;
using unistackbot_bus::createMaster;
using unistackbot_bus::kMaxNodes;

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

	// ①b 交互类别完整性 (2026-09-24 用户纠偏后): 周期交换 + 慢通道事务两类都必须在父类
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::publish_cmd)>,
		"PDO-form cyclic exchange present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::read_param)>,
		"SDO-form slow-channel read present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::write_param)>,
		"SDO-form slow-channel write present");
	static_assert(std::is_member_function_pointer_v<decltype(&MasterBase::quick_stop)>,
		"immediate command present");
	check(true, "interaction categories complete (cyclic + transactional + immediate)");

	// ② 工厂契约: 未知名字拒绝 + 跨包注册机制 (组合根调用)
	check(createMaster("no_such") == nullptr, "unknown name -> null");
	check(registerMaster("test_null", nullptr) == false, "null maker rejected");
	check(availableMasters().empty(), "registry starts empty (impls register via packages)");

	// ③ 交换/配置类型: 默认构造 + 定长数组界
	check(kMaxNodes == 16, "kMaxNodes mirrors kMaxJoints=16");
	MasterConfig cfg;
	BusCommand cmd;
	BusState st;
	check(sizeof(cmd.node) / sizeof(cmd.node[0]) == kMaxNodes, "BusCommand fixed array");
	check(sizeof(st.node) / sizeof(st.node[0]) == kMaxNodes, "BusState fixed array");
	check(st.seq == 0 && cfg.rate_hz == 0.0 && cfg.node_count == 0, "default init zero");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

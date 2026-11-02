#include "base_kinematics/base_kinematics.hpp"

#include <cmath>
#include <map>

namespace unistackbot_algorithm
{
namespace
{

/*
 * diff 差速实现 2026-11-02 随 StationPro 分支产品化出清——A3 示范件, 零真实机型消费者
 * (E2E 走 diffbot 测试夹具; 住 dev/main 基座仓)。
 * 本分支底盘唯一活实现 = n_steer (P20 四舵轮, 三/四舵轮同码);
 * mos 4WS 机制重述 (ODE M3) 落地时在 ensureBuiltinKinematics 挂注册。
 */

// 注册表 (A 主轴组合根式: 懒初始化 + 公共注册函数; 加内置实现 = ensureBuiltin 加一行;
// 外部化挂账走同一扇门 registerBaseKinematics)
std::map<std::string, BaseKinematicsMaker> & registry()
{
	static std::map<std::string, BaseKinematicsMaker> reg;
	return reg;
}

}  // namespace

bool registerBaseKinematics(const std::string & name, BaseKinematicsMaker maker)
{
	return registry().emplace(name, maker).second;   // 重复名 false (组合根 bug fail-fast)
}

// 内置实现自注册 (首次工厂调用执行一次; 函数显式调用, 不用静态初始化器 —— dead-strip 防线)
static void ensureBuiltinKinematics()
{
	static bool done = false;
	if (done)
	{
		return;
	}
	done = true;
	registerNSteerKinematics();
}

std::unique_ptr<BaseKinematics> createBaseKinematics(const std::string & name)
{
	ensureBuiltinKinematics();
	const auto it = registry().find(name);
	return it == registry().end() ? nullptr : it->second();
}

std::vector<std::string> availableBaseKinematics()
{
	ensureBuiltinKinematics();
	std::vector<std::string> names;
	for (const auto & kv : registry())
	{
		names.push_back(kv.first);
	}
	return names;
}

}  // namespace unistackbot_algorithm

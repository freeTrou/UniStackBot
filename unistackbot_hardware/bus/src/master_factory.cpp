#include "unistackbot_bus/master_factory.hpp"

#include <map>

namespace unistackbot_bus
{
namespace
{

using Maker = std::unique_ptr<MasterBase> (*)();

// 可变注册表 —— 函数局部 static 规避初始化顺序问题。
// 注册经 registerMaster() 由各实现包的 registerToMasterFactory() 在组合根启动期调用
// (单线程上下文不加锁, 运行期注册属违规)。内置条目: 无 (全部来自实现包)。
std::map<std::string, Maker> &registry()
{
	static std::map<std::string, Maker> reg;
	return reg;
}

}  // namespace

bool registerMaster(const std::string &name, std::unique_ptr<MasterBase> (*maker)())
{
	if (maker == nullptr || registry().find(name) != registry().end())
	{
		return false;
	}
	registry()[name] = maker;
	return true;
}

namespace
{

}  // namespace

std::unique_ptr<MasterBase> createMaster(const std::string &name)
{
	const auto &reg = registry();
	const auto it = reg.find(name);
	if (it == reg.end())
	{
		return nullptr;
	}
	return it->second();
}

std::vector<std::string> availableMasters()
{
	std::vector<std::string> names;
	for (const auto &kv : registry())
	{
		names.push_back(kv.first);
	}
	return names;
}

}  // namespace unistackbot_bus

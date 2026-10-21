#include "unistackbot_statemachine/node_safety_factory.hpp"

#include <map>

#include "unistackbot_statemachine/unitree_im_safety.hpp"
#include "unistackbot_statemachine/cia402_safety.hpp"
#include "unistackbot_statemachine/dm_safety.hpp"

namespace unistackbot_statemachine
{
namespace
{

using Maker = std::unique_ptr<NodeSafety> (*)();

// 静态注册表 —— 函数局部 static 规避初始化顺序问题。
// **加新协议的安全机 = 此处加一行** (CiA402 / 影子机按加法纪律陆续进此)。
const std::map<std::string, Maker> & registry()
{
	static const std::map<std::string, Maker> reg = {
		{"unitree_im", []() -> std::unique_ptr<NodeSafety> {
			 return std::make_unique<UnitreeImSafety>();
		 }},
		{"cia402", []() -> std::unique_ptr<NodeSafety> {
			 return std::make_unique<Cia402Safety>();
		 }},   // CiA402 官方七态机 (EC/CANFD 伺服标准; 2026-10-10 第二个安全机)
		// 达妙 (2026-10-21 第三个): 码表全系列同, 型号无关; 两 key 与协议工厂同一把
		{"dm4310", []() -> std::unique_ptr<NodeSafety> { return std::make_unique<DmSafety>(); }},
		{"dm4340", []() -> std::unique_ptr<NodeSafety> { return std::make_unique<DmSafety>(); }},
	};
	return reg;
}

}  // namespace

std::unique_ptr<NodeSafety> createNodeSafety(const std::string & name)
{
	const auto & reg = registry();
	const auto it = reg.find(name);
	if (it == reg.end())
	{
		return nullptr;
	}
	return it->second();
}

std::vector<std::string> availableNodeSafetyMachines()
{
	std::vector<std::string> names;
	for (const auto & kv : registry())
	{
		names.push_back(kv.first);
	}
	return names;
}

}  // namespace unistackbot_statemachine

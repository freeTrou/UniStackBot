#include "unistackbot_protocol/protocol_factory.hpp"

#include <map>

#include "unistackbot_protocol/unitree_im_core.hpp"

namespace unistackbot_protocol
{
namespace
{

using Maker = std::unique_ptr<ProtocolCore> (*)();

// 静态注册表 —— 函数局部 static 规避初始化顺序问题。
// **加新协议 = 此处加一行** (对应 include + 一行注册; 详见 protocol_factory.hpp)。
const std::map<std::string, Maker> &registry()
{
	static const std::map<std::string, Maker> reg = {
		{"unitree_im", []() -> std::unique_ptr<ProtocolCore> { return std::make_unique<UnitreeImCore>(); }},
	};
	return reg;
}

}  // namespace

std::unique_ptr<ProtocolCore> createProtocolCore(const std::string &name)
{
	const auto &reg = registry();
	const auto it = reg.find(name);
	if (it == reg.end())
	{
		return nullptr;
	}
	return it->second();
}

std::vector<std::string> availableProtocolCores()
{
	std::vector<std::string> names;
	for (const auto &kv : registry())
	{
		names.push_back(kv.first);
	}
	return names;
}

}  // namespace unistackbot_protocol

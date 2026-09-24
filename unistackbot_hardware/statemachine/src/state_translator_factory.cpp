#include "unistackbot_statemachine/state_translator_factory.hpp"

#include <map>

#include "unistackbot_statemachine/unitree_im_translator.hpp"

namespace unistackbot_statemachine
{
namespace
{

using Maker = std::unique_ptr<StateTranslator> (*)();

// 静态注册表 —— 函数局部 static 规避初始化顺序问题。
// **加新翻译器 = 此处加一行** (CiA402 / 影子机 / 细分格版本按加法纪律陆续进此)。
const std::map<std::string, Maker> &registry()
{
	static const std::map<std::string, Maker> reg = {
		{"unitree_im", []() -> std::unique_ptr<StateTranslator> {
			 return std::make_unique<UnitreeImTranslator>();
		 }},
	};
	return reg;
}

}  // namespace

std::unique_ptr<StateTranslator> createStateTranslator(const std::string &name)
{
	const auto &reg = registry();
	const auto it = reg.find(name);
	if (it == reg.end())
	{
		return nullptr;
	}
	return it->second();
}

std::vector<std::string> availableStateTranslators()
{
	std::vector<std::string> names;
	for (const auto &kv : registry())
	{
		names.push_back(kv.first);
	}
	return names;
}

}  // namespace unistackbot_statemachine

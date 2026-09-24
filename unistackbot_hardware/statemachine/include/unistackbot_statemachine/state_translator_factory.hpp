#ifndef UNISTACKBOT_STATEMACHINE__STATE_TRANSLATOR_FACTORY_HPP_
#define UNISTACKBOT_STATEMACHINE__STATE_TRANSLATOR_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "unistackbot_statemachine/state_translator.hpp"

// 状态翻译器工厂 —— 按名字创建 (同 protocol_factory 组织)。
// 静态注册表 name→make(); **加一个翻译器 = src/state_translator_factory.cpp 注册表
// 加一行**。纪律 (无默认值): 未知名字返回 nullptr, 调用方 fail-fast 列可选项。

namespace unistackbot_statemachine
{

[[nodiscard]] std::unique_ptr<StateTranslator> createStateTranslator(const std::string &name);

[[nodiscard]] std::vector<std::string> availableStateTranslators();

}  // namespace unistackbot_statemachine

#endif

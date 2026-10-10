#ifndef UNISTACKBOT_STATEMACHINE__NODE_SAFETY_FACTORY_HPP_
#define UNISTACKBOT_STATEMACHINE__NODE_SAFETY_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "unistackbot_statemachine/node_safety.hpp"

// 安全状态机工厂 —— 按名字创建 (同 protocol_factory 组织)。
// 静态注册表 name→make(); **加一个协议的安全机 = src/node_safety_factory.cpp
// 注册表加一行** (CiA402 按加法纪律进此)。纪律 (无默认值): 未知名字返回
// nullptr, 调用方 (总线主站 start 期) fail-fast 列可选项。
// key 与协议/翻译器同一把 (如 "unitree_im") —— 一芯三面同一身份。

namespace unistackbot_statemachine
{

[[nodiscard]] std::unique_ptr<NodeSafety> createNodeSafety(const std::string & name);

[[nodiscard]] std::vector<std::string> availableNodeSafetyMachines();

}  // namespace unistackbot_statemachine

#endif  // UNISTACKBOT_STATEMACHINE__NODE_SAFETY_FACTORY_HPP_

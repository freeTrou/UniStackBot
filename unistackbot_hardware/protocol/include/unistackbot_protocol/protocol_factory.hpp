#ifndef UNISTACKBOT_PROTOCOL__PROTOCOL_FACTORY_HPP_
#define UNISTACKBOT_PROTOCOL__PROTOCOL_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "unistackbot_protocol/protocol_core.hpp"

// 协议工厂 —— 按名字创建协议内芯 (组合根的消费入口)。
//
// 设计 (架构文档 §2.1): 静态注册表 name→make()。**加一个协议 = src/protocol_factory.cpp
// 的注册表里加一行**——插件体验、可进 gdb、零动态加载边界 (SystemInterface 用 pluginlib
// 是 ros2_control 的要求, 内芯不需要)。
//
// 纪律 (无默认值): 未知名字返回 nullptr, 调用方 fail-fast 并列 availableProtocolCores()。
// 型号↔协议的对应永远由机器配置显式声明 (protocol: unitree_im), 永不自动推断。

namespace unistackbot_protocol
{

// 按协议名创建内芯; 未知名字返回 nullptr (调用方应打印可选项并拒绝启动)。
[[nodiscard]] std::unique_ptr<ProtocolCore> createProtocolCore(const std::string &name);

// 已注册的协议名列表 (fail-fast 提示用)。
[[nodiscard]] std::vector<std::string> availableProtocolCores();

}  // namespace unistackbot_protocol

#endif

#ifndef UNISTACKBOT_BUS__MASTER_FACTORY_HPP_
#define UNISTACKBOT_BUS__MASTER_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "unistackbot_bus/master_base.hpp"

// 总线主站工厂 —— 按名字创建 (同 protocol/statemachine 工厂组织)。
// 注册表 name→make()。**跨包注册**: 各总线实现包 (serial 等) 暴露自己的
// registerToMasterFactory(), 由组合根在 createMaster 之前调用——注册发生在启动期
// 单线程上下文, 注册表不加锁 (纪律: 运行期不可注册)。
// 纪律 (无默认值): 未知名字返回 nullptr, 调用方 fail-fast 列可选项。

namespace unistackbot_bus
{

// 注册一种总线 (实现包的组合根调用; 名字重复或 maker 空返回 false)
bool registerMaster(const std::string &name, std::unique_ptr<MasterBase> (*maker)());

[[nodiscard]] std::unique_ptr<MasterBase> createMaster(const std::string &name);

[[nodiscard]] std::vector<std::string> availableMasters();

}  // namespace unistackbot_bus

#endif

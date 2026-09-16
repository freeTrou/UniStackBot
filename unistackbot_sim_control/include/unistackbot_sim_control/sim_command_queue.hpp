#ifndef UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_QUEUE_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_QUEUE_HPP_

#include <cstddef>

#include "sp_ring/sp_ring.hpp"

#include "unistackbot_interface/sim_control_contract.hpp"

namespace unistackbot_sim_control
{

// 无锁队列本体已上收至 unistackbot_common/sp_ring (单一事实源), 这里保持原非限定用法
using unistackbot_common::SpscRing;

// 命令帧类型已上收至 unistackbot_interface (跨链契约, gz 适配器同源引用):
// SimCommand / SimCmdType / kMaxJoints —— 此别名保持本包既有引用不变
using unistackbot_interface::SimCommand;
using unistackbot_interface::SimCmdType;
using unistackbot_interface::SimControlServer;
using unistackbot_interface::kMaxJoints;

// 命令队列深度 (2 的幂); 生产者 = /sim_control 服务线程(互斥组串行) / 消费者 = RT read
inline constexpr size_t kQueueCapacity = 16;

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_QUEUE_HPP_

#ifndef UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_QUEUE_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_QUEUE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "sp_ring/sp_ring.hpp"

namespace unistackbot_sim_control
{

// 无锁队列本体已上收至 unistackbot_common/sp_ring (单一事实源), 这里保持原非限定用法
using unistackbot_common::SpscRing;

// 本包统一关节上限: 命令帧/状态帧容量与组件校验共用此唯一定义 (谁使用谁定义, 本包内唯一)
inline constexpr uint32_t kMaxJoints = 64;

// 命令队列深度 (2 的幂); 生产者 = /sim_control 服务线程(互斥组串行) / 消费者 = RT read
inline constexpr size_t kQueueCapacity = 16;

enum class SimCmdType : uint8_t
{
	RESET = 0,       // 全部非 mimic 关节回零, 速度清零
	SET_STATE = 1,   // 瞬移到指定状态 (mask 标记生效关节)
	PAUSE = 2,       // 冻结状态推进
	RESUME = 3,      // 恢复推进
	STEP = 4,        // 暂停状态下推进一步
};

// 定长 POD 命令 —— 无锁队列要求可平凡拷贝 (队列本体 sp_ring.hpp 处另有元素级断言)。
// mask[i]=1 表示关节 i 生效 (由服务端校验器按关节索引填好, mimic 关节不会出现)
struct SimCommand
{
	SimCmdType type{SimCmdType::RESET};
	uint32_t count{0};    // 关节总数快照 (圈定索引范围, 生效与否看 mask)
	std::array<uint8_t, kMaxJoints> mask{};
	std::array<double, kMaxJoints> positions{};
	std::array<double, kMaxJoints> velocities{};
};
static_assert(std::is_trivially_copyable_v<SimCommand>, "SimCommand must be trivially copyable for the lock-free queue");

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_QUEUE_HPP_

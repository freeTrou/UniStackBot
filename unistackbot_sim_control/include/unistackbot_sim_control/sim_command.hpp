#ifndef UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace unistackbot_sim_control
{

/// 契约常量: 单条命令支持的最大关节数
inline constexpr uint32_t kMaxJoints = 16;

/// 无锁队列默认深度
inline constexpr size_t kQueueCapacity = 16;

enum class SimCmdType : uint8_t
{
	RESET = 0,       ///< 全部非 mimic 关节回零, 速度清零
	SET_STATE = 1,   ///< 瞬移到指定状态 (mask 标记生效关节)
	PAUSE = 2,       ///< 冻结状态推进
	RESUME = 3,      ///< 恢复推进
	STEP = 4,        ///< 暂停状态下推进一步
};

/// 定长 POD 命令 —— 无锁队列要求可平凡拷贝。
/// mask[i]=1 表示关节 i 生效 (由服务端校验器按关节索引填好, mimic 关节不会出现)
struct SimCommand
{
	SimCmdType type{SimCmdType::RESET};
	uint32_t count{0};
	std::array<uint8_t, kMaxJoints> mask{};
	std::array<double, kMaxJoints> positions{};
	std::array<double, kMaxJoints> velocities{};
};
static_assert(std::is_trivially_copyable_v<SimCommand>,
	"SimCommand must be trivially copyable for the lock-free queue");

/// SPSC 无锁环形队列。
/// 生产者 = /sim_control/* 服务回调线程 (四个服务挂同一个
///           MutuallyExclusiveCallbackGroup, 天然串行 -> 严格单生产者)
/// 消费者 = controller_manager 实时循环线程 (read() 开头排空)
/// 索引单调递增 (size_t 自然回绕 + 2 的幂容量, 取模恒正确),
/// release/acquire 配对发布数据, 游标按 64B 对齐防伪共享
template <typename T, size_t Capacity>
class SpscRing
{
	static_assert(Capacity != 0 && (Capacity & (Capacity - 1)) == 0,
		"Capacity must be a power of 2");

public:
	/// 生产者专用; 队列满返回 false
	bool push(const T & value)
	{
		const size_t tail = tail_.load(std::memory_order_relaxed);
		if (tail - head_.load(std::memory_order_acquire) == Capacity) {
			return false;
		}
		buf_[tail % Capacity] = value;
		tail_.store(tail + 1, std::memory_order_release);
		return true;
	}

	/// 消费者专用; 队列空返回 false
	bool pop(T & value)
	{
		const size_t head = head_.load(std::memory_order_relaxed);
		if (head == tail_.load(std::memory_order_acquire)) {
			return false;
		}
		value = buf_[head % Capacity];
		head_.store(head + 1, std::memory_order_release);
		return true;
	}

private:
	std::array<T, Capacity> buf_{};
	alignas(64) std::atomic<size_t> tail_{0};
	alignas(64) std::atomic<size_t> head_{0};
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_COMMAND_HPP_

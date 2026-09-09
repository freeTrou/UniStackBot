#ifndef UNISTACKBOT_COMMON__SP_RING_HPP_
#define UNISTACKBOT_COMMON__SP_RING_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace unistackbot_common
{

/*
 * SPSC 无锁环形队列 (事件通道): FIFO, 逐条必达, 满则拒新 (try 语义)。
 * 家族分工: 最新 1 个 → SpLatest; 逐条必达 → 本组件。
 *
 * 契约:
 *   单写者(push) / 单读者(pop), 由使用架构保证 (如多服务挂同一互斥组)
 *   满返回 false (重试/丢弃语义归调用方); 空返回 false —— 全接口无阻塞, RT 热路径安全
 *   元素: 可平凡拷贝、不含指针 (static_assert); 容量: 2 的幂 (static_assert)
 *   游标单调递增: 无符号自然回绕 + 2 的幂容量, 取模恒正确 (32 位平台同样成立, 无需保护)
 *   head/tail 均单写者(纯 store 发布), release/acquire 配对逐行注释; 游标 alignas(64) 防伪共享
 *   越界策略说明: 丢旧保新语义在家族兄弟 ring_log 中, 本组件不含 —— 策略即类型, 语义即组件
 */
template <typename T, size_t Capacity>
class SpscRing
{
	static_assert(Capacity != 0 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");
	static_assert(std::is_trivially_copyable_v<T>, "SpscRing requires a trivially copyable POD type");

public:
	// 生产者专用; 满返回 false —— 失败显式透传 (重试/丢弃语义归调用方)
	[[nodiscard]] bool push(const T & value)
	{
		const size_t tail = tail_.load(std::memory_order_relaxed);   // 自写自读, 无需同步
		if (tail - head_.load(std::memory_order_acquire) == Capacity)   // 与消费者 release 配对: 确认槽位已取走才可复用
		{
			return false;
		}
		buffer_[tail % Capacity] = value;
		tail_.store(tail + 1, std::memory_order_release);   // 发布新元素: 此前的写入对 acquire 侧全部可见
		return true;
	}

	// 消费者专用; 空返回 false ("排空丢弃"是合法用法, 不标 nodiscard)
	bool pop(T & value)
	{
		const size_t head = head_.load(std::memory_order_acquire);   // 与生产者 release 配对: 元素及其写入已完整可见
		if (head == tail_.load(std::memory_order_acquire))
		{
			return false;
		}
		value = buffer_[head % Capacity];
		head_.store(head + 1, std::memory_order_release);   // 发布槽位释放, 生产者可见后才复用该槽
		return true;
	}

	// 观测: 当前堆积数 (仅诊断, 不进控制判断); 常驻 > 容量一半 = 消费跟不上的预警线
	[[nodiscard]] size_t size() const
	{
		return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
	}

private:
	std::array<T, Capacity> buffer_{};         // 槽位存储 (容量 2 的幂, 单调游标取模恒正确)
	alignas(64) std::atomic<size_t> tail_{0};  // 生产者游标 (单写者: 生产线程)
	alignas(64) std::atomic<size_t> head_{0};  // 消费者游标 (单写者: 消费线程)
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__SP_RING_HPP_

// SpscRing 功能测试 + 并发完整性压测 (规范 42 交付门槛)。
// 独立编译, 零 ROS 依赖 (include 根 = unistackbot_common):
//   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_sp_ring.cpp -o test_sp_ring && ./test_sp_ring
// 核心断言: 百万级并发推拉 **不丢、不重、不乱序** —— FIFO 完整性。

#include "sp_ring/sp_ring.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace
{

using unistackbot_common::SpscRing;

constexpr uint64_t kStressItems = 1000000;   // 并发压测条数 (容量 16, 游标回绕数万次)

int g_failures = 0;

void check(bool ok, const char * name)
{
	std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
	if (!ok)
	{
		++g_failures;
	}
}

void functional_tests()
{
	// 空队列: pop 返回 false
	SpscRing<uint64_t, 4> ring;
	uint64_t v = 0;
	check(ring.pop(v) == false, "empty: pop=false");

	// 满: 容量 4, 第 5 条拒
	for (uint64_t i = 1; i <= 4; ++i)
	{
		if (!ring.push(i))
		{
			check(false, "fill: push failed early");
			return;
		}
	}
	check(ring.push(5) == false, "full: 5th push=false");

	// FIFO: 1,2,3,4 依序弹出, 之后空
	bool fifo_ok = true;
	for (uint64_t expect = 1; expect <= 4; ++expect)
	{
		if (!ring.pop(v) || v != expect)
		{
			fifo_ok = false;
			break;
		}
	}
	check(fifo_ok, "fifo: strict order");
	check(ring.pop(v) == false, "drained: pop=false");

	// 容量 1 边界
	SpscRing<uint64_t, 1> tiny;
	check(tiny.push(7) == true && tiny.push(8) == false, "cap1: push/push=false");
	check(tiny.pop(v) == true && v == 7, "cap1: pop ok");
	check(tiny.pop(v) == false, "cap1: drained");
}

void stress_test()
{
	SpscRing<uint64_t, 16> ring;
	std::atomic<bool> producer_done{false};
	std::atomic<uint64_t> lost{0};      // 乱序/重复/丢失 —— 硬失败
	std::atomic<uint64_t> received{0};
	std::atomic<uint64_t> empty_hits{0};   // 消费者扑空次数 (生产者慢于消费者的正常现象)
	std::atomic<uint64_t> full_retries{0}; // 生产者遇满重试次数 (消费者慢于生产者的正常现象)

	// 生产者: 推 1..kStressItems, 满则忙等重试 (不丢 —— 这是"事件通道"语义)
	std::thread producer([&]()
	{
		for (uint64_t i = 1; i <= kStressItems;)
		{
			if (ring.push(i))
			{
				++i;
			}
			else
			{
				full_retries.fetch_add(1, std::memory_order_relaxed);
			}
		}
		producer_done.store(true, std::memory_order_release);
	});

	// 消费者: 收满 kStressItems 条, 校验严格递增 +1 (不丢不重不乱序)
	std::thread consumer([&]()
	{
		uint64_t expect = 1;
		uint64_t v = 0;
		while (expect <= kStressItems)
		{
			if (ring.pop(v))
			{
				if (v != expect)
				{
					lost.fetch_add(1, std::memory_order_relaxed);
				}
				++expect;
				received.fetch_add(1, std::memory_order_relaxed);
			}
			else
			{
				empty_hits.fetch_add(1, std::memory_order_relaxed);
			}
		}
	});

	producer.join();
	consumer.join();

	// 收尾: 收满且队列恰空 (不多不少 = 无幽灵元素)
	uint64_t v = 0;
	check(received.load() == kStressItems && ring.pop(v) == false, "stress: exact 1M received, ring empty");
	check(lost.load() == 0, "stress: zero lost/dup/reorder");
	std::printf("INFO: received=%llu empty_hits=%llu full_retries=%llu\n",
		static_cast<unsigned long long>(received.load()),
		static_cast<unsigned long long>(empty_hits.load()),
		static_cast<unsigned long long>(full_retries.load()));
}

}  // namespace

int main()
{
	functional_tests();
	stress_test();
	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

// MpscRing 功能测试 + 逐出风暴 + 消费者停摆自援 (规范 42 交付门槛)。
// 独立编译, 零 ROS 依赖 (include 根 = unistackbot_common):
//   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_mpsc_ring.cpp -o test_mpsc_ring && ./test_mpsc_ring
// 核心断言: 覆盖式多写单读 —— 收到的子序列严格递增、无重复、无撕裂;
// 账目守恒: 收到 + 逐出 + 残留 == 推入; 消费者死亡不挂死生产者。

#include "mpsc_ring/mpsc_ring.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

namespace
{

using unistackbot_common::cpu_relax;
using unistackbot_common::MpscRing;

// 自校验负载: 所有字段 == stamp —— 任何撕裂/半新半旧立刻现形
struct Payload
{
	uint64_t stamp{0};
	uint64_t a{0};
	uint64_t b{0};
	uint64_t c{0};
};
static_assert(std::is_trivially_copyable_v<Payload>, "test payload must be trivially copyable");

bool consistent(const Payload & p)
{
	return p.stamp == p.a && p.a == p.b && p.b == p.c;
}

constexpr uint64_t kStressItems = 2500000;   // 每生产者条数 (4 生产者共 10M)
constexpr int kProducers = 4;
constexpr uint64_t kPidStride = 10000000;   // stamp = pid*kPidStride + 本地序 (pid∈[0,4), 本地序<2.5M)

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
	// 空: pop=false
	MpscRing<Payload, 4> ring;
	Payload v;
	check(ring.pop(v) == false, "empty: pop=false");
	check(ring.size() == 0, "empty: size==0");

	// 逐出语义: 填 1..4 再推 5,6,7 → 剩 4,5,6,7 (1,2,3 被逐出)
	for (uint64_t i = 1; i <= 7; ++i)
	{
		ring.push(Payload{i, i, i, i});
	}
	bool ok = true;
	for (uint64_t expect = 4; expect <= 7; ++expect)
	{
		if (!ring.pop(v) || v.stamp != expect || !consistent(v))
		{
			ok = false;
			break;
		}
	}
	check(ok, "evict: newest-N FIFO (4..7 survive)");
	check(ring.pop(v) == false, "drained: pop=false");
	check(ring.evicted() == 3, "evict: evicted()==3");

	// 容量 1 (SpLatest 退化形): 只活最新
	MpscRing<Payload, 1> tiny;
	tiny.push(Payload{7, 7, 7, 7});
	tiny.push(Payload{8, 8, 8, 8});
	tiny.push(Payload{9, 9, 9, 9});
	check(tiny.pop(v) == true && v.stamp == 9 && consistent(v), "cap1: only newest survives");
	check(tiny.pop(v) == false, "cap1: drained");
}

void stress_test()
{
	MpscRing<Payload, 16> ring;   // 容量 16 vs 1M 条 → 绝大多数被逐出
	std::atomic<bool> producers_done{false};
	std::atomic<uint64_t> received{0};
	std::atomic<uint64_t> bad{0};   // 乱序(同pid回退) / 重复 / 撕裂 —— 硬失败

	// 断言设计注记: stamp = pid*stride + 本地序 —— 全局 fetch_add 的 stamp 与 push 占位序
	// 是两次独立原子 (取号后可被抢占, 别人先 push), 跨 pid 无全序可言;
	// 只断言"每 pid 内部严格递增(跳号=被逐出)" + 全局 stamp 去重, 配合账目守恒覆盖
	// 重复/乱序/撕裂/丢失四类
	std::vector<std::atomic<uint64_t>> per_pid_prev(kProducers);
	for (auto & x : per_pid_prev)
	{
		x.store(0, std::memory_order_relaxed);
	}
	std::vector<bool> seen(kProducers * kPidStride, false);   // 索引空间 = stamp 编码空间 (pid*stride+local)

	std::thread consumer([&]()
	{
		// 公共验证 (评审2: 主循环与终排空走同一套检查 —— 残留数据同样要过单调/去重/撕裂)
		auto verify = [&](const Payload & v)
		{
			const uint64_t pid = v.stamp / kPidStride;
			const uint64_t local = v.stamp % kPidStride;
			bool ok = consistent(v) && pid < static_cast<uint64_t>(kProducers);
			if (ok)
			{
				ok = local > per_pid_prev[pid].load(std::memory_order_relaxed);
			}
			if (ok && seen[v.stamp])
			{
				ok = false;   // 重复
			}
			if (!ok)
			{
				bad.fetch_add(1, std::memory_order_relaxed);
			}
			else
			{
				seen[v.stamp] = true;
				per_pid_prev[pid].store(local, std::memory_order_relaxed);
			}
			received.fetch_add(1, std::memory_order_relaxed);
		};
		Payload v;
		for (;;)
		{
			if (ring.pop(v))
			{
				verify(v);
			}
			else if (producers_done.load(std::memory_order_acquire))
			{
				while (ring.pop(v))   // 终排空 (false 可能是前缀在途写入)
				{
					verify(v);
				}
				break;
			}
			else
			{
				cpu_relax();   // 空转让位提示 (评审7; 不用 sleep/yield —— 那会降低交错密度, 与压测目的相反)
			}
		}
	});

	std::thread producers[kProducers];
	for (uint64_t p = 0; p < static_cast<uint64_t>(kProducers); ++p)
	{
		producers[p] = std::thread([p, &ring]()   // p 按值捕获: 引用捕获循环变量 = 线程读到递增后的错值
		{
			for (uint64_t i = 1; i <= kStressItems; ++i)
			{
				const uint64_t s = p * kPidStride + i;
				ring.push(Payload{s, s, s, s});
			}
		});
	}
	for (int p = 0; p < kProducers; ++p)
	{
		producers[p].join();
	}
	producers_done.store(true, std::memory_order_release);
	consumer.join();

	const uint64_t pushed = kProducers * kStressItems;
	const uint64_t remain = ring.size();
	// 账目守恒: 收到 + 逐出 + 残留 == 推入 (无幽灵/无凭空)
	check(received.load() + ring.evicted() + remain == pushed, "stress: accounting (recv+evict+remain == pushed)");
	check(bad.load() == 0, "stress: per-pid monotonic, no dup/tear");
	std::printf("INFO: pushed=%llu received=%llu evicted=%llu remain=%llu\n",
		static_cast<unsigned long long>(pushed),
		static_cast<unsigned long long>(received.load()),
		static_cast<unsigned long long>(ring.evicted()),
		static_cast<unsigned long long>(remain));
}

// 消费者停摆自援: 消费者睡 200ms, 生产者照常推 5 万条 (小环) → 生产者有界完成, 不挂死
// 消费者停摆自援 + 恢复并发消费 (评审1: 恢复后必须与生产者交错 pop, 而非事后排空)
void stall_selfaid_test()
{
	MpscRing<Payload, 8> ring;
	std::atomic<bool> producers_done{false};
	std::atomic<uint64_t> received{0};
	std::atomic<uint64_t> bad{0};
	std::atomic<uint64_t> last_seen{0};

	// 消费者: 停摆 100ms → 苏醒后立即并发消费 (与生产者交错) → 生产者完成后排空
	std::thread consumer([&]()
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));   // 故意停摆
		Payload v;
		uint64_t prev = 0;
		for (;;)
		{
			if (ring.pop(v))
			{
				if (v.stamp <= prev || !consistent(v))
				{
					bad.fetch_add(1, std::memory_order_relaxed);
				}
				prev = v.stamp;
				last_seen.store(prev, std::memory_order_relaxed);
				received.fetch_add(1, std::memory_order_relaxed);
			}
			else if (producers_done.load(std::memory_order_acquire))
			{
				while (ring.pop(v))
				{
					if (v.stamp <= prev || !consistent(v))
					{
						bad.fetch_add(1, std::memory_order_relaxed);
					}
					prev = v.stamp;
					last_seen.store(prev, std::memory_order_relaxed);
					received.fetch_add(1, std::memory_order_relaxed);
				}
				break;
			}
			else
			{
				cpu_relax();
			}
		}
	});

	const auto t0 = std::chrono::steady_clock::now();
	std::thread producer([&]()
	{
		for (uint64_t i = 1; i <= 50000; ++i)
		{
			ring.push(Payload{i, i, i, i});
			if (i % 500 == 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));   // 节拍: 让生产期跨越停摆苏醒, 并发交错窗口真正打开
			}
		}
	});
	producer.join();
	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	producers_done.store(true, std::memory_order_release);
	consumer.join();

	// <5s 是防死锁哨兵, 不是性能指标 (50k 自援在 ARM 亦为亚秒级, 5s 为弱平台留裕度)
	check(elapsed.count() < 5000, "stall: bounded push completion (<5s, deadlock sentinel)");
	check(bad.load() == 0, "stall: concurrent resume consumption consistent (mono + no tear)");
	check(last_seen.load() == 50000, "stall: newest record arrives after resume");
	check(received.load() + ring.evicted() == 50000, "stall: accounting (recv+evict == pushed)");
	std::printf("INFO: 50k pushes during 200ms stall took %lldms, received=%llu evicted=%llu\n",
		static_cast<long long>(elapsed.count()),
		static_cast<unsigned long long>(received.load()),
		static_cast<unsigned long long>(ring.evicted()));
}

// 容量 1 高并发 (评审3/9: 单槽 = 逐出与消费竞争最密集形态, 状态机最易错场景)
void cap1_stress_test()
{
	MpscRing<Payload, 1> ring;
	constexpr int kP1Producers = 2;
	constexpr uint64_t kP1Items = 50000;
	std::atomic<bool> producers_done{false};
	std::atomic<uint64_t> received{0};
	std::atomic<uint64_t> bad{0};
	uint64_t per_pid_prev[kP1Producers] = {0, 0};

	std::thread consumer([&]()
	{
		Payload v;
		for (;;)
		{
			if (ring.pop(v))
			{
				const uint64_t pid = v.stamp / kPidStride;
				const uint64_t local = v.stamp % kPidStride;
				if (!consistent(v) || pid >= static_cast<uint64_t>(kP1Producers) ||
					local <= per_pid_prev[pid])   // 单读者: 数组无竞争, 裸访问
				{
					bad.fetch_add(1, std::memory_order_relaxed);
				}
				else
				{
					per_pid_prev[pid] = local;
				}
				received.fetch_add(1, std::memory_order_relaxed);
			}
			else if (producers_done.load(std::memory_order_acquire))
			{
				while (ring.pop(v))
				{
					received.fetch_add(1, std::memory_order_relaxed);
					if (!consistent(v))
					{
						bad.fetch_add(1, std::memory_order_relaxed);
					}
				}
				break;
			}
			else
			{
				cpu_relax();
			}
		}
	});

	const auto t0 = std::chrono::steady_clock::now();
	std::thread producers[kP1Producers];
	for (uint64_t p = 0; p < static_cast<uint64_t>(kP1Producers); ++p)
	{
		producers[p] = std::thread([p, &ring]()   // p 按值捕获 (见 stress 同款坑)
		{
			for (uint64_t i = 1; i <= kP1Items; ++i)
			{
				const uint64_t s = p * kPidStride + i;
				ring.push(Payload{s, s, s, s});
			}
		});
	}
	for (int p = 0; p < kP1Producers; ++p)
	{
		producers[p].join();
	}
	const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	producers_done.store(true, std::memory_order_release);
	consumer.join();

	constexpr uint64_t pushed = kP1Producers * kP1Items;
	check(elapsed.count() < 30000, "cap1 stress: bounded (<30s, deadlock sentinel)");
	check(bad.load() == 0, "cap1 stress: per-pid monotonic, no tear @N=1");
	check(received.load() + ring.evicted() + ring.size() == pushed, "cap1 stress: accounting holds @N=1");
	std::printf("INFO: N=1 pushed=%llu received=%llu evicted=%llu took %lldms\n",
		static_cast<unsigned long long>(pushed),
		static_cast<unsigned long long>(received.load()),
		static_cast<unsigned long long>(ring.evicted()),
		static_cast<long long>(elapsed.count()));
}

}  // namespace

int main()
{
	functional_tests();
	stress_test();
	stall_selfaid_test();
	cap1_stress_test();
	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

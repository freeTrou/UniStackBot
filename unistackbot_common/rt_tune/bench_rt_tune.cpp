/*
 * bench_rt_tune —— rt_tune 组件测试/基准 (2026-09-20)。
 *
 * 双线程各钉核1/核2, 优先级各模拟真实档位 (调度策略 SCHED_FIFO):
 *   核1 = prio 80 (CM update 档) / 核2 = prio 90 (总线主站 IgH 档) — 对应 yaml 双核布局。
 * 周期唤醒自统计唤醒延迟, 与 test/rt_chain_bench.sh 的 cyclictest 互为对照——
 * 验证"我们自己的 RT 线程写法"在隔离核上的延迟分布。
 *
 * 断言 (组件测试语义): rt_tune::apply 绑核 + FIFO 必须全部成功 (本机 ulimit -r 99),
 * 任一失败退出码 1 并标 FAIL。
 *
 * 用法: bench_rt_tune <period_us> <duration_s>
 * 编译 (组件目录内, 同 ulog 范本):
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic bench_rt_tune.cpp -o /tmp/bench_rt_tune -pthread
 */
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>
#include <thread>
#include <vector>

#include "rt_tune.hpp"

namespace
{

constexpr int kCpuA = 1;
constexpr int kCpuB = 2;
constexpr int kPrioA = 80;  // 核1: CM update 档 (与 controllers yaml thread_priority 一致)
constexpr int kPrioB = 90;  // 核2: 总线主站档 (IgH EtherCAT, yaml 注释预留)
constexpr long long kWarmupIters = 100;          // 预热拍: 缺页/冷cache/分支预测 不入统计 (③)
constexpr size_t kStackPadBytes = 256u * 1024u;  // 栈预热深度: 触碰深栈页让缺页发生在此 (②)

struct Stats
{
	long long n = 0;
	double min_us = 1.0e18;
	double max_us = 0.0;
	double sum_us = 0.0;
	long long over_10 = 0;
	long long over_50 = 0;
	long long over_100 = 0;
	std::vector<double> all_us;
};

struct ThreadArg
{
	int cpu = 0;
	int period_us = 1000;
	int prio = 80;
	long long iters = 0;
	int tune_rc = -1;
	Stats st;
};

double clockNowNs()
{
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<double>(ts.tv_sec) * 1.0e9 + static_cast<double>(ts.tv_nsec);
}

timespec toTimespec(double ns)
{
	timespec ts{};
	ts.tv_sec = static_cast<time_t>(ns / 1.0e9);
	ts.tv_nsec = static_cast<long>(ns - static_cast<double>(ts.tv_sec) * 1.0e9);
	return ts;
}

double percentileUs(std::vector<double> &v, double p)
{
	if (v.empty())
	{
		return 0.0;
	}
	std::sort(v.begin(), v.end());
	const size_t idx = static_cast<size_t>(p * static_cast<double>(v.size() - 1));
	return v[idx];
}

void benchMain(ThreadArg *a)
{
	// ---- 栈预热 (②): mlockall 只锁驻留策略不预分配匿名页, 深栈页缺页必须发生在
	//      这里 (volatile 触碰每页首字节), 而不是测量拍内 ----
	volatile unsigned char pad[kStackPadBytes];
	for (size_t i = 0; i < kStackPadBytes; i += 4096)
	{
		pad[i] = 0;
	}
	asm volatile("" ::"r"(pad) : "memory");  // 逃逸屏障: 触碰不可被优化掉

	char name[16];
	std::snprintf(name, sizeof(name), "rt_bench_%d", a->cpu);
	a->tune_rc = unistackbot_common::rt_tune::apply(a->cpu, a->prio, 0, name);

	const double period_ns = static_cast<double>(a->period_us) * 1.0e3;
	double target = clockNowNs() + period_ns;
	
	for (long long i = 0; i < kWarmupIters + a->iters; ++i)
	{
		const timespec ts = toTimespec(target);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
		const double lat_us = (clockNowNs() - target) / 1.0e3;
		target += period_ns;

		if (i < kWarmupIters)
		{
			continue;  // 预热拍不入统计 (③)
		}

		Stats &s = a->st;  // all_us 已在 main 侧 resize+populate, 此处只下标写, 零分配 (①)
		const size_t k = static_cast<size_t>(i - kWarmupIters);
		++s.n;
		s.sum_us += lat_us;
		if (lat_us < s.min_us)
		{
			s.min_us = lat_us;
		}
		if (lat_us > s.max_us)
		{
			s.max_us = lat_us;
		}
		if (lat_us > 10.0)
		{
			++s.over_10;
		}
		if (lat_us > 50.0)
		{
			++s.over_50;
		}
		if (lat_us > 100.0)
		{
			++s.over_100;
		}
		s.all_us[k] = lat_us;
	}
}

}  // namespace

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		std::fprintf(stderr, "用法: %s <period_us> <duration_s>\n", argv[0]);
		return 1;
	}
	const int period_us = std::atoi(argv[1]);
	const int duration_s = std::atoi(argv[2]);
	if (period_us <= 0 || duration_s <= 0)
	{
		std::fprintf(stderr, "参数须为正整数\n");
		return 1;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
	{
		std::fprintf(stderr, "WARN: mlockall 失败 (%s) — 继续未锁定\n", std::strerror(errno));
	}

	ThreadArg ta;
	ThreadArg tb;
	ta.cpu = kCpuA;
	tb.cpu = kCpuB;
	ta.period_us = period_us;
	tb.period_us = period_us;
	ta.prio = kPrioA;
	tb.prio = kPrioB;
	ta.iters = static_cast<long long>(duration_s) * 1000000LL / period_us;
	tb.iters = ta.iters;

	// ---- 堆预分配 + populate (①②): resize 触发分配并 value-init 写零, 分配与缺页
	//      全部留在非 RT 上下文 (main); 测量线程内只剩下标写 ----
	ta.st.all_us.resize(static_cast<size_t>(ta.iters));
	tb.st.all_us.resize(static_cast<size_t>(tb.iters));

	std::thread tha(benchMain, &ta);
	std::thread thb(benchMain, &tb);
	tha.join();
	thb.join();

	int fail = 0;
	for (const ThreadArg *a : {&ta, &tb})
	{
		const Stats &s = a->st;
		std::vector<double> v = s.all_us;
		std::printf(
			"cpu%d: n=%lld min=%.1f avg=%.1f p50=%.1f p99=%.1f max=%.1f | >10µs=%lld >50µs=%lld >100µs=%lld\n",
			a->cpu, s.n, s.min_us, s.sum_us / static_cast<double>(s.n),
			percentileUs(v, 0.50), percentileUs(v, 0.99), s.max_us,
			s.over_10, s.over_50, s.over_100);
		if (a->tune_rc != unistackbot_common::rt_tune::kOk)
		{
			std::printf("FAIL: cpu%d rt_tune::apply rc=%d (期望 0 = 绑核+FIFO 双成功)\n", a->cpu, a->tune_rc);
			++fail;
		}
	}
	if (fail != 0)
	{
		return 1;
	}
	std::printf("PASS: rt_tune 组件断言绿 (核1 SCHED_FIFO prio=%d + 核2 prio=%d, 绑核生效)\n", kPrioA, kPrioB);
	return 0;
}

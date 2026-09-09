// ulog 吞吐基准 (手动跑, 非回归测试): 后端饱和点阶梯 + 全速过载行为。
// 独立编译 (include 根 = unistackbot_common):
//   g++ -std=c++17 -O2 -pthread -I.. bench_ulog.cpp -o bench_ulog && ./bench_ulog

#include "ulog/ulog.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace unistackbot_common;

namespace
{

// 阶梯限速: 每档 3s, dropped==0 = 后端跟得上该速率
void run_rate(int rate_khz, int seconds)
{
	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = "/tmp/ulog_bench_rate.log";
	std::remove(cfg.file_path);
	ulog_init(cfg);
	const int total = rate_khz * 1000 * seconds;
	const auto period = std::chrono::microseconds(1000 * seconds / rate_khz);
	for (int n = 0; n < total; ++n)
	{
		ULOG_INFO("rate-probe-%d-payload-xxxxxxxxxxxxxxxxxx", n);
		std::this_thread::sleep_for(period);
	}
	ulog_shutdown();
	std::printf("  %4d 千行/s × %ds: dropped=%-6llu %s\n", rate_khz, seconds,
		static_cast<unsigned long long>(ulog_dropped()),
		ulog_dropped() == 0 ? "跟得上" : "饱和(丢旧)");
}

// 全速洪泛: 4 线程无间隔, 观察过载形态 (业务不阻塞 + 丢旧 + 账目守恒)
void run_flood()
{
	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = "/tmp/ulog_bench_flood.log";
	std::remove(cfg.file_path);
	ulog_init(cfg);
	constexpr uint64_t kPer = 25000;
	const auto t0 = std::chrono::steady_clock::now();
	uint64_t max_emit_ns = 0;
	std::thread ts[4];
	for (int t = 0; t < 4; ++t)
	{
		ts[t] = std::thread([&]()
		{
			for (uint64_t n = 0; n < kPer; ++n)
			{
				const auto s = std::chrono::steady_clock::now();
				ULOG_INFO("flood-%llu-payload-xxxxxxxxxxxxxxxx", static_cast<unsigned long long>(n));
				const auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - s).count();
				if (static_cast<uint64_t>(dt) > max_emit_ns)
				{
					max_emit_ns = static_cast<uint64_t>(dt);
				}
			}
		});
	}
	for (auto & t : ts)
	{
		t.join();
	}
	ulog_shutdown();
	(void)t0;
	std::printf("  洪泛 4×25000: max_emit=%lluus dropped=%llu 账目=%s\n",
		static_cast<unsigned long long>(max_emit_ns / 1000),
		static_cast<unsigned long long>(ulog_dropped()),
		ulog_written() + ulog_dropped() == 4 * kPer ? "守恒" : "破缺!");
}

}  // namespace

int main()
{
	std::printf("=== 阶梯限速 (找后端饱和点) ===\n");
	run_rate(5, 3);
	run_rate(10, 3);
	run_rate(50, 3);
	run_rate(100, 3);
	run_rate(200, 3);
	std::printf("=== 全速洪泛 (过载行为) ===\n");
	run_flood();
	std::remove("/tmp/ulog_bench_rate.log");
	std::remove("/tmp/ulog_bench_flood.log");
	return 0;
}

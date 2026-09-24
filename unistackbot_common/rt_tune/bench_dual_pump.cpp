/*
 * bench_dual_pump —— 双泵同居核2 场景基准 (2026-09-24, 设计 §17.4 的实测)。
 *
 * 场景: 双臂 = 两条总线段 = 两个泵线程, 双双 SCHED_FIFO 90 钉**同一核 (核2)**。
 *   泵周期 500Hz, 每拍 7 个均匀分发点 (T/7 ≈ 285.7µs, "均匀铺"调度形态):
 *     ppoll 睡到分发期限 → 量迟滞 → 工作突发 (write 20B 到非阻塞管道 + 排干读
 *     + ~15µs encode/decode 计算) —— 模拟 485 帧泵的每时隙行为。
 *
 * 三相对比:
 *   phase 1 solo         : 单泵独占核2 (基线)
 *   phase 2 dual-offset  : 双泵相位错开 T/2 (期望运行点 —— 自然交错)
 *   phase 3 dual-inphase : 双泵同相 (对抗最坏 —— 验证"迟滞 ≈ 对方一个突发"的界)
 *
 * 检验设计 §17.4 的三个论断: ①双泵同居核2 p99 应仍在 µs 级; ②最坏互相推迟
 * ≈ 对方一个工作突发 (~20µs 量级); ③同相对抗场景是上界, 错相是运行点。
 *
 * 断言 (组件测试语义): rt_tune::apply 绑核+FIFO 全成功, 失败退出码 1。
 * RT 纪律 (2026-09-20 教训, 同 bench_rt_tune): 栈预热/堆预分配/warmup 拍剔除。
 *
 * 用法: bench_dual_pump [duration_s=10] [prio=90] [cpu=2]
 * 编译 (组件目录内, 同 ulog 范本):
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic bench_dual_pump.cpp -o /tmp/bench_dual_pump -pthread
 */
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "rt_tune.hpp"

namespace
{

constexpr int kSlots = 7;              // 每拍分发点 = 7 节点 (均匀铺)
constexpr int kPeriodUs = 2000;        // 泵拍 500Hz
constexpr int kWorkIters = 20000;      // 突发计算量 (~15-20µs 档, 具体以实测突发均值打印为准)
constexpr long long kWarmupCycles = 200;
constexpr size_t kStackPadBytes = 256u * 1024u;

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
	double burst_sum_us = 0.0;   // 工作突发总耗时 (均值打印用)
};

struct PumpArg
{
	int cpu = 2;
	int prio = 90;
	int fd_wr = -1;
	int fd_rd = -1;
	double phase_off_ns = 0.0;   // 相位偏移 (runPhase 内换算成新鲜基准)
	double base_ns = 0.0;        // 调度基准 (runPhase 设置, 保证在未来)
	int period_us = kPeriodUs;   // 泵周期 (阶梯相里 EC/CAN 1kHz / 串口 500Hz)
	int slots = kSlots;          // 每拍分发点 (EC/CAN=1, 串口=7)
	int work_iters = kWorkIters; // 突发计算量 (阶梯相里 EC 突发 ≈5× 串口)
	long long cycles = 0;
	long long skipped = 0;       // 追帧跳过的时隙数 (落后纪律的遥测)
	int tune_rc = -1;
	const char *tag = "";
	Stats st;
};

double clockNowNs()
{
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<double>(ts.tv_sec) * 1.0e9 + static_cast<double>(ts.tv_nsec);
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

// 工作突发: 一次 write(20B 非阻塞管道) + 一次排干 read + 定长计算。
// 返回突发耗时 µs —— 模拟泵时隙内的真实 CPU 占用。
double workBurst(PumpArg *a)
{
	const double t0 = clockNowNs();
	static unsigned char frame[20] = {0xFE, 0xEE};   // 静态: 零每拍构建
	ssize_t r = write(a->fd_wr, frame, sizeof(frame));
	(void)r;   // EAGAIN (管道满) 也是一次真实 syscall, 模拟等价
	unsigned char drain[256];
	r = read(a->fd_rd, drain, sizeof(drain));
	(void)r;
	volatile double acc = 0.0;
	for (int i = 0; i < a->work_iters; ++i)
	{
		acc += static_cast<double>(frame[i % 20]) * 0.001 + static_cast<double>(i & 7);
	}
	asm volatile("" ::"r"(frame) : "memory");
	return (clockNowNs() - t0) / 1.0e3;
}

void pumpMain(PumpArg *a)
{
	// ---- 栈预热: 深栈页缺页发生在此, 不进测量拍 ----
	volatile unsigned char pad[kStackPadBytes];
	for (size_t i = 0; i < kStackPadBytes; i += 4096)
	{
		pad[i] = 0;
	}
	asm volatile("" ::"r"(pad) : "memory");

	char name[16];
	std::snprintf(name, sizeof(name), "pump_%s", a->tag);
	a->tune_rc = unistackbot_common::rt_tune::apply(a->cpu, a->prio, 0, name);

	const double period_ns = static_cast<double>(a->period_us) * 1.0e3;
	const double slot_ns = period_ns / a->slots;
	double target = a->base_ns + period_ns;   // 首拍期限 (runPhase 已保证在未来)

	for (long long c = 0; c < kWarmupCycles + a->cycles; ++c)
	{
		for (int k = 0; k < a->slots; ++k, target += slot_ns)
		{
			// ---- 落后追帧纪律 (生产泵同款, 2026-09-24 实测教训): FIFO 同优先级不让出核,
			//      补发过期期限 = 机器枪狂奔 = 饿死同核兄弟——逾期超一个时隙即跳到未来
			//      期限, 跳过数计遥测, 绝不补发 ----
			const double behind = clockNowNs() - target;
			if (behind > slot_ns)
			{
				const long long skip = static_cast<long long>(behind / slot_ns) + 1;
				target += static_cast<double>(skip) * slot_ns;
				a->skipped += skip;
			}
			// ppoll 睡到分发期限 (nfds=0, ns 精度, 与泵同唤醒路径)
			double remain = target - clockNowNs();
			if (remain > 0.0)
			{
				timespec ts{};
				ts.tv_sec = static_cast<time_t>(remain / 1.0e9);
				ts.tv_nsec = static_cast<long>(remain - static_cast<double>(ts.tv_sec) * 1.0e9);
				ppoll(nullptr, 0, &ts, nullptr);
			}
			const double lat_us = (clockNowNs() - target) / 1.0e3;
			const double burst_us = workBurst(a);

			if (c < kWarmupCycles)
			{
				continue;
			}
			Stats &s = a->st;   // all_us 已预分配, 此处只下标写, 零分配
			const size_t idx = static_cast<size_t>((c - kWarmupCycles) * a->slots + k);
			++s.n;
			s.sum_us += lat_us;
			s.burst_sum_us += burst_us;
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
			s.all_us[idx] = lat_us;
		}
	}
}

// 一相: threads 描述参与泵 (1 或 2 个), 跑 duration_s, 打印统计行。
// 基准时间戳每相新鲜计算 (2026-09-24 教训: 旧版用 main 里的过期 start, 相 2/3 全部
// 补发过期期限 → FIFO 同优先级狂奔饿死兄弟, 秒级垃圾数据)
void runPhase(const char *phase, std::vector<PumpArg> &args, int duration_s)
{
	const double base = clockNowNs() + 50.0e6;   // 每相新鲜基准 + 50ms 安定期
	std::vector<std::thread> ths;
	for (PumpArg &a : args)
	{
		a.base_ns = base + a.phase_off_ns;
		a.cycles = static_cast<long long>(duration_s) * 1000000LL / a.period_us;
		a.skipped = 0;
		a.st = Stats{};
		a.st.all_us.assign(static_cast<size_t>(a.cycles) * a.slots, 0.0);
	}
	// 各泵独立管道 (突发模拟用; O_NONBLOCK 防 write 阻塞)
	for (PumpArg &a : args)
	{
		int fds[2];
		if (pipe2(fds, O_NONBLOCK | O_CLOEXEC) != 0)
		{
			std::fprintf(stderr, "FAIL: pipe2 (%s)\n", std::strerror(errno));
			std::exit(1);
		}
		a.fd_wr = fds[1];
		a.fd_rd = fds[0];
	}
	for (PumpArg &a : args)
	{
		ths.emplace_back(pumpMain, &a);
	}
	for (std::thread &t : ths)
	{
		t.join();
	}

	std::printf("== %s ==\n", phase);
	for (PumpArg &a : args)
	{
		Stats &s = a.st;
		std::vector<double> v = s.all_us;
		std::printf(
			"  pump_%s: n=%lld 迟滞 min=%.1f avg=%.1f p50=%.1f p99=%.1f max=%.1f | "
			">10µs=%lld >50µs=%lld >100µs=%lld | 突发均值=%.1fµs 跳帧=%lld\n",
			a.tag, s.n, s.min_us, s.sum_us / static_cast<double>(s.n),
			percentileUs(v, 0.50), percentileUs(v, 0.99), s.max_us,
			s.over_10, s.over_50, s.over_100, s.burst_sum_us / static_cast<double>(s.n), a.skipped);
	}
}

}  // namespace

int main(int argc, char **argv)
{
	const int duration_s = argc > 1 ? std::atoi(argv[1]) : 10;
	const int prio = argc > 2 ? std::atoi(argv[2]) : 90;
	const int cpu = argc > 3 ? std::atoi(argv[3]) : 2;
	if (duration_s <= 0)
	{
		std::fprintf(stderr, "用法: %s [duration_s=10] [prio=90] [cpu=2]\n", argv[0]);
		return 1;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
	{
		std::fprintf(stderr, "WARN: mlockall 失败 (%s) — 继续未锁定\n", std::strerror(errno));
	}

	const double period_ns = static_cast<double>(kPeriodUs) * 1.0e3;

	int fail = 0;

	// phase 1: 单泵基线
	{
		std::vector<PumpArg> args(1);
		args[0].cpu = cpu; args[0].prio = prio; args[0].tag = "A";
		runPhase("phase1 solo (单泵独占核, 基线)", args, duration_s);
		fail += args[0].tune_rc != unistackbot_common::rt_tune::kOk;
	}

	// phase 2: 双泵错相 T/2 (期望运行点)
	{
		std::vector<PumpArg> args(2);
		args[0].cpu = cpu; args[0].prio = prio; args[0].tag = "A";
		args[1].cpu = cpu; args[1].prio = prio; args[1].tag = "B"; args[1].phase_off_ns = period_ns / 2.0;
		runPhase("phase2 dual-offset (双泵错相 T/2 — 期望运行点)", args, duration_s);
		for (PumpArg &a : args)
		{
			fail += a.tune_rc != unistackbot_common::rt_tune::kOk;
		}
	}

	// phase 3: 双泵同相 (对抗最坏)
	{
		std::vector<PumpArg> args(2);
		args[0].cpu = cpu; args[0].prio = prio; args[0].tag = "A";
		args[1].cpu = cpu; args[1].prio = prio; args[1].tag = "B";   // 同相 (off=0)
		runPhase("phase3 dual-inphase (双泵同相 — 对抗最坏)", args, duration_s);
		for (PumpArg &a : args)
		{
			fail += a.tune_rc != unistackbot_common::rt_tune::kOk;
		}
	}

	// phase 4: 总线优先级阶梯 (2026-09-24 用户裁决: EC95 > CAN90 > 串口85 > CM80)
	//   EC 线程   prio 95, 1kHz, 单分发, 突发 ≈5× 串口 (~150µs, ecrt 周期处理假设值,
	//             EC 后端落地后以实测替换)
	//   CAN 线程  prio 90, 1kHz, 单分发, 突发 ≈2.5× 串口 (~80µs)
	//   串口泵    prio 85, 500Hz, 7 时隙 (现役形态) —— 量它被高优先级突发推迟多少
	{
		std::vector<PumpArg> args(3);
		args[0].cpu = cpu; args[0].prio = 95; args[0].tag = "EC95";
		args[0].period_us = 1000; args[0].slots = 1; args[0].work_iters = 5 * kWorkIters;
		args[1].cpu = cpu; args[1].prio = 90; args[1].tag = "CAN90";
		args[1].period_us = 1000; args[1].slots = 1; args[1].work_iters = 5 * kWorkIters / 2;
		args[2].cpu = cpu; args[2].prio = 85; args[2].tag = "S85";
		args[2].period_us = 2000; args[2].slots = kSlots;   // 现役串口泵形态
		args[2].phase_off_ns = period_ns / 7.0;
		runPhase("phase4 ladder (EC95/CAN90/S85 同核 — 阶梯裁决实测)", args, duration_s);
		for (PumpArg &a : args)
		{
			fail += a.tune_rc != unistackbot_common::rt_tune::kOk;
		}
	}

	// phase 5: 真实最坏部署 (2026-09-24 用户裁决: 单核最多 2 总线; 若有第三条上核1
	//   与 CM 同住, 隔离核3 永久出局) —— 最大突发 (EC) + 最低优先级 (串口) 的两总线组合
	{
		std::vector<PumpArg> args(2);
		args[0].cpu = cpu; args[0].prio = 95; args[0].tag = "EC95";
		args[0].period_us = 1000; args[0].slots = 1; args[0].work_iters = 5 * kWorkIters;
		args[1].cpu = cpu; args[1].prio = 85; args[1].tag = "S85";
		args[1].period_us = 2000; args[1].slots = kSlots;
		args[1].phase_off_ns = period_ns / 7.0;
		runPhase("phase5 worst-pair (EC95+S85 同核 — 真实最坏部署)", args, duration_s);
		for (PumpArg &a : args)
		{
			fail += a.tune_rc != unistackbot_common::rt_tune::kOk;
		}
	}

	// phase 6: 四线程全布局 (2026-09-24 用户场景: 第三条总线上核1 与 CM 同住)
	//   核1: CM(80, 500Hz, 单分发 ~24µs=CM 环实测 WCET 口径) + 串口泵A(85, 7时隙)
	//        —— CM 是全系统最低优先级 = 本核受害者; 核内取同相 (对抗最坏)
	//   核2: EC(95, 1kHz, ~152µs 突发) + 串口泵B(85, 7时隙) —— phase5 复验
	//   各线程自由相位 (不跨核同步); 量的是每个参与者的分发迟滞最坏账
	{
		std::vector<PumpArg> args(4);
		args[0].cpu = 1; args[0].prio = 80; args[0].tag = "CM80";
		args[0].period_us = 2000; args[0].slots = 1; args[0].work_iters = 15000;   // ~24µs
		args[1].cpu = 1; args[1].prio = 85; args[1].tag = "S85a";
		args[1].period_us = 2000; args[1].slots = kSlots;                          // 与 CM 同相 = 最坏
		args[2].cpu = 2; args[2].prio = 95; args[2].tag = "EC95";
		args[2].period_us = 1000; args[2].slots = 1; args[2].work_iters = 5 * kWorkIters;
		args[3].cpu = 2; args[3].prio = 85; args[3].tag = "S85b";
		args[3].period_us = 2000; args[3].slots = kSlots;
		args[3].phase_off_ns = period_ns / 7.0;
		runPhase("phase6 full-layout (核1:CM80+S85a / 核2:EC95+S85b — 四线程全布局)", args, duration_s);
		for (PumpArg &a : args)
		{
			fail += a.tune_rc != unistackbot_common::rt_tune::kOk;
		}
	}

	// phase 7: 全系统挤一个核 (2026-09-24 核稀缺平台最小配置: EC95+S85a+S85b+CM80
	//   同核——4核级板卡隔离 1 核、家务 3 核的极限形态; 串口泵的干扰链 = EC突发152
	//   + 同级泵突发31 ≈ 183µs, CM 的链 ≈ EC+泵 ≈ 183µs, 均应仍在预算内)
	{
		std::vector<PumpArg> args(4);
		args[0].cpu = 2; args[0].prio = 95; args[0].tag = "EC95";
		args[0].period_us = 1000; args[0].slots = 1; args[0].work_iters = 5 * kWorkIters;
		args[1].cpu = 2; args[1].prio = 85; args[1].tag = "S85a";
		args[1].period_us = 2000; args[1].slots = kSlots;
		args[2].cpu = 2; args[2].prio = 85; args[2].tag = "S85b";
		args[2].period_us = 2000; args[2].slots = kSlots;
		args[2].phase_off_ns = period_ns / 2.0;
		args[3].cpu = 2; args[3].prio = 80; args[3].tag = "CM80";
		args[3].period_us = 2000; args[3].slots = 1; args[3].work_iters = 15000;
		args[3].phase_off_ns = period_ns / 14.0;
		runPhase("phase7 one-core (EC95+S85a+S85b+CM80 全系统单核 — 核稀缺最小配置)", args, duration_s);
		for (PumpArg &a : args)
		{
			fail += a.tune_rc != unistackbot_common::rt_tune::kOk;
		}
	}

	if (fail != 0)
	{
		std::printf("FAIL: rt_tune::apply 有未成功项 (期望核%d SCHED_FIFO prio=%d)\n", cpu, prio);
		return 1;
	}
	std::printf("PASS: rt_tune 断言绿; 判读: 错相 p99 应近 solo; 同相 max ≈ solo max + 一个突发档\n");
	return 0;
}

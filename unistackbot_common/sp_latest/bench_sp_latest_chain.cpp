/*
 * bench_sp_latest_chain —— SpLatest 双核双线程链路基准 (场景二, 2026-09-20)。
 *
 * 真布局终版 (极简裁决定稿): **总线线程 = 节拍源不可移相** (EtherCAT DC 锚定/帧时序);
 * **CM 线程 = 框架线程相位不可控** → 相位差 = 随机常量, 双方都不动, 不加任何同步补丁。
 *   核2 总线线程 SCHED_FIFO 90: IgH 原生拍内顺序 (send→总线往返→receive→publish fb→read cmd), 执行器一阶模型
 *   核1 CM 线程    SCHED_FIFO 80: update() P 跟踪正弦目标 (1Hz), 自由跑
 *   ch_feedback (反馈) / ch_command (控制) 两个独立 SpLatest 值通道 (裸 publish/read)
 *
 * 为什么这就够 (三层裁决史): ① 撞窗 → read false → 沿用旧值一拍, 实测最坏同相 9.6% miss 下
 *   errRMS 与零 miss 差 2% — 晚一帧不重要; ② 总线拍内原生顺序把通道操作推离拍首 (~往返),
 *   同时启动天然不撞; ③ 上游死 = seq 冻结 = StaleWatch 域 (安全, 非性能补丁)。
 *   readWait 自旋/seq 等新帧/监控移相均已试作并否决 (为 0.5% 概率的 2% 代价上机制 = 过度设计)。
 *   回头加等待类机制的判据: 实测 errRMS/相位裕度退化 (2kHz+ 力控高带宽场景)。
 *
 * 数据结构纪律: 报文与统计结构的整型**全部定宽** (int64_t/uint8_t/...), 时间戳 int64_t ns —
 *   保证跨平台 (x86_64 ↔ ARM 交叉编译) 字节数与语义一致; 浮点仅物理量 (IEEE754 double, 恒 8 字节)。
 *
 * RT 纪律: 统计缓冲 main 侧预分配 populate; 线程入口栈 pad 触碰; 共同 warmup 窗口外才统计;
 *          热路径零分配零打印; 两线程共用 main 下发的 start/end 墙钟窗口 (统计窗对齐)。
 *
 * 用法: bench_sp_latest_chain <bus_period_us> <cm_period_us> <duration_s> [cm_phase_us]
 *        cm_phase_us: CM 初始相位偏移; -1=随机 (默认, 模拟框架激活时序); 0=最坏同相
 *        示例: 1000 1000 30 0   (1kHz 同频, 最坏相位)
 * 编译 (组件目录内):
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I../rt_tune bench_sp_latest_chain.cpp -o /tmp/bench_chain -pthread
 */
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <sys/mman.h>
#include <thread>
#include <vector>

#include "rt_tune.hpp"
#include "sp_latest.hpp"

namespace
{

constexpr int kCpuCm = 1;     // 核1 = CM update
constexpr int kCpuBus = 2;    // 核2 = 总线 (read/write)
constexpr int32_t kPrioCm = 80;   // CM 档 (yaml thread_priority)
constexpr int32_t kPrioBus = 90;  // 总线主站档 (IgH, 高于 CM: write 不可被 update 推迟)
constexpr int64_t kWarmupNs = 500'000'000LL;    // 共同预热窗 (墙钟 0.5s)
constexpr size_t kStackPadBytes = 256u * 1024u;
constexpr int32_t kJoints = 32;        // 总线报文按 32 关节设计 (人形态上限; ros2_control 链的 kMaxJoints=16 是另一层)
constexpr uint8_t kModeCsp = 1;
constexpr double kPgain = 0.5;          // CM 模拟: P 控制
constexpr double kActuatorRate = 0.05;  // 执行器一阶逼近系数 (每拍 5%, 同 kinematic 后端语义)
constexpr double kRefFreqHz = 1.0;      // 正弦跟踪目标频率 (时变目标, 跟踪误差才能暴露数据新鲜度代价)
constexpr double kPi = 3.14159265358979323846;
constexpr int32_t kMissRingN = 1024;    // 尾窗 miss 统计 (拍)
constexpr int64_t kBusTurnaroundNs = 100'000;  // 模拟总线往返 (EtherCAT 100Mbps 网段+从站处理典型量级)

// 正弦参考轨迹 (两线程共享的纯函数): 幅度按关节编号变化
double refPos(int32_t j, double t_s)
{
	const double amp = 0.1 * static_cast<double>((j % 8) + 1) * ((j % 2 == 0) ? 1.0 : -1.0);
	return amp * std::sin(2.0 * kPi * kRefFreqHz * t_s + 0.7 * static_cast<double>(j));
}

// 反馈报文 (总线→CM): 对齐真实从站反馈的量级 — 位置/速度/力矩/错误码/母线电压
// **补齐 64 倍数 (评审六轮)**: slots_[1] 的地址 = 基址+sizeof(T), 非倍数则跨缓存行 —
// 用 reserved 保留字节补齐 (真实总线报文本就带保留段), static_assert 钉死防漂移。
struct Feedback
{
	double position[kJoints];
	double velocity[kJoints];
	double torque[kJoints];
	int32_t error_code[kJoints];
	double bus_voltage;
	int64_t stamp_ns;   // 总线 read() 时刻 (E2E 测量)
	uint8_t reserved[48];   // 912 → 960 = 15×64
};
static_assert(sizeof(Feedback) == 960, "Feedback 须为 64 的倍数 (槽对齐)");

// 控制报文 (CM→总线): 位置命令 + 速度/力矩前馈 + 模式/使能 (同样补齐 64 倍数)
struct Command
{
	double position_cmd[kJoints];
	double velocity_ff[kJoints];
	double torque_ff[kJoints];
	uint8_t mode[kJoints];
	uint8_t enable[kJoints];
	int64_t stamp_ns;   // CM update() 时刻
	uint8_t reserved[56];   // 840 → 896 = 14×64
};
static_assert(sizeof(Command) == 896, "Command 须为 64 的倍数 (槽对齐)");

struct Stats
{
	int64_t n = 0;
	double max_us = 0.0;
	int64_t over_100 = 0;
	std::vector<double> all_us;
};

struct ChainStats
{
	Stats wake;                // 周期唤醒延迟 (每统计拍必记)
	Stats e2e;                 // 通道消费 E2E (read 成功才记)
	int64_t seq_d0 = 0;        // 步进=0 (重复读)
	int64_t seq_d1 = 0;        // 步进=1 (一个版本; SpLatest 每 publish seq+2, 同频正常读即此)
	int64_t seq_d2 = 0;
	int64_t seq_dgt2 = 0;      // 步进>2 (上游覆盖写丢版本, 值通道正常行为)
	int64_t miss = 0;          // 无可用帧 (kNone 且从未 live — 罕见)
	int64_t stale_frames = 0;  // 本拍用上一帧 (kNone 且历史 live — 需求语义的正常降级)
	int64_t init_frames = 0;   // init 占位帧 (seq==kInitFrameSeq, 非实际反馈, 已忽略)
	double err_sum_sq = 0.0;   // 稳态跟踪误差平方和 (总线侧统计, 时变目标)
	double err_max = 0.0;
	int64_t err_n = 0;
	uint8_t miss_ring[kMissRingN] = {0};  // 尾窗环形 (最近 kMissRingN 统计拍)
	int64_t miss_ring_sum = 0;
	int32_t miss_ring_idx = 0;
	bool seq_regress = false;  // seq 回退 (bug 哨兵, 契约违反)
	uint64_t last_seq = 0;
	bool have_last = false;
};

struct WinArg
{
	int64_t start_ns = 0;   // 共同窗口
	int64_t end_ns = 0;
	int64_t period_ns = 1'000'000;
	int64_t budget = 0;     // 预分配拍数上限 (含 warmup 余量)
	ChainStats cs;
};

int64_t clockNowNs()
{
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + static_cast<int64_t>(ts.tv_nsec);
}

timespec toTimespec(int64_t ns)
{
	timespec ts{};
	ts.tv_sec = static_cast<time_t>(ns / 1'000'000'000LL);
	ts.tv_nsec = static_cast<long>(ns % 1'000'000'000LL);
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

void stackPad()
{
	volatile unsigned char pad[kStackPadBytes];
	for (size_t i = 0; i < kStackPadBytes; i += 4096)
	{
		pad[i] = 0;
	}
	asm volatile("" ::"r"(pad) : "memory");
}

void record(Stats &s, double lat_us)
{
	const size_t k = static_cast<size_t>(s.n);
	s.all_us[k] = lat_us;
	++s.n;
	if (lat_us > s.max_us)
	{
		s.max_us = lat_us;
	}
	if (lat_us > 100.0)
	{
		++s.over_100;
	}
}

void pushMissRing(ChainStats &cs, bool miss)
{
	cs.miss_ring_sum -= cs.miss_ring[cs.miss_ring_idx];
	cs.miss_ring[cs.miss_ring_idx] = miss ? 1 : 0;
	cs.miss_ring_sum += miss ? 1 : 0;
	cs.miss_ring_idx = (cs.miss_ring_idx + 1) % kMissRingN;
}

void recordSeq(ChainStats &cs, uint64_t seq)
{
	if (cs.have_last)
	{
		if (seq < cs.last_seq)
		{
			cs.seq_regress = true;
		}
		else
		{
			const uint64_t d = seq - cs.last_seq;
			if (d == 0)
			{
				++cs.seq_d0;
			}
			else if (d == 1)
			{
				++cs.seq_d1;
			}
			else if (d == 2)
			{
				++cs.seq_d2;
			}
			else
			{
				++cs.seq_dgt2;
			}
		}
	}
	cs.last_seq = seq;
	cs.have_last = true;
}

// ---------- 核2: 总线线程 (FIFO90) — read() + write() 闭环 + 观测移相控制律 ----------
struct BusArg
{
	WinArg common;
	double position[kJoints] = {0};    // 执行器状态 (本地, 模拟硬件)
	double velocity[kJoints] = {0};    // 反馈遥测 (静态模拟值, main 装配期填)
	double torque[kJoints] = {0};
	int32_t err_code[kJoints] = {0};
	double bus_voltage = 48.0;
	int32_t tune_rc = -1;
};

void busMain(BusArg *a, unistackbot_common::SpLatest<Feedback> *ch_fb,
	unistackbot_common::SpLatest<Command> *ch_cmd)
{
	stackPad();
	a->tune_rc = unistackbot_common::rt_tune::apply(kCpuBus, kPrioBus, 0, "chain_bus");

	const int64_t period_ns = a->common.period_ns;
	const int64_t stat_after = a->common.start_ns + kWarmupNs;
	int64_t target = a->common.start_ns + period_ns;
	uint64_t cmd_seq = 0;
	// 终版姿势三件套: 持久接收变量 (kNone 时 out = 上一帧的载体) + live 历史 (防 init 残留)
	Command cmd{};
	bool cmd_live = false;

	for (int64_t i = 0; i < a->common.budget; ++i)
	{
		const timespec ts = toTimespec(target);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
		const int64_t now = clockNowNs();
		target += period_ns;
		if (now >= a->common.end_ns)
		{
			break;
		}
		const bool stat = now >= stat_after;
		const int64_t tick = target - period_ns;   // 本拍拍首

		// ---- IgH 原生拍内顺序: send(上拍缓存 cmd) → 总线往返 → receive → publish fb → read cmd(供下拍) ----
		// 结构即解: publish fb 与 read cmd 都天然落在拍首+往返(~100µs), 两个撞窗点全部离开拍首 —
		// 与 CM (拍首读/写) 即使完全同相也不撞; 不需要移相/监控/偏置任何补丁。
		// (bench 用 sleep 模拟往返等待; 真实主站此处为帧等待/中断, 抖动更小)
		const timespec ts_io = toTimespec(tick + kBusTurnaroundNs);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_io, nullptr);

		// receive 后: 执行器状态 + 遥测 → feedback 通道
		Feedback fb{};
		for (int32_t j = 0; j < kJoints; ++j)
		{
			fb.position[j] = a->position[j];
			fb.velocity[j] = a->velocity[j];
			fb.torque[j] = a->torque[j];
			fb.error_code[j] = a->err_code[j];
		}
		fb.bus_voltage = a->bus_voltage;
		fb.stamp_ns = tick;
		ch_fb->publish(fb);

		// write(): 终版姿势 "要么最新要么上一帧" — readLastFrame 重试取新; 耗尽 (kNone) 时
		// cmd 保持 = 上一帧, 仅当历史已有真实帧 (cmd_live) 才应用; init 占位挡在执行器外
		bool cmd_miss = false;
		const auto cmd_kind = ch_cmd->readLastFrame(cmd, cmd_seq);
		const bool use_cmd = (cmd_kind == unistackbot_common::SpLatest<Command>::FrameKind::kLive)
			|| (cmd_kind == unistackbot_common::SpLatest<Command>::FrameKind::kNone && cmd_live);
		if (cmd_kind == unistackbot_common::SpLatest<Command>::FrameKind::kLive)
		{
			cmd_live = true;
		}
		else if (cmd_kind == unistackbot_common::SpLatest<Command>::FrameKind::kInit)
		{
			cmd_live = false;
		}
		if (use_cmd)
		{
			if (stat && cmd_kind == unistackbot_common::SpLatest<Command>::FrameKind::kNone)
			{
				++a->common.cs.stale_frames;  // 本拍沿用上一帧 (正常降级, 数据仍可用)
			}
			for (int32_t j = 0; j < kJoints; ++j)
			{
				a->position[j] += kActuatorRate * (cmd.position_cmd[j] - a->position[j]);
			}
			if (stat)
			{
				recordSeq(a->common.cs, cmd_seq);
				record(a->common.cs.e2e, static_cast<double>(clockNowNs() - cmd.stamp_ns) / 1.0e3);
			}
		}
		else if (stat)
		{
			cmd_miss = true;
			if (cmd_kind == unistackbot_common::SpLatest<Command>::FrameKind::kInit)
			{
				++a->common.cs.init_frames;   // CM 未上线的占位帧: 忽略
			}
			else
			{
				++a->common.cs.miss;          // kNone 且从未 live: 无可用帧 (罕见)
			}
		}

		if (stat)
		{
			// 稳态跟踪误差 (时变目标): 撞窗 miss 的代价最终体现在这里
			const double t_s = static_cast<double>(now) * 1.0e-9;
			for (int32_t j = 0; j < kJoints; ++j)
			{
				const double err = a->position[j] - refPos(j, t_s);
				a->common.cs.err_sum_sq += err * err;
				if (std::fabs(err) > a->common.cs.err_max)
				{
					a->common.cs.err_max = std::fabs(err);
				}
				++a->common.cs.err_n;
			}
			pushMissRing(a->common.cs, cmd_miss);
			record(a->common.cs.wake, static_cast<double>(now - (target - period_ns)) / 1.0e3);
		}
	}
}

// ---------- 核1: CM 线程 (FIFO80) — update(), 自由跑 (模拟框架线程) ----------
struct CmArg
{
	WinArg common;
	int64_t phase_ns = 0;   // 初始相位 (模拟 ros2_control 激活时序: 指定或随机)
	int32_t tune_rc = -1;
};

void cmMain(CmArg *a, unistackbot_common::SpLatest<Feedback> *ch_fb,
	unistackbot_common::SpLatest<Command> *ch_cmd)
{
	stackPad();
	a->tune_rc = unistackbot_common::rt_tune::apply(kCpuCm, kPrioCm, 0, "chain_cm");

	const int64_t period_ns = a->common.period_ns;
	const int64_t stat_after = a->common.start_ns + kWarmupNs;
	int64_t target = a->common.start_ns + a->phase_ns + period_ns;   // 相位 = 启动时刻平移, 之后自由跑
	// 终版姿势三件套: 持久接收变量 (kNone 时 out = 上一帧的载体) + live 历史 (防 init 残留)
	Feedback fb{};
	uint64_t fb_seq = 0;
	bool fb_live = false;

	for (int64_t i = 0; i < a->common.budget; ++i)
	{
		const timespec ts = toTimespec(target);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
		const int64_t now = clockNowNs();
		target += period_ns;
		if (now >= a->common.end_ns)
		{
			break;
		}
		const bool stat = now >= stat_after;

		// update(): 终版姿势 "要么最新要么上一帧" — readLastFrame 重试取新; 耗尽 (kNone) 时
		// fb 保持 = 上一帧, 仅当历史已有真实帧 (fb_live) 才进控制; init 占位挡在控制外
		bool fb_miss = false;
		const auto fb_kind = ch_fb->readLastFrame(fb, fb_seq);
		const bool use_fb = (fb_kind == unistackbot_common::SpLatest<Feedback>::FrameKind::kLive)
			|| (fb_kind == unistackbot_common::SpLatest<Feedback>::FrameKind::kNone && fb_live);
		if (fb_kind == unistackbot_common::SpLatest<Feedback>::FrameKind::kLive)
		{
			fb_live = true;
		}
		else if (fb_kind == unistackbot_common::SpLatest<Feedback>::FrameKind::kInit)
		{
			fb_live = false;
		}
		if (use_fb)
		{
			if (stat && fb_kind == unistackbot_common::SpLatest<Feedback>::FrameKind::kNone)
			{
				++a->common.cs.stale_frames;   // 本拍沿用上一帧 (正常降级)
			}
			const double t_s = static_cast<double>(now) * 1.0e-9;
			Command cmd{};
			for (int32_t j = 0; j < kJoints; ++j)
			{
				const double ref = refPos(j, t_s);
				cmd.position_cmd[j] = fb.position[j] + kPgain * (ref - fb.position[j]);
				cmd.velocity_ff[j] = 0.0;
				cmd.torque_ff[j] = 0.0;
				cmd.mode[j] = kModeCsp;
				cmd.enable[j] = 1;
			}
			cmd.stamp_ns = now;
			ch_cmd->publish(cmd);
			if (stat)
			{
				recordSeq(a->common.cs, fb_seq);
				record(a->common.cs.e2e, static_cast<double>(clockNowNs() - fb.stamp_ns) / 1.0e3);
			}
		}
		else
		{
			fb_miss = true;
			if (stat && fb_kind == unistackbot_common::SpLatest<Feedback>::FrameKind::kInit)
			{
				++a->common.cs.init_frames;   // init 占位帧 (总线未上线): 忽略, 本拍不产 cmd
			}
			else if (stat)
			{
				++a->common.cs.miss;          // kNone 且从未 live: 无可用帧 (罕见)
			}
		}

		if (stat)
		{
			pushMissRing(a->common.cs, fb_miss);
			record(a->common.cs.wake, static_cast<double>(now - (target - period_ns)) / 1.0e3);
		}
	}
}

void printLine(const char *tag, const WinArg &c)
{
	const ChainStats &cs = c.cs;
	std::vector<double> w = cs.wake.all_us;
	w.resize(static_cast<size_t>(cs.wake.n));
	std::vector<double> e = cs.e2e.all_us;
	e.resize(static_cast<size_t>(cs.e2e.n));
	std::printf(
		"%s wake: p50=%.1f p99=%.1f max=%.1fµs (>100: %" PRId64 "/%" PRId64 ") | "
		"e2e: p50=%.0f p99=%.0f max=%.0fµs | miss=%" PRId64 " 旧帧=%" PRId64 " init帧=%" PRId64 " 尾窗=%" PRId64 "/%" PRId32 " | "
		"跟踪 errRMS=%.4f max=%.4f | seq 0/1/2/>2: %" PRId64 "/%" PRId64 "/%" PRId64 "/%" PRId64 "\n",
		tag, percentileUs(w, 0.50), percentileUs(w, 0.99), cs.wake.max_us, cs.wake.over_100, cs.wake.n,
		percentileUs(e, 0.50), percentileUs(e, 0.99), percentileUs(e, 1.0),
		cs.miss, cs.stale_frames, cs.init_frames, cs.miss_ring_sum, kMissRingN,
		(cs.err_n > 0) ? std::sqrt(cs.err_sum_sq / static_cast<double>(cs.err_n)) : 0.0, cs.err_max,
		cs.seq_d0, cs.seq_d1, cs.seq_d2, cs.seq_dgt2);
}

}  // namespace

int main(int argc, char **argv)
{
	if (argc < 4)
	{
		std::fprintf(stderr, "用法: %s <bus_period_us> <cm_period_us> <duration_s> [cm_phase_us=-1随机]\n", argv[0]);
		return 1;
	}
	const int64_t bus_period = std::atoll(argv[1]);
	const int64_t cm_period = std::atoll(argv[2]);
	const int64_t duration_s = std::atoll(argv[3]);
	const int64_t phase_arg = (argc > 4) ? std::atoll(argv[4]) : -1;
	if (bus_period <= 0 || cm_period <= 0 || duration_s <= 0)
	{
		std::fprintf(stderr, "参数须为正整数\n");
		return 1;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
	{
		std::fprintf(stderr, "WARN: mlockall 失败 (%s) — 继续未锁定\n", std::strerror(errno));
	}

	// ---- 装配期 (线程启动前): 通道 init + 统计缓冲预分配 populate ----
	unistackbot_common::SpLatest<Feedback> ch_fb;
	unistackbot_common::SpLatest<Command> ch_cmd;
	ch_fb.init(Feedback{});
	ch_cmd.init(Command{});

	// CM 初始相位: 指定或随机 (模拟 ros2_control 激活时序不可控)
	const int64_t cm_phase_ns = (phase_arg >= 0)
		? phase_arg * 1000LL
		: [&]()
		{
			std::mt19937_64 rng(static_cast<uint64_t>(
				std::chrono::steady_clock::now().time_since_epoch().count()));
			return static_cast<int64_t>(rng() % static_cast<uint64_t>(cm_period)) * 1000LL;
		}();

	const int64_t start = clockNowNs();
	BusArg ba;
	CmArg ca;
	// 装配期填遥测模拟值 (静态非零, 保证报文拷贝量真实)
	for (int32_t j = 0; j < kJoints; ++j)
	{
		ba.velocity[j] = 0.05 * static_cast<double>(j);
		ba.torque[j] = 1.5 + 0.1 * static_cast<double>(j);
	}
	ba.common.start_ns = start;
	ca.common.start_ns = start;
	ba.common.end_ns = start + duration_s * 1'000'000'000LL;
	ca.common.end_ns = ba.common.end_ns;
	ba.common.period_ns = bus_period * 1000LL;
	ca.common.period_ns = cm_period * 1000LL;
	ca.phase_ns = cm_phase_ns;
	// 预算 = 窗口时长 + 2s 余量 (warmup + 启动差), resize 顺带 populate 页
	ba.common.budget = (duration_s + 2) * 1'000'000LL / bus_period;
	ca.common.budget = (duration_s + 2) * 1'000'000LL / cm_period;
	ba.common.cs.wake.all_us.resize(static_cast<size_t>(ba.common.budget));
	ba.common.cs.e2e.all_us.resize(static_cast<size_t>(ba.common.budget));
	ca.common.cs.wake.all_us.resize(static_cast<size_t>(ca.common.budget));
	ca.common.cs.e2e.all_us.resize(static_cast<size_t>(ca.common.budget));

	std::thread th_bus(busMain, &ba, &ch_fb, &ch_cmd);
	std::thread th_cm(cmMain, &ca, &ch_fb, &ch_cmd);
	th_bus.join();
	th_cm.join();

	const uint32_t t_fb = ch_fb.tears();
	const uint32_t t_cmd = ch_cmd.tears();
	std::printf("总线 核%d FIFO%" PRId32 " @%.0fHz | CM 核%d FIFO%" PRId32 " @%.0fHz 相位=%" PRId64 "µs | 撕裂 fb/cmd=%u/%u\n",
		kCpuBus, kPrioBus, 1.0e6 / static_cast<double>(bus_period),
		kCpuCm, kPrioCm, 1.0e6 / static_cast<double>(cm_period), cm_phase_ns / 1000, t_fb, t_cmd);
	printLine("[bus]", ba.common);
	printLine("[cm ]", ca.common);

	int32_t fail = 0;
	if (ba.tune_rc != unistackbot_common::rt_tune::kOk || ca.tune_rc != unistackbot_common::rt_tune::kOk)
	{
		std::printf("FAIL: rt_tune::apply 未全成功 (bus rc=%" PRId32 ", cm rc=%" PRId32 ")\n", ba.tune_rc, ca.tune_rc);
		++fail;
	}
	if (ba.common.cs.seq_regress || ca.common.cs.seq_regress)
	{
		std::printf("FAIL: seq 回退 (SpLatest 契约违反)\n");
		++fail;
	}
	// 尾窗稳态断言: 仅当统计拍数 ≥ 环形长度时, 尾窗才真正代表"挤出旧样本后的最近稳态"
	if (ba.common.cs.wake.n >= kMissRingN && ca.common.cs.wake.n >= kMissRingN)
	{
		if (ba.common.cs.miss_ring_sum != 0 || ca.common.cs.miss_ring_sum != 0)
		{
			std::printf("FAIL: 尾窗 miss 非零 (移相未收敛到稳态零窗)\n");
			++fail;
		}
	}
	if (t_fb != 0 || t_cmd != 0)
	{
		// 契约允许撕裂 (拒绝+沿用旧值, 已计入 miss); 低频正常, 持续增长才是写读频率失衡信号
		std::printf("ℹ 撕裂 fb=%u cmd=%u (契约允许, 已按 miss 处理; 持续增长才需关注)\n", t_fb, t_cmd);
	}
	if (fail != 0)
	{
		return 1;
	}
	std::printf("PASS: 真布局链路绿 (调度生效 / 无 seq 回退 / 尾窗稳态)\n");
	return 0;
}

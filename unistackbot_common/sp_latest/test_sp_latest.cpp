// SpLatest 功能测试 + 撕裂压测 (规范 42 交付门槛)。
// 独立编译, 零 ROS 依赖:
//   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion test_sp_latest.cpp -o test_sp_latest && ./test_sp_latest
// (-Wconversion 抓类型截断 —— uint64 化时 s0/s1 曾漏改, 靠评审抓出; 此 flag 让编译器兜底)
// 核心断言: 写者全速轰击下读者**零脏数据** (撕裂由重试消化, 沿用旧值是契约不是失败)。

#include "sp_latest.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <sys/prctl.h>
#include <thread>
#include <vector>

namespace
{

using unistackbot_common::SpLatest;

// 自校验负载: 所有字段恒等于 stamp —— 任何撕裂/半新半旧立刻现形
struct Payload
{
	uint64_t stamp{0};
	uint64_t a{0};
	uint64_t b{0};
	uint64_t c{0};
};
static_assert(std::is_trivially_copyable_v<Payload>, "test payload must be trivially copyable");

constexpr uint64_t kWriterOps = 1000000;    // 写者压测次数 (全速, 远高于读者消费速度)

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
	// 默认构造: 未发布语义
	SpLatest<Payload> empty;
	Payload out;
	uint64_t s = 0;
	check(empty.read(out, s) == false, "default: read=false before first publish");
	check(empty.seq() == 0, "default: seq==0");

	// 带初值 via init(): 首拍前即有稳定值 (seq 从偶数 2 起算)
	SpLatest<Payload> seeded;
	seeded.init(Payload{7, 7, 7, 7});
	check(seeded.read(out, s) == true && out.a == 7 && s == 2, "seeded: init + first read ok");

	// 往返: 数据与 seq 一致
	seeded.publish(Payload{42, 42, 42, 42});
	check(seeded.read(out, s) == true && out.a == 42 && s == 4, "roundtrip: data+seq");

	// 100 万次往返: 每次数据一致 + seq 严格 +2 单调
	uint64_t prev = s;
	bool round_ok = true;
	for (uint32_t i = 0; i < 1000000; ++i)
	{
		seeded.publish(Payload{i, i, i, i});
		if (!seeded.read(out, s) || out.a != i || s != prev + 2)
		{
			round_ok = false;
			break;
		}
		prev = s;
	}
	check(round_ok, "roundtrip x1M: data consistent + seq strictly monotonic");
}

void stress_test()
{
	SpLatest<Payload> buf;
	std::atomic<bool> writer_done{false};
	std::atomic<uint64_t> dirty{0};     // 脏数据 (不一致快照) —— 硬失败
	std::atomic<uint64_t> reads{0};
	std::atomic<uint64_t> misses{0};    // read=false (契约允许: 沿用旧值)

	std::thread writer([&]()
	{
		for (uint64_t i = 1; i <= kWriterOps; ++i)
		{
			buf.publish(Payload{i, i, i, i});
		}
		writer_done.store(true, std::memory_order_release);
	});

	std::thread reader([&]()
	{
		Payload out;
		uint64_t s = 0;
		while (!writer_done.load(std::memory_order_acquire))
		{
			if (buf.read(out, s))
			{
				reads.fetch_add(1, std::memory_order_relaxed);
				// 脏数据 = 自校验负载不一致 或 seq 为奇(稳定态不该是奇)
				// 注意: seq 每拍 +2, 与 stamp 不再相等, 不能拿 seq==stamp 当一致性判据
				if (out.stamp != out.a || out.a != out.b || out.b != out.c || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
			}
			else
			{
				misses.fetch_add(1, std::memory_order_relaxed);
			}
		}
	});

	writer.join();
	reader.join();

	// 收尾一致: 最后一拍完整可达
	Payload out;
	uint64_t s = 0;
	const bool final_ok = buf.read(out, s) && out.stamp == kWriterOps;
	check(final_ok, "stress: final snapshot is last publish");
	check(dirty.load() == 0, "stress: zero dirty data");
	std::printf("INFO: reads=%llu misses=%llu tears=%u (写 %llu 拍)\n",
		static_cast<unsigned long long>(reads.load()),
		static_cast<unsigned long long>(misses.load()),
		buf.tears(),
		static_cast<unsigned long long>(kWriterOps));
}

// ---- readLastFrame 针对性压测 (2026-09-20: 三轮外部评审三轮边界 bug 后的防复发层) ----
// 校验三件事: ① 恒等负载一致 (撕裂) ② 帧身份自洽 seq_out == 2*stamp+2 (身份错位——
// 旧"读旧槽自证"实现曾按起点奇偶各错半, 靠此断言族抓) ③ 返回 kind 与帧号一致。
// 写者节流 ~5µs/帧 (busy 等待, 覆盖"写窗口 + 空闲窗"两种相位, 让重试路径真实命中)。

void readlast_test()
{
	SpLatest<Payload> buf;
	buf.init(Payload{0, 0, 0, 0});

	std::atomic<bool> writer_done{false};
	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> reads{0};
	std::atomic<uint64_t> nones{0};

	std::thread writer([&]()
	{
		for (uint64_t i = 1; i <= 200000; ++i)
		{
			buf.publish(Payload{i, i, i, i});
			for (volatile int32_t spin = 0; spin < 3000; ++spin)
			{
				// busy 节流 ~5µs: 写窗口 ~1% 占空比, 读者重试路径可命中成功窗
			}
		}
		writer_done.store(true, std::memory_order_release);
	});

	std::thread reader([&]()
	{
		while (!writer_done.load(std::memory_order_acquire))
		{
			Payload out;
			uint64_t s = 0;
			const auto kind = buf.readLastFrame(out, s);
			if (kind == SpLatest<Payload>::FrameKind::kNone)
			{
				nones.fetch_add(1, std::memory_order_relaxed);
				continue;
			}
			reads.fetch_add(1, std::memory_order_relaxed);
			// ① 恒等负载 (撕裂即现形)
			// ② 帧身份自洽: init 帧 (stamp=0) ↔ seq=2; 第 i 帧 (stamp=i) ↔ seq=2+2i
			// ③ kind ↔ 帧号: stamp==0 → kInit, 否则 kLive
			const bool identity = (s == 2 * out.stamp + 2);
			const auto expect = (out.stamp == 0) ? SpLatest<Payload>::FrameKind::kInit
			                                     : SpLatest<Payload>::FrameKind::kLive;
			if (out.stamp != out.a || out.a != out.b || out.b != out.c || !identity || kind != expect)
			{
				dirty.fetch_add(1, std::memory_order_relaxed);
			}
		}
	});

	writer.join();
	reader.join();

	Payload out;
	uint64_t s = 0;
	const bool final_ok = buf.readLastFrame(out, s) == SpLatest<Payload>::FrameKind::kLive
		&& out.stamp == 200000;
	check(final_ok, "readLastFrame: final frame reachable");
	check(dirty.load() == 0, "readLastFrame: zero dirty (tear/identity/kind)");
	std::printf("INFO: readLastFrame reads=%llu none=%llu tears=%u (写 200000 拍, 节流5µs)\n",
		static_cast<unsigned long long>(reads.load()),
		static_cast<unsigned long long>(nones.load()),
		buf.tears());
}

// ---- readLastFrame 极限工况: 全速背靠背写者 + 4KB 大载荷 (用户验收要求, 2026-09-20) ----
// 写者无节流连发 (写窗口占空比拉到极限 ≈ 必撞; 远超设计域——真实总线 1kHz 占空比 ~0.05%),
// 载荷 ~4KB 触发 kRetryPauseN=64 自适应档。验证目标不是成功率而是**安全性**:
// ① 零脏数据 (撕裂/身份错位 = 硬错) ② 帧身份恒自洽 (seq == 2*stamp+2) ③ 写者停后必达最终帧。

constexpr uint64_t kFatStressOps = 200000;

struct FatPayload
{
	uint64_t stamp{0};
	std::array<uint64_t, 511> blob{};   // 8 + 511×8 = 4096 (64 倍数, 恰在 4096B 级)
};
static_assert(sizeof(FatPayload) == 4096, "FatPayload 须为 4096 (64 的倍数)");
static_assert(std::is_trivially_copyable_v<FatPayload>, "FatPayload must be trivially copyable");

// 4096B + 双向 1kHz 真实总线频率 (用户裁定 2026-09-20): 写者每 1ms 发一帧, 读者每 1ms 收一帧。
// 1 万帧 ≈ 10s; 拷贝 1.5µs / 间隔 1000µs → 单次失败 ~0.3%, kNone 预期 ≈0 — 大载荷设计域结论用例。
void readlast_fat_throttled()
{
	SpLatest<FatPayload> buf;
	buf.init(FatPayload{});

	std::atomic<bool> writer_done{false};
	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> reads{0};
	std::atomic<uint64_t> nones{0};

	std::thread writer([&]()
	{
		for (uint64_t i = 1; i <= 10000; ++i)
		{
			FatPayload f;
			f.stamp = i;
			for (uint64_t & v : f.blob)
			{
				v = i;
			}
			buf.publish(f);
			for (volatile int32_t spin = 0; spin < 900000; ++spin)
			{
				// busy 节流 ~1ms = 1kHz (volatile 防优化)
			}
		}
		writer_done.store(true, std::memory_order_release);
	});

	std::thread reader([&]()
	{
		while (!writer_done.load(std::memory_order_acquire))
		{
			FatPayload out;
			uint64_t s = 0;
			const auto kind = buf.readLastFrame(out, s);
			if (kind == SpLatest<FatPayload>::FrameKind::kNone)
			{
				nones.fetch_add(1, std::memory_order_relaxed);
			}
			else
			{
				reads.fetch_add(1, std::memory_order_relaxed);
				bool ok = (s == 2 * out.stamp + 2);
				for (uint64_t v : out.blob)
				{
					if (v != out.stamp)
					{
						ok = false;
						break;
					}
				}
				const auto expect = (out.stamp == 0) ? SpLatest<FatPayload>::FrameKind::kInit
				                                     : SpLatest<FatPayload>::FrameKind::kLive;
				if (!ok || kind != expect)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
			}
			for (volatile int32_t spin = 0; spin < 900000; ++spin)
			{
				// 收帧侧同样 1ms 节拍 (双向 1kHz 同频)
			}
		}
	});

	writer.join();
	reader.join();

	FatPayload out;
	uint64_t s = 0;
	const bool final_ok = buf.readLastFrame(out, s) == SpLatest<FatPayload>::FrameKind::kLive
		&& out.stamp == 10000;
	check(final_ok, "fat-1kHz: final frame reachable");
	check(dirty.load() == 0, "fat-1kHz: zero dirty @4096B 双向1kHz真实频率");
	std::printf("INFO: fat-1kHz reads=%llu none=%llu tears=%u (双向 10000 拍 @1kHz, 载荷 %zuB)\n",
		static_cast<unsigned long long>(reads.load()),
		static_cast<unsigned long long>(nones.load()),
		buf.tears(),
		sizeof(FatPayload));
}

// ---- readLastFrame 同时启动工况 + 3 次重试 WCET 实测 (用户验收, 2026-09-20) ----
// 双线程同时启动 (无相位构造), 各自 1kHz 节拍, 4096B — 同相拍即互撞 (写者拍首 publish,
// 读者拍首 read), 相位随节拍微漂扫过整周期。记录每次调用耗时 → 3 次重试的真实最坏值。

void readlast_fat_collide()
{
	SpLatest<FatPayload> buf;
	buf.init(FatPayload{});

	std::atomic<bool> writer_done{false};
	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> reads{0};
	std::atomic<uint64_t> nones{0};

	std::thread writer([&]()
	{
		for (uint64_t i = 1; i <= 10000; ++i)
		{
			FatPayload f;
			f.stamp = i;
			for (uint64_t & v : f.blob)
			{
				v = i;
			}
			buf.publish(f);
			for (volatile int32_t spin = 0; spin < 900000; ++spin)
			{
				// busy 节流 ~1ms = 1kHz
			}
		}
		writer_done.store(true, std::memory_order_release);
	});

	std::vector<double> lat_us;   // 每次调用耗时 (测试程序, 堆上统计无妨)
	lat_us.reserve(11000);

	std::thread reader([&]()
	{
		while (!writer_done.load(std::memory_order_acquire))
		{
			FatPayload out;
			uint64_t s = 0;
			timespec t0{};
			clock_gettime(CLOCK_MONOTONIC, &t0);
			const auto kind = buf.readLastFrame(out, s);
			timespec t1{};
			clock_gettime(CLOCK_MONOTONIC, &t1);
			const double us = static_cast<double>(t1.tv_sec - t0.tv_sec) * 1.0e6
				+ static_cast<double>(t1.tv_nsec - t0.tv_nsec) / 1.0e3;
			lat_us.push_back(us);

			if (kind == SpLatest<FatPayload>::FrameKind::kNone)
			{
				nones.fetch_add(1, std::memory_order_relaxed);
			}
			else
			{
				reads.fetch_add(1, std::memory_order_relaxed);
				bool ok = (s == 2 * out.stamp + 2);
				for (uint64_t v : out.blob)
				{
					if (v != out.stamp)
					{
						ok = false;
						break;
					}
				}
				const auto expect = (out.stamp == 0) ? SpLatest<FatPayload>::FrameKind::kInit
				                                     : SpLatest<FatPayload>::FrameKind::kLive;
				if (!ok || kind != expect)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
			}
			for (volatile int32_t spin = 0; spin < 900000; ++spin)
			{
				// 读者同 1kHz 节拍 (与写者同时启动)
			}
		}
	});

	writer.join();
	reader.join();

	std::sort(lat_us.begin(), lat_us.end());
	const auto pct = [&](double p) -> double
	{
		return lat_us.empty() ? 0.0 : lat_us[static_cast<size_t>(p * static_cast<double>(lat_us.size() - 1))];
	};

	FatPayload out;
	uint64_t s = 0;
	const bool final_ok = buf.readLastFrame(out, s) == SpLatest<FatPayload>::FrameKind::kLive
		&& out.stamp == 10000;
	check(final_ok, "collide: final frame reachable");
	check(dirty.load() == 0, "collide: zero dirty @必撞工况 (每拍撞写窗)");
	std::printf("INFO: collide reads=%llu none=%llu | readLastFrame 耗时 µs: p50=%.2f p99=%.2f max=%.2f (n=%zu)\n",
		static_cast<unsigned long long>(reads.load()),
		static_cast<unsigned long long>(nones.load()),
		pct(0.50), pct(0.99), lat_us.empty() ? 0.0 : lat_us.back(), lat_us.size());
}

void readlast_stress_fat()
{
	SpLatest<FatPayload> buf;
	buf.init(FatPayload{});

	std::atomic<bool> writer_done{false};
	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> reads{0};
	std::atomic<uint64_t> nones{0};

	std::thread writer([&]()
	{
		for (uint64_t i = 1; i <= kFatStressOps; ++i)
		{
			FatPayload f;
			f.stamp = i;
			for (uint64_t & v : f.blob)
			{
				v = i;
			}
			buf.publish(f);
		}
		writer_done.store(true, std::memory_order_release);
	});

	std::thread reader([&]()
	{
		while (!writer_done.load(std::memory_order_acquire))
		{
			FatPayload out;
			uint64_t s = 0;
			const auto kind = buf.readLastFrame(out, s);
			if (kind == SpLatest<FatPayload>::FrameKind::kNone)
			{
				nones.fetch_add(1, std::memory_order_relaxed);
				continue;
			}
			reads.fetch_add(1, std::memory_order_relaxed);
			bool ok = (s == 2 * out.stamp + 2);
			for (uint64_t v : out.blob)
			{
				if (v != out.stamp)
				{
					ok = false;
					break;
				}
			}
			const auto expect = (out.stamp == 0) ? SpLatest<FatPayload>::FrameKind::kInit
			                                     : SpLatest<FatPayload>::FrameKind::kLive;
			if (!ok || kind != expect)
			{
				dirty.fetch_add(1, std::memory_order_relaxed);
			}
		}
	});

	writer.join();
	reader.join();

	FatPayload out;
	uint64_t s = 0;
	const bool final_ok = buf.readLastFrame(out, s) == SpLatest<FatPayload>::FrameKind::kLive
		&& out.stamp == kFatStressOps;
	check(final_ok, "fat-stress: final frame reachable after writer stops");
	check(dirty.load() == 0, "fat-stress: zero dirty @4KB full-speed writer (必撞场景)");
	std::printf("INFO: fat-stress reads=%llu none=%llu tears=%u (写 %llu 拍全速, 载荷 %zuB)\n",
		static_cast<unsigned long long>(reads.load()),
		static_cast<unsigned long long>(nones.load()),
		buf.tears(),
		static_cast<unsigned long long>(kFatStressOps),
		sizeof(FatPayload));
}

// ---- 双通道全双工测试: 模拟真实工况 (主站 ↔ CM, 两侧等速 1kHz, 各持一个 buf) ----

struct BusCmd    // CM → 主站: 控制值通道 (自校验负载)
{
	uint64_t stamp{0};
	std::array<uint64_t, 16> joints{};   // 全部 == stamp, 模拟 16 关节命令
};

struct BusState  // 主站 → CM: 反馈值通道 (含 cmd 回显, 闭环完整性校验)
{
	uint64_t stamp{0};
	uint64_t echo_cmd{0};                // 主站最近见到的 cmd.stamp —— 回程探针
	std::array<uint64_t, 16> joints{};   // 全部 == stamp, 模拟 16 关节测量
};

constexpr int kDuplexHz = 1000;                // 两侧同速 1kHz (真实总线节拍)
constexpr int kDuplexCycles = 100000;          // 10 万拍 ≈ 100s (日常回归); 100 万拍 ≈ 17min 长跑 soak 按需改
constexpr uint64_t kRoundTripTolerance = 100;  // 收尾回显容差 (拍, 吸收起停抖动, 0.1%)

bool consistent(const BusCmd & c)
{
	for (uint64_t j : c.joints)
	{
		if (j != c.stamp)
		{
			return false;
		}
	}
	return true;
}

bool consistent(const BusState & s)
{
	for (uint64_t j : s.joints)
	{
		if (j != s.stamp)
		{
			return false;
		}
	}
	return true;
}

void duplex_test()
{
	SpLatest<BusCmd> cmd_ex;      // CM 写 / 主站读
	SpLatest<BusState> state_ex;  // 主站写 / CM 读

	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> cm_reads{0};
	std::atomic<uint64_t> master_reads{0};

	// CM 线程: 写控制 → 读反馈 (与真实 ros2_control 周期同构)
	std::thread cm([&]()
	{
		prctl(PR_SET_TIMERSLACK, 1, 0, 0, 0);   // 收紧定时器松弛 (默认 50µs slack 毁节拍)
		auto next = std::chrono::steady_clock::now();
		for (int i = 1; i <= kDuplexCycles; ++i)
		{
			next += std::chrono::milliseconds(1000 / kDuplexHz);
			BusCmd c;
			c.stamp = i;
			for (uint64_t & j : c.joints)
			{
				j = i;
			}
			cmd_ex.publish(c);

			BusState st;
			uint64_t s = 0;
			if (state_ex.read(st, s))
			{
				cm_reads.fetch_add(1, std::memory_order_relaxed);
				if (!consistent(st) || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
			}
			std::this_thread::sleep_until(next);
		}
	});

	// 主站线程: 读控制 → 发反馈 (回显最近命令 = 闭环探针)
	std::thread master([&]()
	{
		prctl(PR_SET_TIMERSLACK, 1, 0, 0, 0);   // 收紧定时器松弛
		auto next = std::chrono::steady_clock::now();
		uint64_t last_cmd = 0;
		for (int i = 1; i <= kDuplexCycles; ++i)
		{
			next += std::chrono::milliseconds(1000 / kDuplexHz);
			BusCmd c;
			uint64_t s = 0;
			if (cmd_ex.read(c, s))
			{
				master_reads.fetch_add(1, std::memory_order_relaxed);
				if (!consistent(c) || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
				last_cmd = c.stamp;
			}
			BusState st;
			st.stamp = i;
			st.echo_cmd = last_cmd;
			for (uint64_t & j : st.joints)
			{
				j = i;
			}
			state_ex.publish(st);
			std::this_thread::sleep_until(next);
		}
	});

	cm.join();
	master.join();

	// 闭环完整性: 主站最终回显的命令号 ≈ 发送总数 (差容忍内 = 每条命令都走完了全程)
	BusState fin;
	uint64_t fs = 0;
	const bool rt = state_ex.read(fin, fs) && fin.echo_cmd + kRoundTripTolerance >= kDuplexCycles;
	check(rt, "duplex: cmd round-trip intact (echo ~= N)");
	check(dirty.load() == 0, "duplex: zero dirty data @1kHz equal-rate x100k");
	std::printf("INFO: cm_reads=%llu master_reads=%llu tears(cmd/state)=%u/%u\n",
		static_cast<unsigned long long>(cm_reads.load()),
		static_cast<unsigned long long>(master_reads.load()),
		cmd_ex.tears(),
		state_ex.tears());
}

// ---- 64 关节大载荷 soak: 1M 拍对称全双工 (64 = 框架契约上界 kMaxJoints, 载荷 ~528B) ----

constexpr uint64_t kBigJoints = 64;      // 与 sim_control 契约常量 kMaxJoints 同值
constexpr uint64_t kBigCycles = 1000000; // 每侧 1M 拍

struct BusCmdBig
{
	uint64_t stamp{0};
	std::array<uint64_t, kBigJoints> joints{};   // 载荷 ~520B, 跨多条缓存行
};

struct BusStateBig
{
	uint64_t stamp{0};
	uint64_t echo_cmd{0};
	std::array<uint64_t, kBigJoints> joints{};
};

template <size_t N>
bool all_match(uint64_t stamp, const std::array<uint64_t, N> & joints)
{
	for (uint64_t j : joints)
	{
		if (j != stamp)
		{
			return false;
		}
	}
	return true;
}

bool consistent(const BusCmdBig & c)
{
	return all_match(c.stamp, c.joints);
}

bool consistent(const BusStateBig & s)
{
	return all_match(s.stamp, s.joints);
}

void duplex_test_big()
{
	SpLatest<BusCmdBig> cmd_ex;      // CM 写 / 主站读
	SpLatest<BusStateBig> state_ex;  // 主站写 / CM 读

	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> cm_reads{0};
	std::atomic<uint64_t> master_reads{0};

	// 对称无间隔轰击: 两线程每拍做完全相同的 publish+read —— 构造上保证"同速、读写相当"
	std::thread cm([&]()
	{
		for (uint64_t i = 1; i <= kBigCycles; ++i)
		{
			BusCmdBig c;
			c.stamp = i;
			for (uint64_t & j : c.joints)
			{
				j = i;
			}
			cmd_ex.publish(c);

			BusStateBig st;
			uint64_t s = 0;
			if (state_ex.read(st, s))
			{
				cm_reads.fetch_add(1, std::memory_order_relaxed);
				if (!consistent(st) || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
			}
		}
	});

	std::thread master([&]()
	{
		uint64_t i = 0;
		uint64_t last_cmd = 0;
		uint64_t guard = 0;
		// 终止条件 = 读到最终命令 (而非固定迭代数): 调度器亏待一侧时仍保证闭环完整,
		// 回显断言因此度量原语而非调度公平性 (踩坑: 固定迭代数版曾因线程推进不均误报)
		while (last_cmd < kBigCycles && guard < kBigCycles * 4)
		{
			++guard;
			++i;
			BusCmdBig c;
			uint64_t s = 0;
			if (cmd_ex.read(c, s))
			{
				master_reads.fetch_add(1, std::memory_order_relaxed);
				if (!consistent(c) || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
				last_cmd = c.stamp;
			}
			BusStateBig st;
			st.stamp = i;
			st.echo_cmd = last_cmd;
			for (uint64_t & j : st.joints)
			{
				j = i;
			}
			state_ex.publish(st);
		}
	});

	cm.join();
	master.join();

	BusStateBig fin;
	uint64_t fs = 0;
	const bool rt = state_ex.read(fin, fs) && fin.echo_cmd == kBigCycles;
	check(rt, "big: cmd round-trip complete (echo == N, 64 joints x1M)");
	check(dirty.load() == 0, "big: zero dirty data (528B payload)");
	std::printf("INFO: payload=%zuB cm_reads=%llu master_reads=%llu tears(cmd/state)=%u/%u\n",
		sizeof(BusStateBig),
		static_cast<unsigned long long>(cm_reads.load()),
		static_cast<unsigned long long>(master_reads.load()),
		cmd_ex.tears(),
		state_ex.tears());
}

// ---- 电机域结构测试: 32 电机真实契约 (控制 MIT 五元组 / 反馈含温度与故障域) ----

struct MotorCmd    // 单电机控制: MIT 模式五元组
{
	double pos{0.0};    // 目标位置 (rad)
	double vel{0.0};    // 目标速度 (rad/s)
	double tff{0.0};    // 前馈力矩 (N·m)
	double kp{0.0};     // 位置增益
	double kd{0.0};     // 速度增益
};

struct MotorState   // 单电机反馈: 运动量 + 热量 + 故障域
{
	double pos{0.0};         // 实测位置 (rad)
	double vel{0.0};         // 实测速度 (rad/s)
	double tff{0.0};         // 实测力矩 (N·m)
	float mos_temp{0.0f};    // MOS 温度 (℃)
	float rin_temp{0.0f};    // 绕组温度 (℃)
	uint32_t error_code{0};  // 驱动器故障码 (0 = 正常)
	uint32_t status{0};      // 驱动器状态字
};

struct BusCmd32     // CM → 主站: 32 电机控制帧
{
	uint64_t stamp{0};
	std::array<MotorCmd, 32> motors{};
};

struct BusState32   // 主站 → CM: 32 电机反馈帧
{
	uint64_t stamp{0};
	uint64_t echo_cmd{0};
	std::array<MotorState, 32> motors{};
};

static_assert(std::is_trivially_copyable_v<BusCmd32>, "BusCmd32 must be trivially copyable");
static_assert(std::is_trivially_copyable_v<BusState32>, "BusState32 must be trivially copyable");

constexpr int kMotorHz = 1000;          // 两侧等速 1kHz (真实总线节拍)
constexpr int kMotorCycles = 100000;    // 10 万拍 ≈ 100s (日常回归); 100 万拍长跑按需改

// 确定性生成器: 双方对同一 (stamp, index) 必算出位级相同的值 —— 一致性校验的基准
double gen_pos(uint64_t s, size_t i)
{
	return static_cast<double>((s * 7 + i) % 6283) * 1e-4;
}

double gen_vel(uint64_t s, size_t i)
{
	return static_cast<double>((s * 3 + i) % 1000) * 1e-3;
}

double gen_tff(uint64_t s, size_t i)
{
	return static_cast<double>((s + i * 5) % 500) * 1e-3;
}

double gen_kp(uint64_t s)
{
	return 50.0 + static_cast<double>(s % 10) * 0.1;
}

double gen_kd(uint64_t s)
{
	return 1.0 + static_cast<double>(s % 5) * 0.01;
}

float gen_mos(uint64_t s, size_t i)
{
	return 40.0f + static_cast<float>((s + i) % 300) * 0.1f;   // 40~70 ℃
}

float gen_rin(uint64_t s, size_t i)
{
	return 45.0f + static_cast<float>((s * 2 + i) % 500) * 0.1f;   // 45~95 ℃
}

uint32_t gen_err(uint64_t s, size_t i)
{
	return static_cast<uint32_t>((s + i) % 7);   // 0 = 正常, 其余故障码轮转
}

uint32_t gen_status(uint64_t s)
{
	return static_cast<uint32_t>(0xC0DE0 + s % 3);
}

void fill_cmd(BusCmd32 & c, uint64_t s)
{
	c.stamp = s;
	for (size_t i = 0; i < c.motors.size(); ++i)
	{
		MotorCmd & m = c.motors[i];
		m.pos = gen_pos(s, i);
		m.vel = gen_vel(s, i);
		m.tff = gen_tff(s, i);
		m.kp = gen_kp(s);
		m.kd = gen_kd(s);
	}
}

void fill_state(BusState32 & st, uint64_t s, uint64_t echo)
{
	st.stamp = s;
	st.echo_cmd = echo;
	for (size_t i = 0; i < st.motors.size(); ++i)
	{
		MotorState & m = st.motors[i];
		m.pos = gen_pos(s, i);
		m.vel = gen_vel(s, i);
		m.tff = gen_tff(s, i);
		m.mos_temp = gen_mos(s, i);
		m.rin_temp = gen_rin(s, i);
		m.error_code = gen_err(s, i);
		m.status = gen_status(s);
	}
}

bool consistent(const BusCmd32 & c)
{
	for (size_t i = 0; i < c.motors.size(); ++i)
	{
		const MotorCmd & m = c.motors[i];
		if (m.pos != gen_pos(c.stamp, i) || m.vel != gen_vel(c.stamp, i) || m.tff != gen_tff(c.stamp, i) ||
			m.kp != gen_kp(c.stamp) || m.kd != gen_kd(c.stamp))
		{
			return false;
		}
	}
	return true;
}

bool consistent(const BusState32 & st)
{
	for (size_t i = 0; i < st.motors.size(); ++i)
	{
		const MotorState & m = st.motors[i];
		if (m.pos != gen_pos(st.stamp, i) || m.vel != gen_vel(st.stamp, i) || m.tff != gen_tff(st.stamp, i) ||
			m.mos_temp != gen_mos(st.stamp, i) || m.rin_temp != gen_rin(st.stamp, i) ||
			m.error_code != gen_err(st.stamp, i) || m.status != gen_status(st.stamp))
		{
			return false;
		}
	}
	return true;
}

void duplex_test_motors()
{
	SpLatest<BusCmd32> cmd_ex;      // CM 写 / 主站读
	SpLatest<BusState32> state_ex;  // 主站写 / CM 读

	std::atomic<uint64_t> dirty{0};
	std::atomic<uint64_t> cm_reads{0};
	std::atomic<uint64_t> master_reads{0};
	std::atomic<float> max_mos{0.0f};
	std::atomic<float> max_rin{0.0f};

	// CM 线程: 写 32 电机控制 → 读 32 电机反馈 (含温度监控)
	std::thread cm([&]()
	{
		prctl(PR_SET_TIMERSLACK, 1, 0, 0, 0);   // 收紧定时器松弛
		auto next = std::chrono::steady_clock::now();
		for (int i = 1; i <= kMotorCycles; ++i)
		{
			next += std::chrono::milliseconds(1000 / kMotorHz);
			BusCmd32 c;
			fill_cmd(c, static_cast<uint64_t>(i));
			cmd_ex.publish(c);

			BusState32 st;
			uint64_t s = 0;
			if (state_ex.read(st, s))
			{
				cm_reads.fetch_add(1, std::memory_order_relaxed);
				if (!consistent(st) || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
				for (const MotorState & m : st.motors)   // 温度巡检 (域语义, 非 int 一致性)
				{
					if (m.mos_temp > max_mos.load(std::memory_order_relaxed))
					{
						max_mos.store(m.mos_temp, std::memory_order_relaxed);
					}
					if (m.rin_temp > max_rin.load(std::memory_order_relaxed))
					{
						max_rin.store(m.rin_temp, std::memory_order_relaxed);
					}
				}
			}
			std::this_thread::sleep_until(next);
		}
	});

	// 主站线程: 读 32 电机控制 → 发 32 电机反馈 (回显命令号)
	std::thread master([&]()
	{
		prctl(PR_SET_TIMERSLACK, 1, 0, 0, 0);   // 收紧定时器松弛
		auto next = std::chrono::steady_clock::now();
		uint64_t last_cmd = 0;
		for (int i = 1; i <= kMotorCycles; ++i)
		{
			next += std::chrono::milliseconds(1000 / kMotorHz);
			BusCmd32 c;
			uint64_t s = 0;
			if (cmd_ex.read(c, s))
			{
				master_reads.fetch_add(1, std::memory_order_relaxed);
				if (!consistent(c) || (s & 1u) != 0u)
				{
					dirty.fetch_add(1, std::memory_order_relaxed);
				}
				last_cmd = c.stamp;
			}
			BusState32 st;
			fill_state(st, static_cast<uint64_t>(i), last_cmd);
			state_ex.publish(st);
			std::this_thread::sleep_until(next);
		}
	});

	cm.join();
	master.join();

	BusState32 fin;
	uint64_t fs = 0;
	const bool rt = state_ex.read(fin, fs) && fin.echo_cmd + kRoundTripTolerance >= kMotorCycles;
	check(rt, "motors: cmd round-trip intact (32 motors, x100k @1kHz)");
	check(dirty.load() == 0, "motors: zero dirty data (MIT cmd + temp/err state)");
	std::printf("INFO: payload cmd=%zuB state=%zuB cm_reads=%llu master_reads=%llu tears=%u/%u | "
		"域数据抽检: max_mos=%.1f℃ max_rin=%.1f℃ motor0: err=%u status=0x%X\n",
		sizeof(BusCmd32), sizeof(BusState32),
		static_cast<unsigned long long>(cm_reads.load()),
		static_cast<unsigned long long>(master_reads.load()),
		cmd_ex.tears(),
		state_ex.tears(),
		static_cast<double>(max_mos.load()),
		static_cast<double>(max_rin.load()),
		fin.motors[0].error_code,
		fin.motors[0].status);
}

}  // namespace

int main()
{
	functional_tests();
	stress_test();
	readlast_test();
	readlast_fat_throttled();
	readlast_fat_collide();
	readlast_stress_fat();
	duplex_test();
	duplex_test_big();
	duplex_test_motors();
	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

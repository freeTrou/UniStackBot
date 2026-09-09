// SpLatest 功能测试 + 撕裂压测 (规范 42 交付门槛)。
// 独立编译, 零 ROS 依赖:
//   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion test_sp_latest.cpp -o test_sp_latest && ./test_sp_latest
// (-Wconversion 抓类型截断 —— uint64 化时 s0/s1 曾漏改, 靠评审抓出; 此 flag 让编译器兜底)
// 核心断言: 写者全速轰击下读者**零脏数据** (撕裂由重试消化, 沿用旧值是契约不是失败)。

#include "sp_latest.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <sys/prctl.h>
#include <thread>

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
	duplex_test();
	duplex_test_big();
	duplex_test_motors();
	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

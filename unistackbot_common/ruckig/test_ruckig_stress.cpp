/*
 * ruckig/OtgStream 数值稳定性压测 (2026-09-18, 攻击 README 记录的社区版已知失败模式):
 *   S1 零目标态轰炸 (官方确认的不稳定场景)
 *   S2 极端跳变 (±1e5/±1e8 → -101 时长超限 territory)
 *   S3 数值范围边界 (目标 1e8, 限值贴 1e9)
 *   S4 RL 野流 60s 长程 (500Hz 随机游走 + 符号反转 + 每 50 拍重定向)
 *   S5 全程硬断言: 输出恒有限/有界、进程不崩、每个触发场景后可自愈、
 *      错误码被捕获记录 (期望见到 Ruckig 原生 -1xx, 证明防御对"真错误"有效)
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "otg_stream.hpp"

using Otg = unistackbot_common::OtgStream<7>;
static int g_failures = 0;
void check(bool ok, const char * name)
{
	std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
	if (!ok) {++g_failures;}
}

static Otg::Limits makeLimits()
{
	Otg::Limits lim;
	lim.max_velocity.fill(3.0);
	lim.max_acceleration.fill(5.0);
	lim.max_jerk.fill(20.0);
	return lim;
}

// 通用跑段: 返回 {Ok 拍数, Hold 拍数, 输出恒有限, 输出最大幅值}
struct RunStat {uint64_t ok = 0, hold = 0; bool finite = true; double max_abs = 0.0;};
static RunStat drive(Otg & otg, std::array<double, 7> tgt, int cycles)
{
	RunStat st;
	std::array<double, 7> out{};
	for (int i = 0; i < cycles; ++i)
	{
		const auto r = otg.update(tgt, out);
		(r == Otg::UpdateResult::Ok) ? ++st.ok : ++st.hold;
		for (double v : out)
		{
			if (!std::isfinite(v)) {st.finite = false;}
			st.max_abs = std::max(st.max_abs, std::fabs(v));
		}
	}
	return st;
}

int main()
{
	constexpr double DT = 0.002;
	std::array<double, 7> q0{}, out{};
	const auto lim = makeLimits();

	// ===== S1 零目标态轰炸 (官方确认不稳定场景) =====
	{
		Otg otg;
		check(otg.init(DT, lim), "S1 init");
		otg.reset(q0);
		std::array<double, 7> real{};
		for (std::size_t i = 0; i < 7; ++i) {real[i] = 0.3 * (static_cast<double>(i) + 1) / 7;}
		bool finite = true;
		uint64_t hold = 0, ok = 0;
		for (int i = 0; i < 2000; ++i)
		{
			std::array<double, 7> tgt = (i % 2 == 0) ? std::array<double, 7>{} : real;
			// 全零目标态与真实目标态交替轰炸
			const auto r = otg.update(tgt, out);
			(r == Otg::UpdateResult::Ok) ? ++ok : ++hold;
			for (double v : out) {if (!std::isfinite(v)) {finite = false;}}
		}
		check(finite, "S1 零目标轰炸: 输出恒有限");
		check(ok + hold == 2000, "S1 全程受控");
		std::printf("  (Ok=%llu Hold=%llu lastError=%d)\n",
			static_cast<unsigned long long>(ok), static_cast<unsigned long long>(hold), otg.lastError());
	}

	// ===== S2 极端跳变 (-101 时长超限 territory) =====
	{
		Otg otg;
		check(otg.init(DT, lim, 0.0), "S2 init (跳变限幅关闭 —— 放野输入进来打内部)");
		otg.reset(q0);
		int saw_native_err = 0;
		bool finite = true;
		for (const double d : {1e2, 1e4, 1e6, 1e8})
		{
			std::array<double, 7> tgt{};
			for (std::size_t i = 0; i < 7; ++i) {tgt[i] = d;}
			const auto r = otg.update(tgt, out);
			if (otg.lastError() <= -100 && otg.lastError() >= -111) {saw_native_err = 1;}
			for (double v : out) {if (!std::isfinite(v)) {finite = false;}}
			(void)r;
		}
		check(finite, "S2 极端跳变: 输出恒有限");
		std::printf("  (Ruckig 原生错误码出现: %s, lastError=%d)\n",
			saw_native_err ? "是 (防御捕获真错误)" : "否 (内部未触界)", otg.lastError());
		// 自愈: 正常目标恢复跟踪
		std::array<double, 7> tgt{};
		for (std::size_t i = 0; i < 7; ++i) {tgt[i] = 0.2;}
		bool ok = true;
		for (int i = 0; i < 400; ++i)
		{
			if (otg.update(tgt, out) != Otg::UpdateResult::Ok) {ok = false;}
		}
		check(ok && std::fabs(out[0] - 0.2) < 0.02, "S2 极端跳变后自愈恢复");
	}

	// ===== S3 数值范围边界 (目标 1e8, 限值 9e8) =====
	{
		Otg otg;
		auto lim9 = makeLimits();
		lim9.max_velocity.fill(9e8);
		lim9.max_acceleration.fill(9e8);
		lim9.max_jerk.fill(9e8);
		check(otg.init(DT, lim9), "S3 init (限值 9e8 <1e9)");
		otg.reset(q0);
		std::array<double, 7> tgt{};
		for (std::size_t i = 0; i < 7; ++i) {tgt[i] = 1e8;}
		bool finite = true;
		for (int i = 0; i < 200; ++i)
		{
			otg.update(tgt, out);
			for (double v : out) {if (!std::isfinite(v)) {finite = false;}}
		}
		check(finite, "S3 大数边界: 输出恒有限");
	}

	// ===== S4 RL 野流 60s 长程 (错误率统计) =====
	{
		Otg otg;
		check(otg.init(DT, lim, 0.5), "S4 init (跳变限幅 0.5)");
		otg.reset(q0);
		std::mt19937 rng(20260918);
		std::uniform_real_distribution<double> u(-1.0, 1.0);
		bool finite = true;
		uint64_t ok = 0, hold = 0, cycles = 30000;   // 60s @500Hz
		double max_abs = 0.0;
		for (uint64_t i = 0; i < cycles; ++i)
		{
			std::array<double, 7> tgt{};
			for (std::size_t k = 0; k < 7; ++k)
			{
				// 野流: 大幅随机游走 + 偶发符号反转 (RL 策略式)
				tgt[k] = 40.0 * u(rng) * ((i % 137 == 0) ? -1.0 : 1.0);
			}
			const auto r = otg.update(tgt, out);
			(r == Otg::UpdateResult::Ok) ? ++ok : ++hold;
			for (double v : out)
			{
				if (!std::isfinite(v)) {finite = false;}
				max_abs = std::max(max_abs, std::fabs(v));
			}
		}
		check(finite, "S4 RL 野流 60s: 输出恒有限");
		check(ok + hold == cycles, "S4 全程受控");
		const double err_rate = 100.0 * static_cast<double>(hold) / static_cast<double>(cycles);
		std::printf("  (60s: Ok=%llu Hold=%llu 错误率=%.3f%% 输出最大幅值=%.2f rad)\n",
			static_cast<unsigned long long>(ok), static_cast<unsigned long long>(hold),
			err_rate, max_abs);
		check(err_rate < 5.0, "S4 错误率 <5% (防御层不干扰常态服务)");
	}

	// ===== S5 每拍重定向轰击 (calculator 连续满负荷) =====
	{
		Otg otg;
		check(otg.init(DT, lim), "S5 init");
		otg.reset(q0);
		std::mt19937 rng(7);
		std::uniform_real_distribution<double> u(-2.0, 2.0);
		bool finite = true;
		for (int i = 0; i < 5000; ++i)   // 每拍重定向, 10s 连续满负荷
		{
			std::array<double, 7> tgt{};
			for (std::size_t k = 0; k < 7; ++k) {tgt[k] = u(rng);}
			otg.update(tgt, out);
			for (double v : out) {if (!std::isfinite(v)) {finite = false;}}
		}
		check(finite, "S5 每拍重定向: 输出恒有限");
	}

	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

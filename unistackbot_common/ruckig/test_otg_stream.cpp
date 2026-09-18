/*
 * OtgStream 防御封装测试 (第 5 条起为封装行为断言, 对应评审防御清单):
 *   T6 非有限目标 (NaN/Inf) → Hold, 输出未动, 错误计数
 *   T7 野跳变目标 → 跳变限幅生效, 全程无 Error 码, 平滑走行
 *   T8 错误自愈: hold 后 otg.reset + 状态回退, 后续目标可恢复跟踪
 *   T9 init 参数校验: 非法限值 (负/零/超 1e9/NaN) 拒绝
 *   T10 观测: update/error 计数与 lastError 语义
 * (T1-T5 库级测试见 test_ruckig.cpp)
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "otg_stream.hpp"

static int g_failures = 0;
void check(bool ok, const char * name)
{
	std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
	if (!ok) {++g_failures;}
}

int main()
{
	constexpr std::size_t N = 7;
	constexpr double DT = 0.002;
	unistackbot_common::OtgStream<N>::Limits lim;
	lim.max_velocity.fill(3.0);
	lim.max_acceleration.fill(5.0);
	lim.max_jerk.fill(20.0);

	// ===== T6 非有限目标 =====
	{
		unistackbot_common::OtgStream<N> otg;
		check(otg.init(DT, lim), "T6 init");
		std::array<double, N> q0{};
		otg.reset(q0);
		std::array<double, N> tgt = q0;
		tgt[0] = std::nan("");
		std::array<double, N> out{};
		for (int i = 0; i < 5; ++i)
		{
			const auto r = otg.update(tgt, out);
			check(r == unistackbot_common::OtgStream<N>::UpdateResult::Hold, "T6 NaN 目标 → Hold");
		}
		check(otg.errorCount() == 5, "T6 NaN 错误计数 5");
		check(otg.lastError() == unistackbot_common::OtgStream<N>::NonFiniteTarget, "T6 lastError=NonFiniteTarget");
		// 修订契约: Hold 时输出恒有效 = 安全保持位 (调用方无条件写硬件即可)
		bool held_valid = true;
		for (std::size_t i = 0; i < N; ++i) {held_valid &= (std::fabs(out[i]) < 1e-12);}
		check(held_valid, "T6 Hold 输出 = 安全保持位");
		// 恢复正常目标 → Ok
		for (std::size_t i = 0; i < N; ++i) {tgt[i] = 0.3 * (static_cast<double>(i) + 1) / 7;}
		const auto r = otg.update(tgt, out);
		check(r == unistackbot_common::OtgStream<N>::UpdateResult::Ok, "T6 恢复正常目标 → Ok");
	}

	// ===== T7 野跳变目标 (RL 式) + 无 Error 码 =====
	{
		unistackbot_common::OtgStream<N> otg;
		check(otg.init(DT, lim, 0.3), "T7 init (跳变限幅 0.3)");
		std::array<double, N> q0{};
		otg.reset(q0);
		std::array<double, N> out{};
		// RL 式狂野目标: 每拍 ±50 rad 跳变
		double t = 0.0;
		bool ok_noerr = true, ok_bound = true;
		std::array<double, N> prev{};
		double worst_step = 0.0;
		for (int i = 0; i < 2500; ++i)   // 5s
		{
			t = i * DT;
			std::array<double, N> tgt{};
			for (std::size_t k = 0; k < N; ++k)
			{
				tgt[k] = 50.0 * std::sin(13.0 * t + static_cast<double>(k));   // 野
			}
			const auto r = otg.update(tgt, out);
			if (r != unistackbot_common::OtgStream<N>::UpdateResult::Ok) {ok_noerr = false;}
			for (std::size_t k = 0; k < N; ++k)
			{
				worst_step = std::max(worst_step, std::fabs(out[k] - prev[k]));
				if (!std::isfinite(out[k]) || std::fabs(out[k]) > 100.0) {ok_bound = false;}
				prev[k] = out[k];
			}
		}
		check(ok_noerr, "T7 野输入全程无 Error (跳变限幅+内部稳定)");
		check(ok_bound, "T7 输出有界且有限");
		std::printf("  (最差输出步进 %.4f rad/拍 = %.1f rad/s 界内)\n", worst_step, worst_step / DT);
		check(otg.errorCount() == 0, "T7 零错误计数");
	}

	// ===== T8 错误自愈 =====
	{
		unistackbot_common::OtgStream<N> otg;
		check(otg.init(DT, lim), "T8 init");
		std::array<double, N> q0{};
		otg.reset(q0);
		std::array<double, N> tgt{};
		tgt[0] = std::nan("");
		std::array<double, N> out{};
		for (int i = 0; i < 3; ++i) {otg.update(tgt, out);}
		// 自愈后正常目标可恢复
		for (std::size_t i = 0; i < N; ++i) {tgt[i] = 0.2;}
		bool ok = true;
		for (int i = 0; i < 300; ++i)
		{
			const auto r = otg.update(tgt, out);
			if (r != unistackbot_common::OtgStream<N>::UpdateResult::Ok) {ok = false;}
		}
		check(ok && std::fabs(out[0] - 0.2) < 0.01, "T8 错误后自愈恢复跟踪");
	}

	// ===== T9 init 参数校验 =====
	{
		unistackbot_common::OtgStream<N> otg;
		auto bad = lim;
		bad.max_velocity[2] = -1.0;
		check(!otg.init(DT, bad), "T9 负速度限拒绝");
		bad = lim; bad.max_jerk[0] = 2e9;
		check(!otg.init(DT, bad), "T9 超 1e9 拒绝");
		bad = lim; bad.max_acceleration[4] = std::nan("");
		check(!otg.init(DT, bad), "T9 NaN 限值拒绝");
		check(otg.init(DT, lim), "T9 合法限值通过");
	}

	// ===== T10 观测计数 =====
	{
		unistackbot_common::OtgStream<N> otg;
		check(otg.init(DT, lim), "T10 init");
		std::array<double, N> q0{};
		otg.reset(q0);
		std::array<double, N> tgt{};
		tgt[0] = 0.5;
		std::array<double, N> out{};
		const uint64_t u0 = otg.updateCount();
		for (int i = 0; i < 10; ++i) {otg.update(tgt, out);}
		check(otg.updateCount() == u0 + 10, "T10 update 计数");
		check(otg.errorCount() == 0, "T10 零错误");
	}

	// ===== T11 健康指标 (二轮评审): duration + 速度/加速度访问器 =====
	{
		unistackbot_common::OtgStream<N> otg;
		check(otg.init(DT, lim, 0.1), "T11 init");
		std::array<double, N> q0{};
		otg.reset(q0);
		std::array<double, N> tgt{};
		tgt[0] = 0.5;
		std::array<double, N> out{};
		double dur_mid = -1.0;
		// 全程 ~0.92s (初期目标爬移 + jerk 限速): 600 拍确保到位
		bool all_ok = true;
		for (int i = 0; i < 600; ++i)
		{
			if (otg.update(tgt, out) != unistackbot_common::OtgStream<N>::UpdateResult::Ok)
			{
				all_ok = false;
			}
			if (i == 75) {dur_mid = otg.lastDuration();}
		}
		check(all_ok, "T11 全程 Ok");
		check(otg.lastDuration() < 0.05, "T11 到位后剩余时长趋零");
		check(dur_mid > 0.0, "T11 行进中剩余时长为正");
		const auto v = otg.lastVelocity();
		const auto a = otg.lastAcceleration();
		bool fin = true;
		for (std::size_t i = 0; i < N; ++i) {fin &= std::isfinite(v[i]) && std::isfinite(a[i]);}
		check(fin, "T11 速度/加速度访问器有限");
	}

	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

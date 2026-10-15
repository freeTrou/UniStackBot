/* VelocityStream 单元测试 (2026-10-15, 底盘域 A1)。
 * g++ -std=c++17 -O2 -Wall -Wextra -I include test_velocity_stream.cpp \
 *     (ruckig 全源: src/ruckig 下 .cpp 一并编入) -o /tmp/t_vs && /tmp/t_vs
 *
 * 覆盖: init 校验 / 阶跃跟随 (a 界) / 目标反转 S 曲线 (j 界) / 减速到零 /
 *       NaN 拒绝 (Hold 恒有效) / 幅值钳 / 重锚 / 观测计数 */
#include <cmath>
#include <cstdio>

#include "otg_stream.hpp"

using unistackbot_common::VelocityStream;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { if (cond) { ++g_pass; } else { ++g_fail; std::printf("FAIL: %s\n", msg); } } while (0)

int main()
{
	constexpr double dt = 0.002;   // 500Hz
	constexpr double a_max = 2.0, j_max = 10.0, v_max = 1.5;
	VelocityStream<3>::Limits lim{};
	lim.max_velocity.fill(v_max);
	lim.max_acceleration.fill(a_max);
	lim.max_jerk.fill(j_max);

	// ---- init 校验: 负值/非有限/越 1e9 拒; dt 非法拒 ----
	{
		VelocityStream<3> s;
		auto bad = lim; bad.max_acceleration[1] = -1.0;
		CHECK(!s.init(dt, bad), "init 拒负加速度");
		auto big = lim; big.max_velocity[2] = 1e10;
		CHECK(!s.init(dt, big), "init 拒超 1e9");
		CHECK(!s.init(0.0, lim), "init 拒 dt=0");
		CHECK(s.init(dt, lim), "init 合法通过");
	}

	VelocityStream<3> s;
	CHECK(s.init(dt, lim), "主实例 init");
	std::array<double, 3> zero{};

	// ---- 未 reset 前调用: Hold 恒有效 (输出零) ----
	{
		std::array<double, 3> out{1, 1, 1};
		CHECK(s.update(zero, out) == VelocityStream<3>::UpdateResult::Hold, "未 reset Hold");
		CHECK(s.lastError() == VelocityStream<3>::NotInitialized, "错误码 NotInitialized");
	}

	// ---- 阶跃: 0→1.0 m/s, 2000 拍; 断言 a 界 / 收敛 / 不越目标 ----
	s.reset(zero);
	{
		std::array<double, 3> out{};
		double vprev = 0.0, max_dv = 0.0;
		for (int k = 0; k < 2000; ++k)
		{
			std::array<double, 3> tgt{1.0, 0.0, 0.0};
			CHECK(s.update(tgt, out) == VelocityStream<3>::UpdateResult::Ok, "阶跃 Ok");
			const double dv = std::fabs(out[0] - vprev);
			if (dv > max_dv) { max_dv = dv; }
			vprev = out[0];
		}
		CHECK(max_dv / dt <= a_max * 1.15, "加速度界 (含离散化余量)");
		CHECK(std::fabs(out[0] - 1.0) < 1e-3, "阶跃收敛到目标");
		CHECK(std::fabs(out[1]) < 1e-12 && std::fabs(out[2]) < 1e-12, "正交分量零");
	}

	// ---- 目标反转 1.0→-1.0: j 界 (加速度变化率受限) + 无速度跳变 ----
	{
		std::array<double, 3> out{};
		double aprev = 0.0, max_da = 0.0, max_dv = 0.0, vprev = 1.0;
		for (int k = 0; k < 3000; ++k)
		{
			std::array<double, 3> tgt{-1.0, 0.0, 0.0};
			CHECK(s.update(tgt, out) == VelocityStream<3>::UpdateResult::Ok, "反转 Ok");
			const auto a = s.lastAcceleration();
			const double dv = std::fabs(out[0] - vprev);
			const double da = std::fabs(a[0] - aprev);
			if (dv > max_dv) { max_dv = dv; }
			if (da > max_da) { max_da = da; }
			vprev = out[0]; aprev = a[0];
		}
		CHECK(max_dv / dt <= a_max * 1.15, "反转加速度界");
		CHECK(max_da / dt <= j_max * 1.5, "反转加加速度界 (含离散化余量)");
		CHECK(std::fabs(out[0] + 1.0) < 1e-3, "反转收敛到 -1.0");
	}

	// ---- 减速到零 (cmd 超时语义的上游件): 断言停得住 ----
	{
		std::array<double, 3> out{};
		for (int k = 0; k < 2000; ++k)
		{
			(void)s.update(zero, out);
		}
		CHECK(std::fabs(out[0]) < 1e-6, "减速到零");
	}

	// ---- NaN 拒绝: Hold 输出 = 上一拍安全值, 计数走账 ----
	{
		std::array<double, 3> out{};
		std::array<double, 3> ok_tgt{0.5, 0.0, 0.0};
		for (int k = 0; k < 500; ++k) { (void)s.update(ok_tgt, out); }
		const double safe = out[0];
		const uint64_t errs = s.errorCount();
		std::array<double, 3> nan_tgt{std::nan(""), 0.0, 0.0};
		CHECK(s.update(nan_tgt, out) == VelocityStream<3>::UpdateResult::Hold, "NaN Hold");
		CHECK(std::fabs(out[0] - safe) < 1e-12, "Hold 输出=安全保持值");
		CHECK(s.errorCount() == errs + 1, "错误计数+1");
		CHECK(s.lastError() == VelocityStream<3>::NonFiniteTarget, "错误码 NonFiniteTarget");
		for (int k = 0; k < 50; ++k) { (void)s.update(ok_tgt, out); }   // 自愈: 合法目标恢复 Ok
		CHECK(s.update(ok_tgt, out) == VelocityStream<3>::UpdateResult::Ok, "NaN 后自愈 Ok");
	}

	// ---- 幅值钳: 100 倍越界目标 → 轨迹不越 ±v_max ----
	{
		std::array<double, 3> out{};
		s.reset(zero);
		std::array<double, 3> big{150.0, -150.0, 0.0};
		for (int k = 0; k < 3000; ++k)
		{
			CHECK(s.update(big, out) == VelocityStream<3>::UpdateResult::Ok, "钳位目标 Ok");
			CHECK(std::fabs(out[0]) <= v_max * 1.001 && std::fabs(out[1]) <= v_max * 1.001, "输出不越界");
		}
		CHECK(std::fabs(out[0] - v_max) < 1e-3, "钳到 +v_max 收敛");
		CHECK(std::fabs(out[1] + v_max) < 1e-3, "负向钳到 -v_max 收敛");
	}

	// ---- 重锚: reset(0.5) → 下拍从 0.5 出发 (无跳变) ----
	{
		std::array<double, 3> anchor{0.5, 0.0, 0.0};
		s.reset(anchor);
		std::array<double, 3> out{};
		(void)s.update(anchor, out);
		CHECK(std::fabs(out[0] - 0.5) < 0.05, "重锚连续 (无跳变)");
	}

	std::printf("VelocityStream: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}

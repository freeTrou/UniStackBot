/*
 * ruckig 集成测试 (g++ 直编, 组件同款)。场景对齐设计文档 §"率失配填充":
 *   T1 单轴到位: 时间最优 profile, 位置/速度/加速度全程限内
 *   T2 7 轴同步: 多自由度同时到达 (xarm7 形态)
 *   T3 率失配重定向 (核心): 10Hz 慢目标流 + 500Hz 控制, 中途换目标 ×3
 *      —— 断言: 位置/速度逐拍连续 (无跳变), 速度全程限内, 最终到位
 *   T4 RT 零分配: 稳态 update 零 malloc (operator new 计数)
 *   T5 计时: update p50/p99/max (µs 级预算证据)
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <ruckig/ruckig.hpp>

static int g_failures = 0;
void check(bool ok, const char * name)
{
	std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
	if (!ok) {++g_failures;}
}

// ---- RT 零分配检测 (ulog/dls_ik 同款钩子) ----
static bool g_counting = false;
static std::size_t g_allocs = 0;
void * operator new(std::size_t sz)
{
	if (g_counting) {++g_allocs;}
	return std::malloc(sz ? sz : 1);
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

template <std::size_t N>
bool limitsHeld(const ruckig::OutputParameter<N> & o,
	const std::array<double, N> & vmax, const std::array<double, N> & amax)
{
	for (std::size_t i = 0; i < N; ++i)
	{
		if (std::fabs(o.new_velocity[i]) > vmax[i] + 1e-6) {return false;}
		if (std::fabs(o.new_acceleration[i]) > amax[i] + 1e-6) {return false;}
	}
	return true;
}

int main()
{
	constexpr double DT = 0.002;   // 500Hz

	// ===== T1 单轴到位 =====
	{
		ruckig::Ruckig<1> otg{DT};
		ruckig::InputParameter<1> in;
		ruckig::OutputParameter<1> out;
		in.current_position = {0.0};
		in.target_position = {1.0};
		in.max_velocity = {1.0};
		in.max_acceleration = {2.0};
		in.max_jerk = {10.0};
		std::array<double, 1> vmax{1.0}, amax{2.0};
		bool ok_limits = true, finished = false;
		double t_end = 0.0;
		for (int i = 0; i < 3000; ++i)   // 上限 6s
		{
			const auto r = otg.update(in, out);
			if (!limitsHeld(out, vmax, amax)) {ok_limits = false;}
			if (r == ruckig::Result::Finished)
			{
				finished = true;
				t_end = (i + 1) * DT;
				break;
			}
			in.current_position = out.new_position;
			in.current_velocity = out.new_velocity;
			in.current_acceleration = out.new_acceleration;
		}
		check(finished, "T1 单轴: 到达目标");
		check(ok_limits, "T1 单轴: 速度/加速度全程限内");
		check(std::fabs(out.new_position[0] - 1.0) < 1e-9, "T1 单轴: 终点位置误差 <1e-9");
		check(t_end > 1.0 && t_end < 3.0, "T1 单轴: 时长在时间最优量级");
	}

	// ===== T2 7 轴同步 =====
	{
		ruckig::Ruckig<7> otg{DT};
		ruckig::InputParameter<7> in;
		ruckig::OutputParameter<7> out;
		for (std::size_t i = 0; i < 7; ++i)
		{
			in.current_position[i] = 0.0;
			in.target_position[i] = (i % 2 == 0) ? 0.8 : -0.6;
			in.max_velocity[i] = 3.0;
			in.max_acceleration[i] = 5.0;
			in.max_jerk[i] = 20.0;
		}
		std::array<double, 7> vmax{}, amax{};
		vmax.fill(3.0); amax.fill(5.0);
		bool finished = false, ok_limits = true;
		for (int i = 0; i < 5000; ++i)
		{
			const auto r = otg.update(in, out);
			if (!limitsHeld(out, vmax, amax)) {ok_limits = false;}
			if (r == ruckig::Result::Finished)
			{
				finished = true;
				break;
			}
			in.current_position = out.new_position;
			in.current_velocity = out.new_velocity;
			in.current_acceleration = out.new_acceleration;
		}
		check(finished, "T2 7轴: 全部到达");
		check(ok_limits, "T2 7轴: 限内");
	}

	// ===== T3 率失配重定向 (核心场景) =====
	{
		ruckig::Ruckig<7> otg{DT};
		ruckig::InputParameter<7> in;
		ruckig::OutputParameter<7> out;
		for (std::size_t i = 0; i < 7; ++i)
		{
			in.current_position[i] = 0.0;
			in.max_velocity[i] = 3.0;
			in.max_acceleration[i] = 5.0;
			in.max_jerk[i] = 20.0;
		}
		std::array<double, 7> vmax{}, amax{};
		vmax.fill(3.0); amax.fill(5.0);
		// 慢上层: 10Hz 换目标 (500Hz 控制的 1/50), 目标来回摆动
		const int n_targets = 4;
		double last_pos[7] = {0}, last_vel[7] = {0};
		bool ok_pos_cont = true, ok_vel_cont = true, ok_limits = true, reached_final = false;
		double worst_dpos = 0.0, worst_dvel = 0.0;
		int target_i = 0;
		for (int i = 0; i < 7500; ++i)   // 15s @500Hz
		{
			if (i % 500 == 0 && target_i < n_targets)   // 每 1s 一个新目标
			{
				const double dir = (target_i % 2 == 0) ? 1.0 : -1.0;
				for (std::size_t k = 0; k < 7; ++k)
				{
					in.target_position[k] = dir * (0.3 + 0.1 * k);
				}
				++target_i;
			}
			const auto r = otg.update(in, out);
			// 逐拍连续性: 位置/速度相对上一拍的增量受 v/a*dt 限 (重定向也不许跳)
			for (std::size_t k = 0; k < 7; ++k)
			{
				const double dpos = std::fabs(out.new_position[k] - last_pos[k]);
				const double dvel = std::fabs(out.new_velocity[k] - last_vel[k]);
				worst_dpos = std::max(worst_dpos, dpos);
				worst_dvel = std::max(worst_dvel, dvel);
				if (dpos > vmax[k] * DT + 1e-9) {ok_pos_cont = false;}
				if (dvel > amax[k] * DT + 1e-9) {ok_vel_cont = false;}
				last_pos[k] = out.new_position[k];
				last_vel[k] = out.new_velocity[k];
			}
			if (!limitsHeld(out, vmax, amax)) {ok_limits = false;}
			if (r == ruckig::Result::Finished && target_i == n_targets)
			{
				reached_final = true;
				break;
			}
			in.current_position = out.new_position;
			in.current_velocity = out.new_velocity;
			in.current_acceleration = out.new_acceleration;
		}
		check(ok_pos_cont, "T3 率失配: 位置逐拍连续 (无跳变)");
		check(ok_vel_cont, "T3 率失配: 速度逐拍连续 (加速度受限)");
		check(ok_limits, "T3 率失配: 限内");
		check(reached_final, "T3 率失配: 最终目标到达");
		std::printf("  (最差逐拍 Δpos=%.4f rad [限 %.4f]  Δvel=%.4f [限 %.4f])\n",
			worst_dpos, 3.0 * DT, worst_dvel, 5.0 * DT);
	}

	// ===== T4 RT 零分配 + T5 计时 =====
	{
		ruckig::Ruckig<7> otg{DT};
		ruckig::InputParameter<7> in;
		ruckig::OutputParameter<7> out;
		for (std::size_t i = 0; i < 7; ++i)
		{
			in.current_position[i] = 0.0;
			in.target_position[i] = 0.5;
			in.max_velocity[i] = 3.0;
			in.max_acceleration[i] = 5.0;
			in.max_jerk[i] = 20.0;
		}
		// 预热 (首触)
		for (int i = 0; i < 10; ++i) {otg.update(in, out);}
		// 稳态: 1000 次 update, 重定向每 50 拍换目标 (制造完整计算路径)
		std::vector<double> times;
		times.reserve(1000);   // 预留必须在计数窗口外 (reserve 本身是 malloc)
		g_counting = true;
		g_allocs = 0;
		for (int i = 0; i < 1000; ++i)
		{
			if (i % 50 == 0)
			{
				in.target_position[0] = (i % 100 == 0) ? 0.5 : -0.3;
			}
			const auto t0 = std::chrono::steady_clock::now();
			otg.update(in, out);
			times.push_back(std::chrono::duration<double, std::micro>(
				std::chrono::steady_clock::now() - t0).count());
			in.current_position = out.new_position;
			in.current_velocity = out.new_velocity;
			in.current_acceleration = out.new_acceleration;
		}
		g_counting = false;
		std::printf("  (稳态 1000 拍 malloc 总数: %zu, 每拍 %.2f 次)\n", g_allocs, g_allocs / 1000.0);
		check(g_allocs == 0, "T4 稳态 update 零 malloc");
		std::sort(times.begin(), times.end());
		std::printf("  (update 计时: p50=%.2fµs p99=%.2fµs max=%.2fµs)\n",
			times[times.size() / 2], times[times.size() * 99 / 100], times.back());
		check(times[times.size() * 99 / 100] < 50.0, "T5 update p99 <50µs (RT 预算证据)");
	}

	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

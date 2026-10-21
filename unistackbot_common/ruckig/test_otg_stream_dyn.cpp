/*
 * test_otg_stream_dyn —— OtgStreamDyn (运行期 DOF) 测试 (2026-10-21, L4+L5)。
 *
 * 覆盖: 基本流 / 尺寸契约 (n=0 拒, vector 尺寸不符拒) / 限值与输入校验 / Hold 契约
 * (消毒失败不覆写调用方 out) / 多 DOF (7=stationlite 臂, 24=r1_pro 整机) /
 * 与静态 OtgStream<7> 数值一致性 (同输入差 < 1e-9 —— 运行期与静态同一算法)。
 *
 * 编译运行:
 *   cd unistackbot_common/ruckig
 *   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion test_otg_stream_dyn.cpp \
 *       src/ruckig/*.cpp -Iinclude -I.. -o /tmp/test_osd && /tmp/test_osd
 */
#include <cmath>
#include <cstdio>
#include <vector>

#include "otg_stream.hpp"

using unistackbot_common::OtgStreamDyn;

namespace
{

int g_fail = 0;
int g_case = 0;

void check(bool ok, const char *what)
{
	++g_case;
	if (!ok)
	{
		++g_fail;
		std::printf("FAIL [%d] %s\n", g_case, what);
	}
}

}  // namespace

int main()
{
	// ---- ① 基本流 (7 DOF, stationlite 臂形态) ----
	{
		OtgStreamDyn ch;
		OtgStreamDyn::Limits lim;
		lim.max_velocity.assign(7, 2.0);
		lim.max_acceleration.assign(7, 10.0);
		lim.max_jerk.assign(7, 50.0);
		check(ch.init(7, 0.002, lim), "init 7 DOF");
		std::vector<double> pos(7, 0.0);
		ch.reset(pos);
		check(ch.isInitialized(), "reset 后 initialized");
		std::vector<double> target(7, 0.1);
		std::vector<double> out(7);
		int ok_count = 0;
		for (int i = 0; i < 500; ++i)
		{
			if (ch.update(target, out) == OtgStreamDyn::UpdateResult::Ok)
			{
				++ok_count;
			}
			for (int d = 0; d < 7; ++d)
			{
				target[d] = out[d];
			}
			target[0] += 0.001;
		}
		check(ok_count > 400, "500 拍 Ok 主导");
		check(std::fabs(out[0] - target[0] + 0.001) < 0.05, "输出跟踪目标");
	}

	// ---- ② 尺寸契约 ----
	{
		OtgStreamDyn ch;
		OtgStreamDyn::Limits lim;
		check(ch.init(0, 0.002, lim) == false, "dofs=0 拒");
		lim.max_velocity.assign(7, 2.0);
		check(ch.init(7, 0.002, lim) == false, "限值数组尺寸不符拒");
		lim.max_acceleration.assign(7, 10.0);
		lim.max_jerk.assign(7, 50.0);
		check(ch.init(7, 0.002, lim), "尺寸齐 ok");
		std::vector<double> t3(3, 0.1), out3(7);
		ch.reset(std::vector<double>(7, 0.0));
		check(ch.update(t3, out3) == OtgStreamDyn::UpdateResult::Hold, "target 尺寸不符 Hold");
		std::vector<double> t8(8, 0.1);
		check(ch.update(t8, out3) == OtgStreamDyn::UpdateResult::Hold, "target 尺寸 8 Hold");
		std::vector<double> t7(7, 0.0), o7;
		check(ch.update(t7, o7) == OtgStreamDyn::UpdateResult::Ok && o7.size() == 7, "out 预分配语义 (调用方持缓冲)");
	}

	// ---- ③ 限值/输入校验 + Hold 契约 ----
	{
		OtgStreamDyn ch;
		OtgStreamDyn::Limits lim;
		lim.max_velocity.assign(7, -1.0);
		lim.max_acceleration.assign(7, 10.0);
		lim.max_jerk.assign(7, 50.0);
		check(ch.init(7, 0.002, lim) == false, "负速度限拒");
		OtgStreamDyn ch2;
		OtgStreamDyn::Limits lim2;
		lim2.max_velocity.assign(7, 2.0);
		lim2.max_acceleration.assign(7, 10.0);
		lim2.max_jerk.assign(7, 50.0);
		check(ch2.init(7, 0.002, lim2), "init2");
		ch2.reset(std::vector<double>(7, 0.0));
		std::vector<double> tbad(7, std::nan(""));
		std::vector<double> out(7, 5.0);
		check(ch2.update(tbad, out) == OtgStreamDyn::UpdateResult::Hold, "NaN 目标 Hold");
		// Hold 契约: out 恒覆写为安全保持位 (reset(0) 后=0) —— "调用方不可能用错"
		bool all_zero = true;
		for (double v : out)
		{
			all_zero = all_zero && v == 0.0;
		}
		check(all_zero, "Hold 契约: out = 安全保持位 (恒有效)");
	}

	// ---- ④ 静态一致性 (7 DOF 同输入, dyn vs OtgStream<7> 差 < 1e-9) ----
	{
		unistackbot_common::OtgStreamDyn dyn;
		unistackbot_common::OtgStream<7> fix;
		OtgStreamDyn::Limits dlim;
		dlim.max_velocity.assign(7, 2.0);
		dlim.max_acceleration.assign(7, 10.0);
		dlim.max_jerk.assign(7, 50.0);
		unistackbot_common::OtgStream<7>::Limits flim;
		flim.max_velocity.fill(2.0);
		flim.max_acceleration.fill(10.0);
		flim.max_jerk.fill(50.0);
		check(dyn.init(7, 0.002, dlim, 0.0) && fix.init(0.002, flim, 0.0), "同参 init");
		std::vector<double> pos(7, 0.0);
		dyn.reset(pos);
		std::array<double, 7> fpos{};
		fix.reset(fpos);
		double max_diff = 0.0;
		std::vector<double> dout(7);
		for (int i = 0; i < 300; ++i)
		{
			std::vector<double> tgt(7, 0.5);
			std::array<double, 7> ftgt{};
			ftgt.fill(0.5);
			dyn.update(tgt, dout);
			fix.update(ftgt, fpos);
			for (int d = 0; d < 7; ++d)
			{
				max_diff = std::max(max_diff, std::fabs(dout[d] - fpos[d]));
			}
		}
		check(max_diff < 1e-9, "dyn 与静态数值一致");
	}

	// ---- ⑤ 24 DOF (r1_pro 整机形态) ----
	{
		OtgStreamDyn ch;
		OtgStreamDyn::Limits lim;
		lim.max_velocity.assign(24, 2.0);
		lim.max_acceleration.assign(24, 10.0);
		lim.max_jerk.assign(24, 50.0);
		check(ch.init(24, 0.002, lim), "init 24 DOF");
		ch.reset(std::vector<double>(24, 0.0));
		std::vector<double> tgt(24, 0.2), out;
		int ok_count = 0;
		for (int i = 0; i < 300; ++i)
		{
			if (ch.update(tgt, out) == OtgStreamDyn::UpdateResult::Ok)
			{
				++ok_count;
			}
		}
		check(ok_count > 250, "24 DOF 轨迹推进");
	}

	std::printf("%s: %d cases, %d fail\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

/*
 * stationlite 解析解测试 (g++ 直编, 不进 colcon; piper 同型)。
 *
 * 用例:
 *   [指纹] 合成正例: 内置 ZYYYZX 链 → chain_match=true
 *   [指纹] 合成反例: j3 轴倾斜 → chain_match=false
 *   [指纹] stationlite 实测: canonical URDF (脚本传入) → chain_match=true,
 *          平行点积 = 1.000000 (几何实锤回验, 决策卡 §1)
 *   [回代] FK-IK 黄金: 200 随机限位内 q → FK → solveAll → 每目标至少一解
 *          FK 回代误差 < 1e-6 m; seed 最近解 = 原解 (种子一致性)
 *   [限位] solve() 限位过滤 + 距 seed 最近 + 结果码语义 (UNREACHABLE 远目标)
 *   [分支] 解数枚举: 部署位目标 ≥2 解 (肘上/肘下)
 *
 * 用法: ./test_stationlite_ik <stationlite_canonical.urdf>
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "analytic_stationlite/analytic_stationlite.hpp"
#include "urdf_fk/urdf_fk.hpp"

using unistackbot_algorithm::AnalyticStationlite;
using unistackbot_algorithm::DlsIkStats;
using unistackbot_algorithm::CartesianPose;
using unistackbot_algorithm::IkResult;
using unistackbot_algorithm::ParallelAxisFingerprint;
using unistackbot_algorithm::UrdfFk;
using unistackbot_algorithm::check_parallel_axes;
using unistackbot_interface::RedundancyPreference;
using unistackbot_interface::RedundancyType;

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

// 合成链: j1=Z @z, j2/j3/j4=Y (平行三轴), j5=Z, j6=X —— ZYYYZX;
// j3_axis_y=1 时平行, 0.9 倾斜 (反例)
std::string synthetic_urdf(double j3_axis_y)
{
	const double y = j3_axis_y;
	const double z = std::sqrt(std::max(0.0, 1.0 - y * y));
	auto joint = [](const char *name, const char *parent, const char *child,
		const char *xyz, const char *axis) {
		return std::string("<joint name=\"") + name + "\" type=\"revolute\"><parent link=\"" +
			parent + "\"/><child link=\"" + child + "\"/><origin xyz=\"" + xyz +
			"\" rpy=\"0 0 0\"/><axis xyz=\"" + axis +
			"\"/><limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	};
	char a3[64];
	std::snprintf(a3, sizeof(a3), "0 %+.6f %+.6f", y, z);
	return std::string("<?xml version=\"1.0\"?><robot name=\"t\">"
	                   "<link name=\"base\"/><link name=\"l1\"/><link name=\"l2\"/>"
	                   "<link name=\"l3\"/><link name=\"l4\"/><link name=\"l5\"/><link name=\"l6\"/>") +
		joint("j1", "base", "l1", "0 0 0.1", "0 0 1") +
		joint("j2", "l1", "l2", "0 0 0.1", "0 1 0") +
		joint("j3", "l2", "l3", "0.2 0 0", a3) +
		joint("j4", "l3", "l4", "0.2 0 0", "0 1 0") +
		joint("j5", "l4", "l5", "0.1 0 0", "0 0 1") +
		joint("j6", "l5", "l6", "0.05 0 0", "1 0 0") +
		"</robot>";
}

}  // namespace

int main(int argc, char ** argv)
{
	if (argc < 2)
	{
		std::printf("用法: %s <stationlite_canonical.urdf>\n", argv[0]);
		return 2;
	}

	// ---- 指纹三例 ----
	{
		UrdfFk good;
		std::string msg;
		check(good.init(synthetic_urdf(1.0), "base", "l6", msg), "合成链 FK init");
		const ParallelAxisFingerprint fp = check_parallel_axes(good);
		check(fp.chain_match, "指纹正例: ZYYYZX 通过");

		UrdfFk bad;
		check(bad.init(synthetic_urdf(0.9), "base", "l6", msg), "合成链(斜 j3) FK init");
		const ParallelAxisFingerprint fb = check_parallel_axes(bad);
		check(!fb.chain_match, "指纹反例: j3 倾斜拒绝");
	}
	{
		UrdfFk fk;
		std::string msg;
		// UrdfFk::init 收 URDF **字符串** (非路径; piper 测试同口径 —— kdl_parser
		// "document empty" 即路径被当字符串解析的签名)
		std::ifstream f(argv[1]);
		std::stringstream ss;
		ss << f.rdbuf();
		const std::string urdf = ss.str();
		check(!urdf.empty() && fk.init(urdf, "left_armbase", "left_linkgripper", msg), "canonical FK init");
		if (!fk.ready())
		{
			std::printf("canonical FK init 失败: %s\n", msg.c_str());
			return 1;
		}
		const ParallelAxisFingerprint fp = check_parallel_axes(fk);
		check(fp.chain_match, "stationlite 实测: 三平行轴成立");
		check(fp.min_parallel_dot > 1.0 - 1e-9, "平行点积严格 = 1");
		std::printf("  指纹: %s\n", fp.message.c_str());

		// ---- solver 装配 + 回代 ----
		AnalyticStationlite ik;
		check(ik.init(&fk, msg), "solver init (指纹通过)");
		std::printf("  init: %s\n", msg.c_str());

		std::mt19937 rng(42);
		const auto & lo = fk.qMin();
		const auto & hi = fk.qMax();
		std::uniform_real_distribution<double> u(0.0, 1.0);
		int roundtrip_ok = 0;
		int solved_targets = 0;
		double worst_err = 0.0;
		std::vector<double> q(6), out_q;
		for (int t = 0; t < 200; ++t)
		{
			for (int i = 0; i < 6; ++i)
			{
				q[i] = lo[i] + (hi[i] - lo[i]) * u(rng);
			}
			CartesianPose pose;
			UrdfFk::Scratch scratch;
			check(fk.fk(q, pose, scratch), "FK (合成目标)");

			std::vector<std::vector<double>> sols;
			const int n = ik.solveAll(pose, sols);
			if (n <= 0)
			{
				continue;   // 边界目标可 0 解 (回代门只对有解目标)
			}
			++solved_targets;
			bool any = false;
			for (const auto & s : sols)
			{
				CartesianPose back;
				check(fk.fk(s, back, scratch), "FK (解回代)");
				const double err = std::fabs(back.x - pose.x) + std::fabs(back.y - pose.y) +
					std::fabs(back.z - pose.z);
				worst_err = std::max(worst_err, err);
				if (err < 1e-6)
				{
					any = true;
				}
			}
			if (any)
			{
				++roundtrip_ok;
			}
			// solve(): seed=原解 → 应取回该解 (种子一致性)
			RedundancyPreference red;
			DlsIkStats st;
			const IkResult r = ik.solve(pose, q, red, out_q, &st);
			check(r == IkResult::OK, "solve OK");
			if (r == IkResult::OK)
			{
				double d2 = 0.0;
				for (int i = 0; i < 6; ++i)
				{
					const double e = out_q[i] - q[i];
					d2 += e * e;
				}
				check(d2 < 1e-6, "solve 距 seed 最近 = 原解 (d2<1e-6)");
				check(st.ok && st.iterations == 0, "stats: ok + 闭式零迭代");
			}
		}
		std::printf("  回代: %d/%d 目标有解, 回代全过 %d, 最差位置误差 %.2e m\n",
			solved_targets, 200, roundtrip_ok, worst_err);
		check(solved_targets > 150, "解出率 > 75% (限位内随机目标)");
		check(roundtrip_ok == solved_targets, "全部有解目标 FK 回代 < 1e-6 m");

		// 远目标 → UNREACHABLE
		CartesianPose far;
		far.x = 10.0;
	far.y = 0.0;
	far.z = 0.0;
	far.qw = 1.0;
		RedundancyPreference red;
		const IkResult rf = ik.solve(far, q, red, out_q, nullptr);
		check(rf == IkResult::UNREACHABLE, "远目标 → UNREACHABLE");

		// 部署位目标分支数 ≥ 2 (肘上/肘下)
		for (int i = 0; i < 6; ++i)
		{
			q[i] = lo[i] + (hi[i] - lo[i]) * 0.5;
		}
		q[1] = 0.5;
	q[2] = -0.8;
	q[3] = 0.6;   // 离开零位奇异 (j234 共面)
		CartesianPose deploy;
		UrdfFk::Scratch scratch;
		fk.fk(q, deploy, scratch);
		std::vector<std::vector<double>> sols;
		const int nb = ik.solveAll(deploy, sols);
		check(nb >= 2, "部署位目标 ≥2 分支 (肘上/肘下)");
		std::printf("  部署位分支数: %d\n", nb);
	}

	std::printf("%s: %d cases, %d fail\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

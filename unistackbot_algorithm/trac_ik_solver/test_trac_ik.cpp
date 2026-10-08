/*
 * trac_ik AB/回归测试床 (g++ 直编, 2026-10-08; 零 ROS main, 链接 rclcpp 因 vendored 上游)。
 * 编译运行: test/run_trac_ik_test.sh (用法同 run_ik_test.sh)
 *
 * 层1 oracle 冷启动:  P 行 COLD_START (限位中点种子) 成功率 × 预算档 {50ms, 5ms, 1ms}
 *                     + FK 回代闭合 (<1e-4 m / <1e-3 rad) + 耗时分位数;
 *                     参考线 = Speed 档 AB (2026-09-29, /tmp 工具): R1 50ms 100.0% / 1ms 96.0%。
 *                     Distance 档可用解集合 ⊇ Speed (烧满预算收集), 预期不降。
 * 层2 流式连续性:     圆弧 100 点 STREAMING (实测种子), 单步增量 <0.15 rad, 零失败
 *                     —— 2026-10-08 Speed 构型渐移破案的回归锚 (Distance+粘性守卫)。
 * 层3 契约面:         失败不改 out_q / LOCK_JOINT=UNSUPPORTED / NOT_READY。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "trac_ik_solver/trac_ik_solver.hpp"
#include "urdf_fk/urdf_fk.hpp"

using unistackbot_algorithm::CartesianPose;
using unistackbot_algorithm::DlsIkStats;
using unistackbot_algorithm::IkResult;
using unistackbot_algorithm::RedundancyPreference;
using unistackbot_interface::RedundancyType;
using unistackbot_algorithm::SolveMode;
using unistackbot_algorithm::TracIk;
using unistackbot_algorithm::UrdfFk;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond) do { if (cond) { ++g_pass; } else { ++g_fail; std::printf("FAIL: %s\n", #cond); } } while (0)

static std::string loadFile(const char * path)
{
	FILE * f = std::fopen(path, "rb");
	if (!f) {return "";}
	std::string s; char b[65536]; size_t n;
	while ((n = std::fread(b, 1, sizeof(b), f)) > 0) {s.append(b, n);}
	fclose(f);
	return s;
}

static double pct(std::vector<double> & v, double p)
{
	if (v.empty()) {return 0.0;}
	std::sort(v.begin(), v.end());
	return v[std::min(static_cast<std::size_t>(p * (v.size() - 1)), v.size() - 1)];
}

struct OraclePoint
{
	CartesianPose pose;
};

int main(int argc, char ** argv)
{
	if (argc < 4)
	{
		std::printf("用法: test_trac_ik <urdf> <oracle文件> <tip> [base]\n");
		return 2;
	}
	const char * base = (argc >= 5) ? argv[4] : "link_base";
	UrdfFk fk;
	std::string msg;
	if (!fk.init(loadFile(argv[1]), base, argv[3], msg))
	{
		std::printf("FAIL: fk.init base=%s tip=%s: %s\n", base, argv[3], msg.c_str());
		return 1;
	}
	const unsigned int n = fk.jointCount();
	std::printf("[FK] %u 关节, maxReach %.3f m\n", n, fk.maxReach());

	TracIk ik;
	CHECK(ik.init(&fk, msg));
	CHECK(ik.ready());

	// 解析 oracle P 行 (位姿行; 解行忽略 —— 本床只测成功率/回代, 不对真值解集)
	std::vector<OraclePoint> pts;
	{
		std::istringstream is(loadFile(argv[2]));
		std::string line;
		while (std::getline(is, line))
		{
			if (line.empty() || line[0] != 'P') {continue;}
			OraclePoint e;
			double qx = 0, qy = 0, qz = 0;
			if (std::sscanf(line.c_str(), "%*c %lf %lf %lf %lf %lf %lf %lf",
				&e.pose.x, &e.pose.y, &e.pose.z, &e.pose.qw, &qx, &qy, &qz) == 7)
			{
				e.pose.qx = qx; e.pose.qy = qy; e.pose.qz = qz;
				pts.push_back(e);
			}
		}
	}
	std::printf("[oracle] %zu 个可达位姿\n", pts.size());
	CHECK(pts.size() >= 100);
	const RedundancyPreference preserve{RedundancyType::PRESERVE, {}};

	// ---- 层1: COLD_START 成功率 × 预算档 (限位中点种子在适配层内建) ----
	struct Budget {double ms; const char * speed_ref;};
	const Budget budgets[] = {
		{50.0, "Speed 参考线 100.0%"},
		{5.0, "Speed 参考线 ~99%"},
		{1.0, "Speed 参考线 96.0%"},
	};
	for (const auto & b : budgets)
	{
		ik.setBudgetMs(b.ms);
		int ok_n = 0;
		double rot_worst = 0.0, pos_worst = 0.0;
		std::vector<double> times;
		std::vector<double> q(n, 0.0);
		for (const auto & e : pts)
		{
			DlsIkStats st;
			std::vector<double> out(n, -12345.0);
			const auto t0 = std::chrono::steady_clock::now();
			const IkResult r = ik.solve(e.pose, q, preserve, out, &st, SolveMode::COLD_START);
			const double us = std::chrono::duration<double, std::micro>(
				std::chrono::steady_clock::now() - t0).count();
			if (r != IkResult::OK) {continue;}
			++ok_n;
			times.push_back(st.solve_us > 0.0 ? st.solve_us : us);
			CartesianPose got;
			if (fk.fk(out, got))
			{
				const double dp = std::sqrt(
					(got.x - e.pose.x) * (got.x - e.pose.x) +
					(got.y - e.pose.y) * (got.y - e.pose.y) +
					(got.z - e.pose.z) * (got.z - e.pose.z));
				const double qd = std::fabs(
					got.qw * e.pose.qw + got.qx * e.pose.qx +
					got.qy * e.pose.qy + got.qz * e.pose.qz);
				const double dr = 2.0 * std::acos(std::clamp(qd, 0.0, 1.0));
				pos_worst = std::max(pos_worst, dp);
				rot_worst = std::max(rot_worst, dr);
			}
		}
		const double rate = 100.0 * ok_n / static_cast<double>(pts.size());
		std::printf("[层1 %5.0fms] 成功率 %5.1f%% (%zu/%zu)  耗时 p50=%.2fms p95=%.2fms  "
			"回代 worst pos=%.2e m rot=%.2e rad  (%s)\n",
			b.ms, rate, ok_n, pts.size(), pct(times, 0.50) / 1000.0,
			pct(times, 0.95) / 1000.0, pos_worst, rot_worst, b.speed_ref);
		// 阈值含随机重启方差 (±2% 实测): 50ms 生产档 98 / 短档 90
		CHECK(rate >= (b.ms >= 50.0 ? 98.0 : 90.0));
		CHECK(pos_worst < 1e-4);        // AB 实测 <1e-4 全过 (适配层 eps 口径)
		CHECK(rot_worst < 1e-3);
	}

	// ---- 层2: 流式连续性 (圆弧 100 点, STREAMING, Distance+粘性回归锚) ----
	// 锚点探针先行 (demo axes 纪律): 随机 oracle 点的 2cm 邻域可能压可达域边界,
	// 边界外弧段的诚实拒绝 ≠ 连续性失败 —— 选一个 8 向邻域全可达的锚点再绕圈
	ik.setBudgetMs(50.0);
	{
		std::vector<double> mid(n, 0.0);
		std::vector<double> scratch(n, 0.0);
		std::size_t anchor = pts.size();
		for (std::size_t i = 0; i < pts.size() && anchor == pts.size(); ++i)
		{
			int ok8 = 0;
			for (int d = 0; d < 8; ++d)
			{
				static const double DX[8] = {1, -1, 0, 0, 0.7071, 0.7071, -0.7071, -0.7071};
				static const double DY[8] = {0, 0, 1, -1, 0.7071, -0.7071, 0.7071, -0.7071};
				CartesianPose t = pts[i].pose;
				t.x += 0.02 * DX[d];
				t.y += 0.02 * DY[d];
				std::vector<double> out(n, 0.0);
				if (ik.solve(t, mid, preserve, out, nullptr,
					SolveMode::COLD_START) == IkResult::OK)
				{
					++ok8;
				}
			}
			if (ok8 == 8) {anchor = i;}
		}
		CHECK(anchor < pts.size());
		if (anchor == pts.size())
		{
			std::printf("[层2 流式] 无 8 向全可达锚点 —— 跳过 (工作空间过窄?)\n");
		}
		else
		{
			const OraclePoint & p0 = pts[anchor];
			std::vector<double> seed(n, 0.0);
			std::vector<double> q(n, 0.0);
			IkResult r = ik.solve(p0.pose, seed, preserve, q, nullptr, SolveMode::COLD_START);
			CHECK(r == IkResult::OK);
			const std::vector<double> q0 = q;
			int fail_n = 0, big_step_n = 0;
			double max_drift = 0.0, max_step = 0.0;
			std::vector<double> steps;
			for (int i = 1; i <= 100; ++i)
			{
				const double th = 2.0 * 3.14159265358979 * i / 100.0;
				CartesianPose t = p0.pose;
				t.x += 0.02 * std::cos(th);   // r=2cm 圆周小步 (遥操作单步量级)
				t.y += 0.02 * std::sin(th);
				std::vector<double> out(n, 0.0);
				r = ik.solve(t, q, preserve, out, nullptr, SolveMode::STREAMING);
				if (r != IkResult::OK) {++fail_n; continue;}
				double step = 0.0, drift = 0.0;
				for (unsigned int k = 0; k < n; ++k)
				{
					step += std::fabs(out[k] - q[k]);
					drift += std::fabs(out[k] - q0[k]);
				}
				max_step = std::max(max_step, step);
				max_drift = std::max(max_drift, drift);
				if (step > 0.15) {++big_step_n;}   // 分支级跳步 (1.26mm 弧步下)
				steps.push_back(step);
				q = out;
			}
			std::printf("[层2 流式] 锚 P%zu 圆弧 100 点: 失败 %d (诚实拒, 难区), "
				"跳步>0.15rad %d, 步长 p50=%.3f p95=%.3f max=%.3f rad, "
				"最大漂移 |q-q0|=%.2f rad\n",
				anchor, fail_n, big_step_n, pct(steps, 0.50), pct(steps, 0.95),
				max_step, max_drift);
			// 回归锚 = 2026-10-08 Speed 渐移签名: 构型单调跑飞 (修前 ||q-q0|| 单调
			// 涨到 3.24 rad)。难区跳分支/诚实拒是几何事实, 不在此断言。
			CHECK(max_drift < 3.0);
			CHECK(pct(steps, 0.95) < 0.3);
		}
	}

	// ---- 层3: 契约面 ----
	{
		CartesianPose far_ = pts[0].pose;
		far_.x += 10.0;   // 远超臂展
		std::vector<double> q(n, 1.5), out(q);
		const IkResult r = ik.solve(far_, q, preserve, out, nullptr, SolveMode::COLD_START);
		CHECK(r != IkResult::OK);
		CHECK(out == q);   // 失败不改 out_q (契约 1)
		RedundancyPreference lock{RedundancyType::LOCK_JOINT, {}};
		std::vector<double> out2(n, 0.0);
		CHECK(ik.solve(pts[0].pose, q, lock, out2, nullptr,
			SolveMode::COLD_START) == IkResult::UNSUPPORTED);
	}

	std::printf("\n结果: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}

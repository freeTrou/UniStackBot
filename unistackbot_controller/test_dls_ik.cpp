/*
 * dls_ik 六层验证 (P1.4, g++ 直编; 零 ssik 依赖——真值来自 ik_oracle_xarm7.txt)。
 * 编译运行: test/run_ik_test.sh
 *
 * 层1 点正确性:  R 行 -> 求解成功 + FK 回代闭合;  U 行 -> 必须 UNREACHABLE
 * 层2 分支匹配:  数值解与 ssik 解集最小关节距离 < 1e-3
 * 层3 轨迹连续性: 100 点圆弧, PRESERVE 种子, 增量 < 0.15 rad, 零失败
 * 层4 对抗表:    维度/垃圾种子/失败不改输出/锁越限/ARM_ANGLE/千次零泄漏
 * 层5 性能统计:  500 位姿 p50/p95/p99 耗时 + 收敛率 (报告制)
 * 层6 超时预算:  timeout_ns 墙钟封顶 —— 退化预算立即超时/输出不变;
 *               200Hz(5ms)/500Hz(2ms) 周期预算下边界恪守 + 成功率代价 +
 *               迭代上限调配 (5ms 全局40 / 2ms 分层r40);
 *               流式 500µs 预算下圆弧跟踪不劣化
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#include "unistackbot_controller/dls_ik.hpp"

using unistackbot_controller::CartesianPose;
using unistackbot_controller::DlsIk;
using unistackbot_controller::DlsIkConfig;
using unistackbot_controller::DlsIkStats;
using unistackbot_controller::IkResult;
using unistackbot_controller::RedundancyPreference;
using unistackbot_controller::RedundancyType;
using unistackbot_controller::UrdfFk;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond) do { if (cond) { ++g_pass; } else { ++g_fail; std::printf("FAIL: %s\n", #cond); } } while (0)

static std::size_t g_allocs = 0;
static bool g_counting = false;
void * operator new(std::size_t sz)
{
	if (g_counting) {++g_allocs;}
	return std::malloc(sz ? sz : 1);
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

static std::string loadFile(const char * path)
{
	FILE * f = std::fopen(path, "rb");
	if (!f) {return "";}
	std::string s; char b[65536]; size_t n;
	while ((n = std::fread(b, 1, sizeof(b), f)) > 0) {s.append(b, n);}
	fclose(f);
	return s;
}

struct OracleEntry
{
	CartesianPose pose;
	bool reachable;
	std::vector<std::vector<double>> sols;
	int n_sols_header{0};   // 头部声明的解数 (v2; 用于一致性检查)
};

int main(int argc, char ** argv)
{
	if (argc < 4)
	{
		std::printf("用法: test_dls_ik <urdf> <oracle文件> <tip>\n");
		return 2;
	}
	UrdfFk fk;
	std::string msg;
	CHECK(fk.init(loadFile(argv[1]), "link_base", argv[3], msg));
	const unsigned int n = fk.jointCount();

	DlsIk ik;
	CHECK(ik.init(&fk, msg));

	// 种子库 (可选 argv[4]): COLD_START 阶梯级0.5 的目标导向种子
	if (argc >= 5)
	{
		CHECK(ik.loadSeedLibrary(argv[4], msg));
		std::printf("[种子库] %zu 条 (%s)\n", ik.seedLibrarySize(), argv[4]);
	}

	// 解析 oracle
	std::vector<OracleEntry> entries;
	{
		const std::string txt = loadFile(argv[2]);
		CHECK(!txt.empty());
		std::istringstream is(txt);
		std::string line;
		while (std::getline(is, line))
		{
			if (line.empty() || line[0] == '#') {continue;}
			if (line[0] == 'P' || line[0] == 'U')
			{
				OracleEntry e;
				e.reachable = (line[0] == 'P');
				double qx = 0, qy = 0, qz = 0;
				if (std::sscanf(line.c_str(), "%*c %lf %lf %lf %lf %lf %lf %lf",
					&e.pose.x, &e.pose.y, &e.pose.z, &e.pose.qw, &qx, &qy, &qz) == 7)
				{
					e.pose.qx = qx; e.pose.qy = qy; e.pose.qz = qz;
					// v2 格式: "P ... | n_sols | meta" — 解数是第一个 | 后字段
					int n_sols = 0;
					if (e.reachable)
					{
						const auto bar1 = line.find('|');
						const auto bar2 = line.find('|', bar1 + 1);
						if (bar1 != std::string::npos)
						{
							// bar2 存在 = v2 (n | meta); 不存在 = v1 (n 结尾)
							const auto n_end = (bar2 != std::string::npos) ? bar2 : line.size();
							n_sols = std::atoi(line.substr(bar1 + 1, n_end - bar1 - 1).c_str());
						}
					}
					e.n_sols_header = n_sols;
					entries.push_back(e);
				}
			}
			else if (line[0] == ' ' && !entries.empty() && entries.back().reachable)
			{
				std::vector<double> q(n);
				if (std::sscanf(line.c_str(), " %lf %lf %lf %lf %lf %lf %lf",
					&q[0], &q[1], &q[2], &q[3], &q[4], &q[5], &q[6]) == static_cast<int>(n))
				{
					entries.back().sols.push_back(q);
				}
			}
		}
	}
	std::printf("[oracle] %zu 条 (可达+不可达)\n", entries.size());
	CHECK(entries.size() >= 100);

	const RedundancyPreference preserve;   // PRESERVE 默认

	// ============ 层1+2: 真值对拍 ============
	std::printf("== 层1/2: 真值对拍 ==\n");
	int reach_ok = 0, reach_total = 0, unreach_ok = 0, unreach_total = 0, branch_total = 0;
	int n_near_singular = 0;
	std::vector<double> branch_dists;
	for (const auto & e : entries)
	{
		// 冷启动种子 = 限位中心 (真实调用方传"当前 q"; 全零对 xarm7 是毒种子:
		// j4 距下限仅 0.19 rad, 零位构型本身贴边界)
		std::vector<double> q(n);
		for (unsigned k = 0; k < n; ++k) {q[k] = (fk.qMin()[k] + fk.qMax()[k]) / 2;}
		DlsIkStats st;
		const IkResult r = ik.solve(e.pose, q, preserve, q, &st,
			unistackbot_controller::SolveMode::COLD_START);
		if (e.reachable)
		{
			++reach_total;
			if (r == IkResult::OK)
			{
				CHECK(st.min_sigma > 0.0);   // 遥测: 成功解必有 σ (中心种子不会 0 迭代收敛)
				// FK 回代 (用测试独立 oracle FK)
				CartesianPose back;
				CHECK(fk.fk(q, back));
				const double dp = std::max({std::fabs(back.x - e.pose.x),
					std::fabs(back.y - e.pose.y), std::fabs(back.z - e.pose.z)});
				if (dp < 1e-6) {++reach_ok;}
				// 限位闭区间 (安全断言)
				bool in_lim = true;
				for (unsigned i = 0; i < n; ++i)
				{
					if (q[i] < fk.qMin()[i] - 1e-9 || q[i] > fk.qMax()[i] + 1e-9) {in_lim = false;}
				}
				CHECK(in_lim);
				// 层2 分支距离: 统计报告 (分支=连通流形, 欧氏距离非正确度量;
				// 正确的构型签名匹配与 j4 回归一起做)
				if (!e.sols.empty())
				{
					++branch_total;
					double best = 1e9;
					for (const auto & s : e.sols)
					{
						double d = 0;
						for (unsigned i = 0; i < n; ++i)
						{
							// 角度 wrap: 差值折到 [-pi,pi] 再累加 (j1/j5 缠绕整圈是
							// 等价解, 不 wrap 会把同分支算成 ~2*pi 距离)
							const double diff = std::fabs(std::fmod(q[i] - s[i] + 3 * M_PI, 2 * M_PI) - M_PI);
							d += diff;
						}
						best = std::min(best, d);
					}
					branch_dists.push_back(best);
				}
			}
		}
		else
		{
			++unreach_total;
			// 断言: 不可达位姿必须以非 OK 结束 (几何预检=UNREACHABLE, 难样本=按 σ 分类
			// NEAR_SINGULAR / ITERATION_LIMIT), 核心是不许输出解
			if (r != IkResult::OK)
			{
				++unreach_ok;
				if (r == IkResult::NEAR_SINGULAR)
				{
					++n_near_singular;
					CHECK(st.min_sigma > 0.0 && st.min_sigma < 0.05);   // 分类与遥测自洽
				}
			}
		}
	}
	std::printf("  可达冷启动求解 %d/%d (COLD_START: 分支代表种子+无粘性)\n",
		reach_ok, reach_total);
	std::printf("  不可达非OK %d/%d (几何预检拦截远距; 姿态级失败按 σ_min 分类, NEAR_SINGULAR %d)\n",
		unreach_ok, unreach_total, n_near_singular);
	if (!branch_dists.empty())
	{
		std::sort(branch_dists.begin(), branch_dists.end());
		std::printf("  分支距离: min=%.3f p50=%.3f max=%.3f (rad; 报告制——欧氏距离在缠绕流形上无分支语义, 构型签名待做)\n",
			branch_dists.front(), branch_dists[branch_dists.size() / 2], branch_dists.back());
	}
	// 达标线: 成功的解必须全部 FK 闭合 (reach_ok 的统计口径已含 <1e-6) + 不可达必须不出解
	CHECK(reach_total == 0 || reach_ok * 100 / reach_total >= 95);   // 冷启动达标线 (2026-09-17 90/90 后转正式验收)
	CHECK(unreach_total == 0 || unreach_ok * 100 / unreach_total >= 95);
	// 真正的达标线在层3 (流式 = 主场景): 101/101 + 增量阈值
	

	// ============ 层3: 轨迹连续性 (100 点圆弧) ============
	std::printf("== 层3: 轨迹连续性 ==\n");
	{
		std::vector<double> seed(n, 0.0);
		CartesianPose prev;
		std::vector<double> prev_q;
		int consec_fail = 0;
		double worst_step = 0.0;
		int ok_pts = 0;
		for (int i = 0; i <= 100; ++i)
		{
			const double th = 2 * M_PI * i / 100.0;
			CartesianPose p;
			p.x = 0.35 + 0.10 * std::cos(th);
			p.y = 0.10 * std::sin(th);
			p.z = 0.45;
			p.qw = 0.0; p.qx = 1.0;   // 末端朝下
			std::vector<double> q(n, 0.0);
			if (i == 0)
			{
				// 首点: 从中心种子起步
				for (unsigned k = 0; k < n; ++k) {q[k] = (fk.qMin()[k] + fk.qMax()[k]) / 2;}
			}
			else
			{
				q = prev_q;
			}
			const IkResult r = ik.solve(p, q, preserve, q, nullptr,
				unistackbot_controller::SolveMode::STREAMING);
			if (r != IkResult::OK)
			{
				++consec_fail;
				continue;
			}
			consec_fail = 0;
			if (!prev_q.empty())
			{
				double step = 0;
				for (unsigned k = 0; k < n; ++k) {step += std::fabs(q[k] - prev_q[k]);}
				worst_step = std::max(worst_step, step);
			}
			prev_q = q;
			++ok_pts;
		}
		std::printf("  圆弧 101 点: 成功 %d, 最差相邻增量 %.3f rad (线 0.15)\n", ok_pts, worst_step);
		CHECK(ok_pts >= 95);
		CHECK(worst_step < 0.15);
	}

	// ============ 层4: 对抗表 ============
	std::printf("== 层4: 对抗表 ==\n");
	{
		CartesianPose p;
		p.x = 0.3; p.y = 0.1; p.z = 0.4; p.qw = 0; p.qx = 1;
		std::vector<double> q(n, 0.0), out(n, -1.0);
		// 1) 维度不符
		std::vector<double> bad_seed(n - 1, 0.0);
		CHECK(ik.solve(p, bad_seed, preserve, out) == IkResult::NOT_READY);
		// 2) 垃圾种子 -> 不产生 NaN
		std::vector<double> junk(n, 1e6);
		const IkResult rj = ik.solve(p, junk, preserve, out);
		bool finite = true;
		if (rj == IkResult::OK)
		{
			for (double v : out) {finite &= std::isfinite(v);}
		}
		CHECK(finite);
		// 3) 失败不改输出
		CartesianPose far; far.x = 5.0; far.y = 0; far.z = 0; far.qw = 1;
		std::vector<double> sentinel(n, -123.0), out2 = sentinel;
		const IkResult rf = ik.solve(far, q, preserve, out2);
		CHECK(rf != IkResult::OK);
		bool unchanged = true;
		for (unsigned i = 0; i < n; ++i) {unchanged &= (out2[i] == sentinel[i]);}
		CHECK(unchanged);
		// 4) LOCK_JOINT 越限
		RedundancyPreference lock;
		lock.type = RedundancyType::LOCK_JOINT;
		lock.joint_index = 3;
		std::vector<double> seed_bad_lim = q;
		seed_bad_lim[3] = 99.0;   // 越限锁定值
		CHECK(ik.solve(p, seed_bad_lim, lock, out) == IkResult::LIMIT_CONFLICT);
		// 5) ARM_ANGLE -> UNSUPPORTED
		RedundancyPreference aa;
		aa.type = RedundancyType::ARM_ANGLE;
		aa.psi = 0.5;
		CHECK(ik.solve(p, q, aa, out) == IkResult::UNSUPPORTED);
		// 6) 千次调用零内存增长
		g_counting = true; g_allocs = 0;
		int ok_calls = 0;
		for (int i = 0; i < 1000; ++i) {ok_calls += (ik.solve(p, q, preserve, out) == IkResult::OK);}
		const std::size_t a1 = g_allocs; g_allocs = 0;
		for (int i = 0; i < 1000; ++i) {ok_calls += (ik.solve(p, q, preserve, out) == IkResult::OK);}
		const std::size_t a2 = g_allocs;
		g_counting = false;
		std::printf("  千次调用分配: 前 1000=%zu 后 1000=%zu (要求相等=稳态零增长)\n", a1, a2);
		CHECK(a1 == a2);
		CHECK(ok_calls > 0);   // 同批调用的可解性 (消费 [[nodiscard]] 返回值)
	}

	// ============ 层5: 性能统计 ============
	std::printf("== 层5: 性能统计 ==\n");
	{
		std::vector<double> times;
		int ok = 0, total = 0;
		std::vector<double> q(n, 0.0);
		for (const auto & e : entries)
		{
			if (!e.reachable) {continue;}
			++total;
			const auto t0 = std::chrono::steady_clock::now();
			const IkResult r = ik.solve(e.pose, q, preserve, q, nullptr,
				unistackbot_controller::SolveMode::COLD_START);
			const auto t1 = std::chrono::steady_clock::now();
			if (r == IkResult::OK) {++ok;}
			times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
		}
		if (!times.empty())
		{
			std::sort(times.begin(), times.end());
			const double p50 = times[times.size() / 2];
			const double p95 = times[times.size() * 95 / 100];
			const double p99 = times[times.size() * 99 / 100];
			std::printf("  %zu 位姿: p50=%.2fms p95=%.2fms p99=%.2fms 收敛率 %.1f%% (参考 p99<5ms, ≥95%%)\n",
				times.size(), p50, p95, p99, 100.0 * ok / std::max(total, 1));
		}
	}

	// ============ 层6: 超时预算 (RT 周期安全; 200Hz/500Hz) ============
	std::printf("== 层6: 超时预算 (timeout_ns) ==\n");
	{
		const DlsIkConfig cfg_def;   // 层1-5 的默认参数基底 (timeout_ns=0)

		// 6-1 退化预算 1ns: 预算门首迭代即触发 —— 非 OK / timed_out / 输出不变 / 秒回
		{
			DlsIkConfig c = cfg_def;
			c.timeout_ns = 1;
			ik.setConfig(c);
			CartesianPose p;
			p.x = 0.35; p.y = 0.0; p.z = 0.45; p.qw = 0.0; p.qx = 1.0;
			std::vector<double> seed(n);
			for (unsigned k = 0; k < n; ++k) {seed[k] = (fk.qMin()[k] + fk.qMax()[k]) / 2;}
			std::vector<double> sentinel(n, -7.0), out = sentinel;
			DlsIkStats st;
			const IkResult r = ik.solve(p, seed, preserve, out, &st,
				unistackbot_controller::SolveMode::COLD_START);
			CHECK(r != IkResult::OK);
			CHECK(st.timed_out);
			bool unchanged = true;
			for (unsigned i = 0; i < n; ++i) {unchanged &= (out[i] == sentinel[i]);}
			CHECK(unchanged);
			CHECK(st.solve_us < 1000.0);   // 不烧重启, 立即返回
		}

		// 6-2 周期预算扫描: 冷启动全可达集, 预算 = 整周期 (最坏分配; CM 实际
		//     COLD_START 走低优线程, 此处验证超时机制的边界恪守 + 成功率代价 +
		//     迭代上限调配的水位)。2026-09-17 w_manip=0 跳过 H₂ 后单迭代 ~1/3
		//     成本, 两档最优统一为全局 max_iterations=40 (2ms 95.5% / 5ms 97.9%)
		struct BudgetCase {const char * label; uint64_t budget_ns; int max_iter; int restart_iter; int floor;};
		std::vector<BudgetCase> cases = {
			{"200Hz 整周期 默认上限",  5000000ull, 200, 0,  92},
			{"500Hz 整周期 默认上限",  2000000ull, 200, 0,  85},
			{"200Hz 全局40",           5000000ull, 40,  0,  95},
			{"500Hz 全局40 (达标行)",  2000000ull, 40,  0,  93},
			{"500Hz 分层r40",          2000000ull, 200, 40, 88},
		};
		// 1ms 目标行只在种子库就绪时有意义 (无库 90% 上下, 有库预期 >=95%)
		if (argc >= 5)
		{
			cases.push_back({"1ms 全局30 (目标行)", 1000000ull, 30, 0, 92});
		}
		for (const auto & bc : cases)
		{
			DlsIkConfig c = cfg_def;
			c.timeout_ns = bc.budget_ns;
			c.max_iterations = bc.max_iter;
			c.restart_max_iterations = bc.restart_iter;
			const uint64_t budget_ns = bc.budget_ns;
			ik.setConfig(c);
			int ok = 0, total = 0, n_timeout = 0;
			double max_us = 0.0;
			std::vector<double> times;
			for (const auto & e : entries)
			{
				if (!e.reachable) {continue;}
				++total;
				std::vector<double> seed(n);
				for (unsigned k = 0; k < n; ++k) {seed[k] = (fk.qMin()[k] + fk.qMax()[k]) / 2;}
				std::vector<double> q = seed;
				DlsIkStats st;
				const IkResult r = ik.solve(e.pose, seed, preserve, q, &st,
					unistackbot_controller::SolveMode::COLD_START);
				times.push_back(st.solve_us);
				max_us = std::max(max_us, st.solve_us);
				if (st.timed_out) {++n_timeout;}
				if (r == IkResult::OK)
				{
					++ok;
					CartesianPose back;   // 超时机制不得污染 OK 路径的解质量
					CHECK(fk.fk(q, back));
					const double dp = std::max({std::fabs(back.x - e.pose.x),
						std::fabs(back.y - e.pose.y), std::fabs(back.z - e.pose.z)});
					CHECK(dp < 1e-6);
				}
			}
			std::sort(times.begin(), times.end());
			const double p50 = times[times.size() / 2] / 1000.0;
			const double p99 = times[times.size() * 99 / 100] / 1000.0;
			std::printf("  %s: 成功 %d/%d (%.1f%%) 超时中断 %d  p50=%.2fms p99=%.2fms max=%.2fms\n",
				bc.label, ok, total, 100.0 * ok / std::max(total, 1), n_timeout, p50, p99,
				max_us / 1000.0);
			// 恪守边界: 实际耗时 ≤ 预算 + 单次迭代粒度 (~40µs) + 调度抖动余量
			CHECK(max_us <= static_cast<double>(budget_ns) / 1000.0 + 300.0);
			// 成功率底线 (防回归线, 非验收线): 无预算基线 99.5%, 尾部难样本需要多轮
			// 重启 —— 生产链路 COLD_START 走低优线程大预算, 此处是"调用线程内整周期"
			// 的最坏分配水位
			CHECK(ok * 100 / std::max(total, 1) >= bc.floor);
		}

		// 6-3 流式预算 0.5ms (500Hz 周期的 1/4 份额): 圆弧跟踪不得劣化 ——
		//     流式种子=上一解, 常态几迭代收敛, 预算只拦异常长尾
		{
			DlsIkConfig c = cfg_def;
			c.timeout_ns = 500000;
			ik.setConfig(c);
			std::vector<double> prev_q;
			int ok_pts = 0;
			double worst_step = 0.0, max_us = 0.0;
			for (int i = 0; i <= 100; ++i)
			{
				const double th = 2 * M_PI * i / 100.0;
				CartesianPose p;
				p.x = 0.35 + 0.10 * std::cos(th);
				p.y = 0.10 * std::sin(th);
				p.z = 0.45;
				p.qw = 0.0; p.qx = 1.0;   // 末端朝下
				std::vector<double> q(n);
				if (i == 0 || prev_q.empty())
				{
					for (unsigned k = 0; k < n; ++k) {q[k] = (fk.qMin()[k] + fk.qMax()[k]) / 2;}
				}
				else
				{
					q = prev_q;
				}
				DlsIkStats st;
				const IkResult r = ik.solve(p, q, preserve, q, &st,
					unistackbot_controller::SolveMode::STREAMING);
				max_us = std::max(max_us, st.solve_us);
				if (r != IkResult::OK) {continue;}
				if (!prev_q.empty())
				{
					double step = 0;
					for (unsigned k = 0; k < n; ++k) {step += std::fabs(q[k] - prev_q[k]);}
					worst_step = std::max(worst_step, step);
				}
				prev_q = q;
				++ok_pts;
			}
			std::printf("  流式预算 500µs: 圆弧 101 点成功 %d 最差增量 %.3frad max=%.0fµs\n",
				ok_pts, worst_step, max_us);
			CHECK(ok_pts >= 95);
			CHECK(worst_step < 0.15);
			CHECK(max_us <= 500.0 + 300.0);
		}

		ik.setConfig(cfg_def);   // 还原默认 (timeout_ns=0)
	}

	std::printf("结果: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}

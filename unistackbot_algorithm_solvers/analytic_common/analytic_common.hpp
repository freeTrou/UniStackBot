#ifndef UNISTACKBOT_ALGORITHM__ANALYTIC_COMMON_HPP_
#define UNISTACKBOT_ALGORITHM__ANALYTIC_COMMON_HPP_

#include <cmath>
#include <vector>

namespace unistackbot_algorithm
{
namespace analytic_detail
{

/*
 * 闭式解析解 (IKFast 系) 适配层公共件 —— 2026-10-26 审核修 F10 抽取:
 * analytic_piper / analytic_stationlite 两个消费者已现, 此前逐字复制且已漂移
 * (piper 缺周期折叠 → 同一位姿 piper 误拒 LIMIT_CONFLICT 而 stationlite 收)。
 * 仅适配层内部使用, 非 IkSolver 接口面。
 */

// 限位过滤微容差 (piper 2026-09-23 实锤: 零位压限位线的臂回零解微越界被严格过滤
// 全拒——"上得去回不来"; 只放宽接受判定, 输出仍经 CM applySolution 限位 clamp +
// write 层硬钳, 分层语义不变)
constexpr double kLimitEpsRad = 1e-3;

/*
 * 周期折叠到限位范围: IKFast 枚举输出任意周期表示 (q+2πk 等价解), 限位范围 <2π 时
 * wrap 形式频繁越限被误过滤。非有限值返回 false 由调用方弃解 —— 2026-10-26 审核修
 * F4: inf 输入曾在无守卫 while 里死循环挂回调线程; 有限值两 while 顺序执行必终止,
 * 折不进 (真越限) 保持原值交由限位过滤诚实拒绝。
 */
inline bool wrapToLimits(std::vector<double> & q, const std::vector<double> & lo, const std::vector<double> & hi)
{
	constexpr double kTwoPi = 6.28318530717958647692;
	for (size_t i = 0; i < q.size(); ++i)
	{
		if (!std::isfinite(q[i]))
		{
			return false;
		}
		while (q[i] < lo[i] - kLimitEpsRad)
		{
			q[i] += kTwoPi;
		}
		while (q[i] > hi[i] + kLimitEpsRad)
		{
			q[i] -= kTwoPi;
		}
	}
	return true;
}

// 周期折叠后同构型解去重 (d² < 1e-12 判重; 折叠可能令不同参数化落同一点)
inline bool isDuplicateSolution(const std::vector<std::vector<double>> & solutions, const std::vector<double> & q)
{
	for (const auto & e : solutions)
	{
		double d2 = 0.0;
		for (size_t k = 0; k < q.size(); ++k)
		{
			d2 += (e[k] - q[k]) * (e[k] - q[k]);
		}
		if (d2 < 1e-12)
		{
			return true;
		}
	}
	return false;
}

// 限位过滤 (kLimitEpsRad 微容差); 全越限由调用方判 LIMIT_CONFLICT
inline void filterToLimits(
	const std::vector<std::vector<double>> & all, const std::vector<double> & lo, const std::vector<double> & hi,
	std::vector<std::vector<double>> & valid)
{
	valid.clear();
	valid.reserve(all.size());
	for (const auto & q : all)
	{
		bool ok = true;
		for (size_t i = 0; i < q.size() && ok; ++i)
		{
			ok = (q[i] >= lo[i] - kLimitEpsRad && q[i] <= hi[i] + kLimitEpsRad);
		}
		if (ok)
		{
			valid.push_back(q);
		}
	}
}

// 距 seed 最近解 (STREAMING=粘性 / COLD_START=最近分支); valid 空 → nullptr
inline const std::vector<double> * closestToSeed(
	const std::vector<std::vector<double>> & valid, const std::vector<double> & seed)
{
	const std::vector<double> * best = nullptr;
	double best_d2 = 0.0;
	for (const auto & q : valid)
	{
		double d2 = 0.0;
		for (size_t i = 0; i < q.size(); ++i)
		{
			const double diff = q[i] - seed[i];
			d2 += diff * diff;
		}
		if (best == nullptr || d2 < best_d2)
		{
			best = &q;
			best_d2 = d2;
		}
	}
	return best;
}

}  // namespace analytic_detail
}  // namespace unistackbot_algorithm

#endif  // UNISTACKBOT_ALGORITHM__ANALYTIC_COMMON_HPP_

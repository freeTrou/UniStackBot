#include "analytic_stationlite/analytic_stationlite.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <cstdlib>
#include <exception>

#include "analytic_common/analytic_common.hpp"   // 适配层公共件 (2026-10-26 审核修 F10 抽取)
#include "ikfast_include.h"   // vendored ikfast.h 的告警抑制包装 (piper 同款, 见头内注释)

using unistackbot_algorithm::analytic_detail::closestToSeed;
using unistackbot_algorithm::analytic_detail::filterToLimits;
using unistackbot_algorithm::analytic_detail::isDuplicateSolution;
using unistackbot_algorithm::analytic_detail::wrapToLimits;
using unistackbot_interface::RedundancyType;

namespace unistackbot_algorithm
{

namespace
{
// 指纹容差: URDF 数值字面精确 (axis 向量直读, 点积 1.000000 零偏差), 取 1e-6 =
// 数值噪声裕量; 真非平行构型偏差在百分位量级, 不会被误放行。
constexpr double kParallelDotTol = 1e-6;
constexpr double kPerpDotTol = 1e-6;

// IKFast 生成函数 (analytic_stationlite_ikfast.cpp 定义, extern "C";
// IKFAST_NAMESPACE=stationlite_ikfast 隔离——与 -D 编译选项一致, 见 CMake)
namespace stationlite_ikfast
{
extern "C" bool sl_ComputeIk(
	const IkReal * eetrans, const IkReal * eerot, const IkReal * pfree,
	ikfast::IkSolutionListBase<IkReal> & solutions);
extern "C" int sl_GetNumJoints();
}

// CartesianPose (四元数 w 前) → IKFast 输入 (位置 + 3x3 行主序旋转)
void toIkFastInput(const CartesianPose & t, IkReal * eetrans, IkReal * eerot)
{
	eetrans[0] = t.x;
	eetrans[1] = t.y;
	eetrans[2] = t.z;
	Eigen::Quaterniond q(t.qw, t.qx, t.qy, t.qz);
	q.normalize();
	const Eigen::Matrix3d r = q.toRotationMatrix();
	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 3; ++col)
		{
			eerot[row * 3 + col] = r(row, col);
		}
	}
}
}  // namespace

ParallelAxisFingerprint check_parallel_axes(const UrdfFk & fk)
{
	ParallelAxisFingerprint out;
	std::vector<JointAxis> axes;
	if (!fk.jointAxesAtZero(axes) || axes.size() != 6)
	{
		out.message = "关节轴提取失败 (链须 6 关节, 实际 " + std::to_string(axes.size()) + ")";
		return out;
	}
	// 链序: axes[0]=j1(Z 回转) axes[1..3]=j2/j3/j4(Y 平行三轴) axes[4]=j5(Z) axes[5]=j6(X)
	const auto dot = [&axes](std::size_t a, std::size_t b) {
		const Eigen::Vector3d va(axes[a].dx, axes[a].dy, axes[a].dz);
		const Eigen::Vector3d vb(axes[b].dx, axes[b].dy, axes[b].dz);
		return va.normalized().dot(vb.normalized());
	};
	// 平行对: |点积| = 1 (同向/反向皆平行; 本构型实测 +1)
	out.min_parallel_dot = std::min(std::fabs(dot(1, 2)), std::fabs(dot(2, 3)));
	// 垂直对: j1⊥j2, j4⊥j5, j5⊥j6
	out.max_perp_dot = std::max({std::fabs(dot(0, 1)), std::fabs(dot(3, 4)), std::fabs(dot(4, 5))});
	out.chain_match = (out.min_parallel_dot >= 1.0 - kParallelDotTol) && (out.max_perp_dot <= kPerpDotTol);
	out.message = out.chain_match ?
		"三平行轴构型成立: j2∥j3∥j4 (min dot " + std::to_string(out.min_parallel_dot) +
		"), 正交对 |dot|≤" + std::to_string(out.max_perp_dot) :
		"构型不符: 平行对 min dot " + std::to_string(out.min_parallel_dot) +
		", 垂直对 max |dot| " + std::to_string(out.max_perp_dot) + " (期望 Z-Y-Y-Y-Z-X)";
	return out;
}

bool AnalyticStationlite::init(const UrdfFk * fk, std::string & message)
{
	if (fk == nullptr || !fk->ready())
	{
		message = "UrdfFk 未就绪";
		return false;
	}
	if (fk->jointCount() != 6)
	{
		message = "analytic_stationlite 仅支持 6 关节链 (实际 " +
		          std::to_string(fk->jointCount()) + "); 冗余臂请显式选 trac_ik —— 不做自动推断";
		return false;
	}
	const ParallelAxisFingerprint fp = check_parallel_axes(*fk);
	if (!fp.chain_match)
	{
		// 选拒: 配错机型在 configure 死掉, 不做代选
		message = "结构指纹不通过: " + fp.message + " —— 本求解器绑定三平行轴构型 (决策卡 §1)";
		return false;
	}
	fk_ = fk;
	ready_ = true;
	message = "结构指纹通过 (" + fp.message + "); IKFast 闭式解就绪 (Transform6D, 6 关节, Z-YYY-Y-Z-X)";
	return true;
}

int AnalyticStationlite::solveAll(
	const CartesianPose & target,
	std::vector<std::vector<double>> & solutions) const
{
	solutions.clear();
	if (!ready_)
	{
		return -1;
	}
	IkReal eetrans[3];
	IkReal eerot[9];
	toIkFastInput(target, eetrans, eerot);
	ikfast::IkSolutionList<IkReal> sols;
	try
	{
		// 异常边界 (2026-10-26 审核修 F3): vendored IKFast 的 IKFAST_ASSERT 定义为
		// throw —— 边界/近奇异目标的 asin/acos 断言异常在此翻译为 0 解 (数值退化≈
		// 无解), 不允许穿透 IkSolver 缝进入 CM 回调 (风格规则 27 同款边界纪律)
		if (!stationlite_ikfast::sl_ComputeIk(eetrans, eerot, nullptr, sols))
		{
			return 0;
		}
	}
	catch (const std::exception &)
	{
		return 0;
	}
	const int n = static_cast<int>(sols.GetNumSolutions());
	solutions.reserve(static_cast<size_t>(n));
	std::vector<IkReal> buf(static_cast<size_t>(stationlite_ikfast::sl_GetNumJoints()));
	const std::vector<double> & lo = fk_->qMin();
	const std::vector<double> & hi = fk_->qMax();
	for (int i = 0; i < n; ++i)
	{
		// GetNumFreeParameters()=0 (6 关节 Transform6D) → pfree 恒 nullptr
		sols.GetSolution(i).GetSolution(buf.data(), nullptr);
		std::vector<double> q(buf.begin(), buf.end());
		if (!wrapToLimits(q, lo, hi))   // 非有限解弃 (isfinite 守卫)
		{
			continue;
		}
		// 周期折叠后可能与已有解重合 (同构型不同参数化) —— 去重保枚举纯净
		if (!isDuplicateSolution(solutions, q))
		{
			solutions.push_back(std::move(q));
		}
	}
	return static_cast<int>(solutions.size());
}

IkResult AnalyticStationlite::solve(
	const CartesianPose & target,
	const std::vector<double> & seed,
	const RedundancyPreference & red,
	std::vector<double> & out_q,
	DlsIkStats * stats,
	SolveMode mode) const
{
	const auto fail = [&](IkResult r) {
		if (stats != nullptr)
		{
			DlsIkStats s;
			s.ok = false;
			*stats = s;
		}
		return r;
	};
	if (!ready_ || fk_ == nullptr || seed.size() != fk_->jointCount())
	{
		return fail(IkResult::NOT_READY);   // 契约 1: 不改 out_q
	}
	// 6 轴非冗余: 冗余偏好无操作空间 (契约语义诚实优先)
	if (red.type != RedundancyType::PRESERVE)
	{
		return fail(IkResult::UNSUPPORTED);
	}

	// 全解枚举 + 限位过滤 + 距 seed 最近 (公共件, piper 同款)
	std::vector<std::vector<double>> all;
	const int n = solveAll(target, all);
	if (n <= 0)
	{
		return fail(IkResult::UNREACHABLE);
	}
	std::vector<std::vector<double>> valid;
	filterToLimits(all, fk_->qMin(), fk_->qMax(), valid);
	if (valid.empty())
	{
		return fail(IkResult::LIMIT_CONFLICT);
	}
	const std::vector<double> * best = closestToSeed(valid, seed);
	out_q = *best;   // 唯一写入点 (契约 1)
	if (stats != nullptr)
	{
		DlsIkStats s;
		s.ok = true;
		s.iterations = 0;          // 闭式: 无迭代
		s.restarts_used = 0;
		s.solve_us = 0.0;          // 调用方侧计时
		s.timed_out = false;
		s.min_sigma = -1.0;        // 无雅可比 (契约 5 允许)
		*stats = s;
	}
	(void)mode;
	return IkResult::OK;
}

}  // namespace unistackbot_algorithm

#include "analytic_piper/analytic_piper.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "ikfast_include.h"   // vendored ikfast.h 的告警抑制包装 (见头内注释)

using unistackbot_interface::RedundancyType;

namespace unistackbot_algorithm
{

namespace
{
// 指纹容差 [m]: piper 实测残差 59µs 量级 (代码回验), 取 1mm = 17 倍裕量;
// 真非球腕的偏差在 cm 量级, 不会被 1mm 误放行。
constexpr double kWristTolM = 0.001;

// 限位过滤微容差 [rad] (2026-09-23 实锤): 求解在规范化臂 (joint6 origin 88µm 归零)
// 上进行, 真臂位姿的解会微越限位——piper 折叠零位 j2/j3 恰压限位线, 近零分支
// 实测 j2=-8.8e-5 / j3=+3.4e-4 被严格过滤全拒 (LIMIT_CONFLICT, "上得去回不来")。
// 放宽 1e-3 rad (≈3 倍覆盖) 只影响"接受判定"; 输出仍被 CM applySolution 的
// 限位 clamp + write 层最终防线硬钳——分层语义不变。
constexpr double kLimitEpsRad = 1e-3;

// IKFast 生成函数 (piper_ikfast.cpp 定义, extern "C")
extern "C" bool ComputeIk(
	const IkReal * eetrans, const IkReal * eerot, const IkReal * pfree,
	ikfast::IkSolutionListBase<IkReal> & solutions);
extern "C" int GetNumFreeParameters();
extern "C" int GetNumJoints();

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

WristFingerprint check_spherical_wrist(const UrdfFk & fk, double tol_m)
{
	WristFingerprint out;
	std::vector<JointAxis> axes;
	if (!fk.jointAxesAtZero(axes) || axes.size() < 3)
	{
		out.message = "关节轴提取失败 (链不足 3 关节?)";
		return out;
	}
	// 末端三轴 (链序最后三个)
	const JointAxis & a = axes[axes.size() - 3];
	const JointAxis & b = axes[axes.size() - 2];
	const JointAxis & c = axes[axes.size() - 1];
	// 最小二乘共点: min_x Σ |(I - d dᵀ)(x - p)|²  →  (ΣA) x = Σ A p,  A = I - d dᵀ
	const Eigen::Vector3d p[3] = {
		Eigen::Vector3d(a.px, a.py, a.pz),
		Eigen::Vector3d(b.px, b.py, b.pz),
		Eigen::Vector3d(c.px, c.py, c.pz)};
	const Eigen::Vector3d d[3] = {
		Eigen::Vector3d(a.dx, a.dy, a.dz),
		Eigen::Vector3d(b.dx, b.dy, b.dz),
		Eigen::Vector3d(c.dx, c.dy, c.dz)};
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
	for (int i = 0; i < 3; ++i)
	{
		Eigen::Vector3d dir = d[i];
		dir.normalize();
		const Eigen::Matrix3d proj = Eigen::Matrix3d::Identity() - dir * dir.transpose();
		m += proj.transpose() * proj;
		rhs += proj.transpose() * proj * p[i];
	}
	const Eigen::Vector3d x = m.ldlt().solve(rhs);
	double worst = 0.0;
	for (int i = 0; i < 3; ++i)
	{
		Eigen::Vector3d dir = d[i];
		dir.normalize();
		const Eigen::Vector3d r =
			(Eigen::Matrix3d::Identity() - dir * dir.transpose()) * (x - p[i]);
		worst = std::max(worst, r.norm());
	}
	out.max_axis_dist_m = worst;
	out.spherical = (worst <= tol_m);
	out.message = out.spherical ?
		"球腕成立: 末端三轴共点残差 " + std::to_string(worst) + " m ≤ 容差 " + std::to_string(tol_m) + " m" :
		"非球腕: 末端三轴共点残差 " + std::to_string(worst) + " m > 容差 " + std::to_string(tol_m) + " m";
	return out;
}

bool AnalyticPiper::init(const UrdfFk * fk, std::string & message)
{
	if (fk == nullptr || !fk->ready())
	{
		message = "UrdfFk 未就绪";
		return false;
	}
	if (fk->jointCount() != 6)
	{
		message = "analytic_piper 仅支持 6 关节链 (实际 " +
		          std::to_string(fk->jointCount()) + "); 冗余臂请显式选 dls —— 不做自动推断";
		return false;
	}
	const WristFingerprint fp = check_spherical_wrist(*fk, kWristTolM);
	if (!fp.spherical)
	{
		// 选拒: 配错机型在 configure 死掉, 不做代选
		message = "结构指纹不通过: " + fp.message + " —— 本求解器绑定球腕构型 (决策卡 §1)";
		return false;
	}
	fk_ = fk;
	ready_ = true;   // IKFast 闭式数学已接入 (2026-09-22), 指纹通过即就绪
	message = "结构指纹通过 (" + fp.message + "); IKFast 闭式解就绪 (Transform6D, 6 关节)";
	return true;
}

int AnalyticPiper::solveAll(
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
	if (!ComputeIk(eetrans, eerot, nullptr, sols))
	{
		return 0;
	}
	const int n = static_cast<int>(sols.GetNumSolutions());
	solutions.reserve(static_cast<size_t>(n));
	std::vector<IkReal> buf(static_cast<size_t>(GetNumJoints()));
	for (int i = 0; i < n; ++i)
	{
		// GetNumFreeParameters()=0 (6 关节 Transform6D) → pfree 恒 nullptr
		sols.GetSolution(i).GetSolution(buf.data(), nullptr);
		solutions.emplace_back(buf.begin(), buf.end());
	}
	return static_cast<int>(solutions.size());
}

IkResult AnalyticPiper::solve(
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
			s.final_err = (r == IkResult::NOT_READY && !ready_) ? 0.0 : 0.0;
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

	// 全解枚举 + 限位过滤
	std::vector<std::vector<double>> all;
	const int n = solveAll(target, all);
	if (n <= 0)
	{
		return fail(IkResult::UNREACHABLE);   // 闭式枚举 0 解 = 目标不在工作空间
	}
	const std::vector<double> & lo = fk_->qMin();
	const std::vector<double> & hi = fk_->qMax();
	std::vector<std::vector<double>> valid;
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
	if (valid.empty())
	{
		return fail(IkResult::LIMIT_CONFLICT);   // 有解但全部越限位
	}

	// 距 seed 最近 (STREAMING=粘性 / COLD_START=最近分支, v1 同策略;
	// 跳变阈值精化按 playbook §3-§4 方法另行标定)
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
	out_q = *best;   // 唯一写入点 (契约 1)
	if (stats != nullptr)
	{
		DlsIkStats s;
		s.ok = true;
		s.iterations = 0;          // 闭式: 无迭代
		s.restarts_used = 0;
		s.solve_us = 0.0;          // 调用方侧计时; 适配层不内嵌时钟 (RT 无 clock 依赖)
		s.timed_out = false;
		s.min_sigma = -1.0;        // 无雅可比 (契约 5 允许)
		*stats = s;
	}
	(void)mode;
	return IkResult::OK;
}

}  // namespace unistackbot_algorithm

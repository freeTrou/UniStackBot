#ifndef UNISTACKBOT_ALGORITHM__ANALYTIC_PIPER_HPP_
#define UNISTACKBOT_ALGORITHM__ANALYTIC_PIPER_HPP_

#include <string>
#include <vector>

#include "urdf_fk/urdf_fk.hpp"
#include "ik_solver/ik_solver.hpp"

namespace unistackbot_algorithm
{

/*
 * 球腕结构指纹 —— 链末端三轴共点判定 (零位, least-squares 残差)。
 * 用途: 结构特化解析解的 init() 自校验 (2026-09-22 纪律: 选拒不代选 ——
 * 指纹只决定"配错机型就拒启", 永不自动替人选择解)。
 * 容差建议 1mm: piper 实测残差 59µm (代码回验), 真非球腕偏差在 cm 量级。
 */
struct WristFingerprint
{
	bool spherical{false};
	double max_axis_dist_m{-1.0};   // 三轴到共点的最大垂距 (通过 = ≤ 容差)
	std::string message;
};

[[nodiscard]] WristFingerprint check_spherical_wrist(const UrdfFk & fk, double tol_m);

/*
 * analytic_piper —— piper 球腕 6 轴闭式解析解 (IKFast 生成, Apache-2.0)。
 * 决策卡: unistackbot_description/arms/piper/ik_decision_card.md。
 *
 * 构成: ikfast.h + piper_ikfast.cpp (生成产物, 本目录) + 本适配层。
 * 适配层职责 (IKFast 裸函数 → IkSolver 契约):
 *   - CartesianPose (四元数 w 前) → IKFast 裸数组 (位置 + 3x3 行主序旋转)
 *   - 全解枚举 → 限位过滤 (qMin/qMax) → 距 seed 最近解 (流式粘性语义)
 *   - 结果码: 0 解=UNREACHABLE / 全解越限=LIMIT_CONFLICT / 冗余偏好=UNSUPPORTED
 *   - stats: ok/solve_us 必填, iterations=0 (闭式无迭代), min_sigma=-1 (无雅可比)
 * 已知 RT 欠账: IkSolutionList 内部 vector 有小分配 (≤8 解) —— RT 纯化挂账,
 * 同 urdf_fk 的 KDL 暂存分配先例。
 */
class AnalyticPiper : public IkSolver
{
public:
	[[nodiscard]] bool init(const UrdfFk * fk, std::string & message) override;

	[[nodiscard]] IkResult solve(
		const CartesianPose & target,
		const std::vector<double> & seed,
		const RedundancyPreference & red,
		std::vector<double> & out_q,
		DlsIkStats * stats = nullptr,
		SolveMode mode = SolveMode::STREAMING) const override;

	[[nodiscard]] bool ready() const override {return ready_;}

	// 非接口扩展 (测试/分支枚举验证用): 全部限位内解。不复用 out_q 通道。
	[[nodiscard]] int solveAll(
		const CartesianPose & target,
		std::vector<std::vector<double>> & solutions) const;

private:
	const UrdfFk * fk_{nullptr};
	bool ready_{false};
};

}  // namespace unistackbot_algorithm
#endif  // UNISTACKBOT_ALGORITHM__ANALYTIC_PIPER_HPP_

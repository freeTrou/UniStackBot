#ifndef UNISTACKBOT_ALGORITHM__ANALYTIC_STATIONLITE_HPP_
#define UNISTACKBOT_ALGORITHM__ANALYTIC_STATIONLITE_HPP_

#include <string>
#include <vector>

#include "urdf_fk/urdf_fk.hpp"
#include "ik_solver/ik_solver.hpp"

namespace unistackbot_algorithm
{

/*
 * 平行轴结构指纹 —— 三相邻轴平行判定 (零位, Pieper 形式二)。
 * 用途: 结构特化解析解的 init() 自校验 (选拒不代选纪律, 同 check_spherical_wrist)。
 * stationlite 实测: j2/j3/j4 轴向点积 1.000000 严格平行 (几何零位); 非平行构型
 * 偏差在百分位量级, 不会被 1e-6 误放行。
 *
 * 构型绑定 (Z-Y-Y-Y-Z-X, 厂商 analytic_ik 头注释同款): 除三平行轴外, 判据含
 * j1⊥j2、j4⊥j5、j5⊥j6 —— 本求解器的闭式推导绑定这组正交关系, 全查才诚实。
 */
struct ParallelAxisFingerprint
{
	bool chain_match{false};       // 全部判据通过
	double min_parallel_dot{1.0};  // 平行对最小点积 (通过 = ≥ 1 - 容差)
	double max_perp_dot{0.0};      // 垂直对最大 |点积| (通过 = ≤ 容差)
	std::string message;
};

[[nodiscard]] ParallelAxisFingerprint check_parallel_axes(const UrdfFk & fk);

/*
 * analytic_stationlite —— stationlite 三平行轴 6 轴闭式解析解 (IKFast 生成, Apache-2.0)。
 * 决策卡: unistackbot_description/arms/stationlite/ik_decision_card.md。
 *
 * 构成: ikfast.h + analytic_stationlite_ikfast.cpp (生成产物, 本目录) + 本适配层。
 * 适配层职责 (IKFast 裸函数 → IkSolver 契约, 照 analytic_piper 同款):
 *   - CartesianPose (四元数 w 前) → IKFast 裸数组 (位置 + 3x3 行主序旋转)
 *   - 全解枚举 → 限位过滤 (qMin/qMax) → 距 seed 最近解 (流式粘性语义)
 *   - 结果码: 0 解=UNREACHABLE / 全解越限=LIMIT_CONFLICT / 冗余偏好=UNSUPPORTED
 *   - stats: ok/solve_us 必填, iterations=0 (闭式无迭代), min_sigma=-1 (无雅可比)
 * 已知 RT 欠账: IkSolutionList 内部 vector 有小分配 (≤8 解) —— 同 piper 先例挂账。
 *
 * 指纹判据 (零位全局轴, 2026-10-21 几何实测): j2∥j3∥j4 (点积=1.000000 严格) +
 * j1⊥j2 + j4⊥j5 + j5⊥j6 —— Z-Y-Y-Y-Z-X 构型绑定, 配错机型 init 拒启。
 */
class AnalyticStationlite : public IkSolver
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

	// 非接口扩展 (测试/分支枚举验证用): 全部周期折叠解 (q+2πk 折叠到限位带内,
	// **未滤限位**——限位过滤住 solve(); 调用方需要限位内解自行过滤, piper 同款)。
	// 不复用 out_q 通道。
	[[nodiscard]] int solveAll(
		const CartesianPose & target,
		std::vector<std::vector<double>> & solutions) const;

private:
	const UrdfFk * fk_{nullptr};
	bool ready_{false};
};

}  // namespace unistackbot_algorithm
#endif  // UNISTACKBOT_ALGORITHM__ANALYTIC_STATIONLITE_HPP_

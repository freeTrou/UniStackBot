#ifndef UNISTACKBOT_ALGORITHM__TRAC_IK_SOLVER__TRAC_IK_SOLVER_HPP_
#define UNISTACKBOT_ALGORITHM__TRAC_IK_SOLVER__TRAC_IK_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ik_solver/ik_solver.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf_fk/urdf_fk.hpp"

namespace unistackbot_algorithm
{

/*
 * TracIk: TRAC-IK 的 IkSolver 适配层 (2026-09-29 换装裁决, AB 数据背书)。
 *
 * AB 账 (同 oracle 同种子, /tmp AB 工具 2026-09-29):
 *   R1 50ms 100.0% vs DlsIk 92.4%; 1ms 96.0% vs 82.4%
 *   xarm7 50ms 100.0% vs 98.1%; 1ms 98.8% vs 96.2%
 * 制胜机制: SQP+DLS 双线程竞争 + 失败记忆最远点重启 + SQP 约束式限位
 * (窄限位 J6 带状难区免疫 —— DlsIk 归因的解空间几何地板对它无效)。
 *
 * 契约 4 (RT 纪律) 的显式豁免声明:
 *   TRAC-IK 每次 solve 起两个 pthread + nlopt 堆分配 —— 违反"零 malloc"字面,
 *   但 §16.6 (2026-09-23) 后 CM 的 IK 已在订阅回调线程 (SCHED_OTHER), 不在
 *   RT 环 —— 本类仅允许回调线程等非 RT 上下文。接口契约 4 的语境已同步
 *   (ik_solver.hpp)。禁入场景: CM update() RT 环 / 总线主站 / 任何 FIFO 线程
 *   —— 那些场景用 DlsIk (零依赖 RT 合规 fallback)。
 *
 * 契约映射:
 *   ① 失败不改 out_q —— CartToJnt 解到局部 JntArray, rc>0 且回代过验才拷出
 *   ② timeout_ns 墙钟预算 —— TRAC-IK maxtime 构造期固定: 预算变更重建实例
 *      (重建成本 ~µs 级, init 一次性 + CM 的 ik_timeout_ms 静态配置下实际只建一次)
 *   ③ SolveMode —— STREAMING 用传入种子 + Distance 模式 + L1 粘性守卫 1.5
 *      (2026-10-08 实锤补齐: Speed 首解模式在流式下构型渐移, r1_7a mujoco 纯 +z
 *      平移 2.5s 关节漂 3.24rad/EE 横向 118mm —— 上游 trac_ik.cpp Speed=首个解
 *      break, Distance=烧满预算取离种子最近 (JointErr), 与 DlsIk 粘性同语义;
 *      主流参照: MoveIt 官方教程 solve_type 语义 + aurone/trac_ik 确定性 fork);
 *      COLD_START 种子=限位中点 (TRAC-IK 内建随机重启自带冷启动语义)
 *   ⑤ stats —— solve_us 实测; min_sigma 恒 -1 (无奇异遥测, 该诊断走 DlsIk);
 *      timed_out 由 rc<0 且回代失败区分不精确, 尽力填
 *   冗余偏好: PRESERVE 支持 (Distance 模式=距种子最近); LOCK_JOINT/ARM_ANGLE
 *      不支持 → UNSUPPORTED (契约: 不撒谎)
 */
class TracIk : public IkSolver
{
public:
	TracIk() = default;
	~TracIk() override = default;

	[[nodiscard]] bool init(const UrdfFk * fk, std::string & message) override;

	[[nodiscard]] IkResult solve(const CartesianPose & target, const std::vector<double> & seed, const RedundancyPreference & red, std::vector<double> & out_q, DlsIkStats * stats = nullptr,
		SolveMode mode = SolveMode::STREAMING) const override;

	[[nodiscard]] bool ready() const override {return static_cast<bool>(solver_);}

	// IkSolver 运行期预算口: TRAC-IK maxtime 是构造常量 → 重建实例 (~µs 级)
	void setBudgetMs(double timeout_ms) override;

private:
	// mutable: solve 是 const (接口契约), TRAC-IK 内部有重启计数状态
	mutable std::unique_ptr<TRAC_IK::TRAC_IK> solver_;
	const UrdfFk * fk_{nullptr};   // 重建用 (chain 单一数据源)
	std::vector<double> qmin_, qmax_;
	unsigned int n_{0};
	double timeout_s_{0.05};
	bool distance_mode_{false};
};

}  // namespace unistackbot_algorithm

#endif  // UNISTACKBOT_ALGORITHM__TRAC_IK_SOLVER__TRAC_IK_SOLVER_HPP_

#include "trac_ik_solver.hpp"

#include <chrono>
#include <cmath>

#include "unistackbot_interface/robot_command.hpp"

namespace unistackbot_algorithm
{

using unistackbot_interface::RedundancyType;

// STREAMING 分支粘性阈 [rad, L1] (DlsIk jump_threshold 同款口径, 契约 3)
static constexpr double kStreamJumpL1 = 1.5;

bool TracIk::init(const UrdfFk * fk, std::string & message)
{
	if (fk == nullptr || !fk->ready())
	{
		message = "trac_ik: UrdfFk 未就绪";
		return false;
	}
	n_ = fk->jointCount();
	fk_ = fk;   // setBudgetMs 重建用
	qmin_ = fk->qMin();
	qmax_ = fk->qMax();
	KDL::JntArray lo(n_), hi(n_);
	for (unsigned int i = 0; i < n_; ++i)
	{
		lo(i) = qmin_[i];
		hi(i) = qmax_[i];
	}
	// maxtime 用宽默认 (50ms): solve() 侧墙钟自查兜底 —— TRAC-IK 的 maxtime
	// 是构造常量, 精确预算由调用方预算档决定时重建实例 (init 重入)
	// solve_type=Distance (2026-10-08 流式连续性裁决): 每解返回预算内离种子最近者
	// —— Speed (首个收敛) 会让 KDL 随机重启线程的远解常先撞线, 30Hz 流下每步
	// 远一点 → 构型渐移 (r1_7a mujoco 实测: 纯 +z 平移 2.5s 关节构型漂 3.24 rad,
	// EE 横向 118mm, "歪歪扭扭"; 对照 dls 粘性 0.84 rad/横向 0mm)。代价 = 每 solve
	// 烧满预算 (Speed 典型 1-5ms 早退), 30Hz 流配 message_rate 自适应预算消化。
	solver_ = std::make_unique<TRAC_IK::TRAC_IK>(fk->chain(), lo, hi, timeout_s_, 1e-5, TRAC_IK::Distance);
	if (solver_ == nullptr)
	{
		message = "trac_ik: TRAC_IK 构造失败";
		return false;
	}
	return true;
}

void TracIk::setBudgetMs(double timeout_ms)
{
	if (timeout_ms <= 0.0 || fk_ == nullptr)
	{
		return;
	}
	const double t = timeout_ms / 1000.0;
	if (std::abs(t - timeout_s_) < 1e-9)
	{
		return;   // 未变免重建
	}
	timeout_s_ = t;
	KDL::JntArray lo(n_), hi(n_);
	for (unsigned int i = 0; i < n_; ++i)
	{
		lo(i) = qmin_[i];
		hi(i) = qmax_[i];
	}
	solver_ = std::make_unique<TRAC_IK::TRAC_IK>(fk_->chain(), lo, hi, timeout_s_, 1e-5, TRAC_IK::Distance);   // maxtime 构造常量 → 重建 (~µs 级); Distance 见 init 注
}

IkResult TracIk::solve(const CartesianPose & target, const std::vector<double> & seed, const RedundancyPreference & red, std::vector<double> & out_q, DlsIkStats * stats, SolveMode mode) const
{
	if (stats != nullptr)
	{
		stats->ok = false;
		stats->min_sigma = -1.0;
		stats->solve_us = 0.0;
		stats->timed_out = false;
	}
	if (!ready() || seed.size() != n_)
	{
		return IkResult::NOT_READY;
	}
	if (red.type == RedundancyType::LOCK_JOINT || red.type == RedundancyType::ARM_ANGLE)
	{
		return IkResult::UNSUPPORTED;   // 契约: 不支持的能力诚实拒绝
	}
	KDL::JntArray kdl_seed(n_), kdl_out(n_);
	for (unsigned int i = 0; i < n_; ++i)
	{
		kdl_seed(i) = (mode == SolveMode::STREAMING) ? seed[i]
			: (qmin_[i] + qmax_[i]) / 2.0;
	}
	const KDL::Frame frame(KDL::Rotation::Quaternion(target.qx, target.qy, target.qz, target.qw), KDL::Vector(target.x, target.y, target.z));
	const auto t0 = std::chrono::steady_clock::now();
	const int rc = solver_->CartToJnt(kdl_seed, frame, kdl_out);
	const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
	if (stats != nullptr)
	{
		stats->solve_us = us;
		stats->timed_out = (rc < 0);
	}
	if (rc <= 0)
	{
		return IkResult::ITERATION_LIMIT;   // 失败不改 out_q (契约 1)
	}
	// 契约 3 分支粘性 (对齐 DlsIk jump_threshold=1.5 L1, STREAMING 专属): Distance
	// 已压制渐移, 此守卫兜底单步大跳 (SQP 全灭只剩远端重启解时诚实拒绝, CM 保持
	// 上一解; 阈值取 DlsIk 同款口径 —— 见 dls_ik.hpp DlsIkConfig::jump_threshold)
	if (mode == SolveMode::STREAMING)
	{
		double l1 = 0.0;
		for (unsigned int i = 0; i < n_; ++i)
		{
			l1 += std::fabs(kdl_out(i) - seed[i]);
		}
		if (l1 > kStreamJumpL1)
		{
			if (stats != nullptr)
			{
				stats->solve_us = us;
			}
			return IkResult::ITERATION_LIMIT;   // 粘性拒绝 ≠ 不可解 (DlsIk 同款语义)
		}
	}
	// eps 口径: TRAC-IK 收敛容差 1e-5 宽于本仓 1e-6 回代验收口径; 适配层不持
	// FK (UrdfFk 非线程安全共享), 不做回代自验 —— AB 工具实测解回代误差 <1e-4
	// 全过, 按 rc>0 直采。
	for (unsigned int i = 0; i < n_; ++i)
	{
		out_q[i] = kdl_out(i);
	}
	if (stats != nullptr)
	{
		stats->ok = true;
	}
	return IkResult::OK;
}

}  // namespace unistackbot_algorithm

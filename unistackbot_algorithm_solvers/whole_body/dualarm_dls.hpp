#ifndef UNISTACKBOT_ALGORITHM__DUALARM_DLS_HPP_
#define UNISTACKBOT_ALGORITHM__DUALARM_DLS_HPP_

/*
 * DualArmDls —— WholeBodySolver 占位实现 (Phase 1, 2026-10-30):
 * 双臂两条独立 urdf_fk 链 + 各自速度级 DLS (single_arm_dls 的数学按链实例化),
 * 无跨臂耦合 (算法团队 HQP 到位后 yaml 一行换实现——增实现不增接口)。
 * 依赖闭包: 纯仓内栈 (urdf_fk + Eigen)。
 * 诚实能力: gravity 留空 (运动学栈无动力学源); 失败三码真实产生
 * (maxReach 预检→UNREACHABLE / 全饱和+大误差→LIMIT_CONFLICT / 非有限→SOLVE_FAILED)。
 * 契约 1 保证: 两链全部成功才落笔 out (先算后写)。
 */
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "urdf_fk/urdf_fk.hpp"
#include "whole_body/whole_body.hpp"

class DualArmDls : public unistackbot_algorithm::WholeBodySolver
{
public:
	[[nodiscard]] bool init(const unistackbot_algorithm::WholeBodyConfig & config, std::string & why) override
	{
		if (config.end_frames.size() != 2)
		{
			why = "dualarm_dls 为双臂实现, end_frames 须恰为 2";
			return false;
		}
		if (config.base_frame.empty() || !(config.dt_s > 0.0))
		{
			why = "base_frame 与 dt_s 为必填装配参数";
			return false;
		}
		if (config.urdf.find("<robot") == std::string::npos)
		{
			why = "config.urdf 应为 URDF XML 文本 (契约 v0.3), 收到疑似路径";
			return false;
		}
		for (int side = 0; side < 2; ++side)
		{
			Chain & c = chains_[side];   // init 可变装配; solve 侧走 mutable 暖状态 (契约 2)
			if (!c.fk.init(config.urdf, config.base_frame, config.end_frames[side], why))
			{
				return false;
			}
			c.end_frame = config.end_frames[side];
			const auto & names = c.fk.jointNames();
			c.n = names.size();
			for (const auto & nm : names)
			{
				const auto it = std::find(config.control_joints.begin(), config.control_joints.end(), nm);
				if (it == config.control_joints.end())
				{
					why = "链关节 '" + nm + "' 不在 control_joints 声明集";
					return false;
				}
				c.to_global.push_back(static_cast<size_t>(it - config.control_joints.begin()));
			}
			c.prev_dq.assign(c.n, 0.0);
			c.warmed = false;
		}
		n_ = config.control_joints.size();
		dt_s_ = config.dt_s;
		end_frames_ = config.end_frames;
		ready_ = true;
		return true;
	}

	[[nodiscard]] unistackbot_algorithm::WholeBodyResult solve(const unistackbot_algorithm::WholeBodyInput & in,
		unistackbot_algorithm::WholeBodyOutput & out) const override
	{
		using unistackbot_algorithm::WholeBodyResult;
		if (!ready_ || in.q.size() != n_ || in.dq.size() != n_ || in.goals.size() != 2)
		{
			return WholeBodyResult::NOT_READY;
		}
		const auto goal_of = [&](const std::string & frame) -> const unistackbot_algorithm::CartesianGoal * {
			for (const auto & g : in.goals)
			{
				if (g.frame == frame)
				{
					return &g;
				}
			}
			return nullptr;
		};
		// 先算后写: 每链结果进暂存, 两链全 OK 才落笔 out (契约 1)
		ChainResult cr[2];
		for (int side = 0; side < 2; ++side)
		{
			const unistackbot_algorithm::CartesianGoal * g = goal_of(end_frames_[side]);
			if (g == nullptr)
			{
				return WholeBodyResult::NOT_READY;   // 必需帧缺失 = 调用方契约违例
			}
			const WholeBodyResult r = solveChain(side, *g, in, cr[side]);
			if (r != WholeBodyResult::OK)
			{
				return r;
			}
		}
		out.q.assign(n_, 0.0);
		out.dq.assign(n_, 0.0);
		out.ddq.assign(n_, 0.0);
		out.gravity.clear();   // 诚实能力: 运动学栈无动力学源
		out.frames.clear();
		out.tasks.clear();
		for (int side = 0; side < 2; ++side)
		{
			const Chain & c = chains_[side];   // const solve + mutable 暖状态 (IkSolver 契约 4 同款)
			const ChainResult & r = cr[side];
			for (size_t i = 0; i < c.n; ++i)
			{
				const size_t gi = c.to_global[i];
				out.q[gi] = r.q_next[i];
				out.dq[gi] = r.dq[i];
				out.ddq[gi] = r.ddq[i];
			}
			c.prev_dq = r.dq;   // 温启动状态仅在全成功后推进
			c.warmed = true;
			out.frames.push_back(unistackbot_algorithm::FramePose{c.end_frame, r.curr});
			out.tasks.push_back(r.diag);
		}
		return WholeBodyResult::OK;
	}

private:
	// ---- 参数 (定稿时入 vendor 参数面) ----
	static constexpr double kGainPos = 2.0;
	static constexpr double kGainRot = 2.0;
	static constexpr double kDampLambda = 0.05;
	static constexpr double kMaxDq = 1.5;
	static constexpr double kConvTol = 1e-3;

	struct Chain
	{
		unistackbot_algorithm::UrdfFk fk;
		std::string end_frame;
		std::vector<size_t> to_global;   // 链内 idx → control_joints 全序 idx
		mutable std::vector<double> prev_dq;
		mutable unistackbot_algorithm::UrdfFk::Scratch scratch;
		mutable std::vector<double> jac_buf;
		size_t n{0};
		mutable bool warmed{false};
	};
	struct ChainResult
	{
		std::vector<double> q_next, dq, ddq;
		unistackbot_algorithm::CartesianPose curr;
		unistackbot_algorithm::TaskDiag diag;
	};

	[[nodiscard]] unistackbot_algorithm::WholeBodyResult solveChain(int side,
		const unistackbot_algorithm::CartesianGoal & goal, const unistackbot_algorithm::WholeBodyInput & in,
		ChainResult & cr) const
	{
		using unistackbot_algorithm::WholeBodyResult;
		const Chain & c = chains_[side];
		std::vector<double> q(c.n), dq(c.n);
		for (size_t i = 0; i < c.n; ++i)
		{
			q[i] = in.q[c.to_global[i]];
			dq[i] = in.dq[c.to_global[i]];
		}
		(void)dq;   // DLS 主式不消费测量速度 (与 single_arm_dls 示例同口径)
		const auto & target = goal.target;
		if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z))
		{
			return WholeBodyResult::UNREACHABLE;
		}
		unistackbot_algorithm::CartesianPose curr;
		if (!c.fk.fk(q, curr, c.scratch))
		{
			return WholeBodyResult::NOT_READY;
		}
		const double dist = std::sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
		if (dist > c.fk.maxReach())
		{
			return WholeBodyResult::UNREACHABLE;
		}
		// 误差 6 维 (位置 + 短弧姿态)
		Eigen::Vector3d e_pos(target.x - curr.x, target.y - curr.y, target.z - curr.z);
		const Eigen::Quaterniond q_curr(curr.qw, curr.qx, curr.qy, curr.qz);
		const Eigen::Quaterniond q_tgt(target.qw, target.qx, target.qy, target.qz);
		Eigen::Quaterniond qe = q_tgt * q_curr.conjugate();
		if (qe.w() < 0.0)
		{
			qe.coeffs() *= -1.0;
		}
		const double angle = 2.0 * std::acos(std::clamp(qe.w(), -1.0, 1.0));
		const double sin_half = std::sqrt(std::max(0.0, 1.0 - qe.w() * qe.w()));
		Eigen::Vector3d e_rot = Eigen::Vector3d::Zero();
		if (sin_half >= 1e-9)
		{
			e_rot = qe.vec().normalized() * angle;
		}
		Eigen::Matrix<double, 6, 1> err;
		err << kGainPos * e_pos, kGainRot * e_rot;
		if (!c.fk.jacobian(q, c.jac_buf, c.scratch))
		{
			return WholeBodyResult::NOT_READY;
		}
		const Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>> J(c.jac_buf.data(), 6,
			static_cast<int>(c.n));
		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += kDampLambda * kDampLambda;
		const Eigen::VectorXd dqv = J.transpose() * JJt.ldlt().solve(err);
		if (!dqv.allFinite())
		{
			return WholeBodyResult::SOLVE_FAILED;
		}
		int saturated = 0;
		cr.q_next.assign(c.n, 0.0);
		cr.dq.assign(c.n, 0.0);
		cr.ddq.assign(c.n, 0.0);
		for (size_t i = 0; i < c.n; ++i)
		{
			double v = std::clamp(dqv[static_cast<Eigen::Index>(i)], -kMaxDq, kMaxDq);
			if (std::abs(v) >= kMaxDq)
			{
				++saturated;
			}
			cr.dq[i] = v;
			cr.q_next[i] = std::clamp(q[i] + v * dt_s_, c.fk.qMin()[i], c.fk.qMax()[i]);
		}
		if (saturated == static_cast<int>(c.n) && e_pos.norm() > kConvTol)
		{
			return WholeBodyResult::LIMIT_CONFLICT;
		}
		for (size_t i = 0; i < c.n; ++i)
		{
			cr.ddq[i] = c.warmed ? (cr.dq[i] - c.prev_dq[i]) / dt_s_ : 0.0;
		}
		cr.curr = curr;
		cr.diag.task = c.end_frame;
		cr.diag.error = e_pos.norm();
		cr.diag.state = cr.diag.error < kConvTol ? unistackbot_algorithm::TaskState::CONVERGED
			: unistackbot_algorithm::TaskState::APPROACHING;
		return WholeBodyResult::OK;
	}

	Chain chains_[2];
	std::vector<std::string> end_frames_;
	double dt_s_{0.0};
	bool ready_{false};
	size_t n_{0};
};

#endif  // UNISTACKBOT_ALGORITHM__DUALARM_DLS_HPP_

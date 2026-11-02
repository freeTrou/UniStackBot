#ifndef UNISTACKBOT_ALGORITHM__SINGLE_ARM_DLS_HPP_
#define UNISTACKBOT_ALGORITHM__SINGLE_ARM_DLS_HPP_

/*
 * SingleArmServoDls —— WholeBodySolver 契约的第二个参考实现 (2026-10-29, 示例身份):
 * 单臂速度级阻尼最小二乘 (DLS) 伺服, 数学栈 = urdf_fk (KDL) + Eigen——
 * 与 upper_body_hqp_mos (pinocchio+qpOASES 双臂 HQP) 并列, 演示"同契约第二种算法":
 * 拓扑 (单臂/双臂) 与流派 (DLS/HQP) 都住实现层, 契约不变 (增实现不增接口)。
 *
 * 依赖闭包: 纯仓内栈 (urdf_fk + Eigen), 零 pinocchio/qpOASES/yaml-cpp。
 *
 * 与 mos 示例的镜像对照:
 *   - 形态选拒相反: 本实现 end_frames 须恰为 1;
 *   - 关节序校验**天然存在** (urdf_fk 有 jointNames() 接口——mos 缺口②在本栈不存在);
 *   - 失败分类天然完整: UNREACHABLE (maxReach 几何预检)/LIMIT_CONFLICT (全饱和+大误差)
 *     /SOLVE_FAILED (非有限解) 都能真实产生, 不需预留。
 *
 * 诚实能力: out.gravity 留空——运动学栈无动力学源, 永不伪造 (壳按空=无前馈)。
 *
 * 本示例暴露并已收口的契约项 (v0.2, 2026-10-29 用户批):
 *   WholeBodyConfig.base_frame (目标参考系显性化——本实现直接用它建链) +
 *   dt_s (worker 节拍约定值——本实现直接用它积分); 原 constexpr 占位已删除。
 *
 * 规范检查命令 (g++ 直编, 零 ROS):
 *   g++ -std=c++17 -Wall -Wextra -fsyntax-only -I<本包根> -I/usr/include/eigen3 \
 *       whole_body/single_arm_dls.hpp
 */
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "urdf_fk/urdf_fk.hpp"
#include "whole_body.hpp"

class SingleArmServoDls : public unistackbot_algorithm::WholeBodySolver
{
public:
	[[nodiscard]] bool init(const unistackbot_algorithm::WholeBodyConfig & config, std::string & why) override
	{
		// 形态选拒 (契约 3): 本实现 = 单臂, 与 mos 双臂示例互为镜像
		if (config.end_frames.size() != 1)
		{
			why = "single_arm_dls 为单臂实现, end_frames 须恰为 1 (当前 " + std::to_string(config.end_frames.size()) + ")";
			return false;
		}
		// v0.2 字段防呆 (无默认纪律的 impl 侧守门): base_frame 空 / dt_s 非法即拒
		if (config.base_frame.empty() || !(config.dt_s > 0.0))
		{
			why = "base_frame 与 dt_s 为必填装配参数 (base_frame 非空, dt_s > 0)";
			return false;
		}
		if (!fk_.init(config.urdf, config.base_frame, config.end_frames[0], why))
		{
			return false;
		}
		dt_s_ = config.dt_s;
		// 关节序严格校验 (mos 缺口②在本栈不存在: urdf_fk 有链序名接口)
		if (fk_.jointNames() != config.control_joints || fk_.jointCount() != config.control_joints.size())
		{
			why = "control_joints 与 URDF 链序不一致 (须逐位相等)";
			return false;
		}
		n_ = config.control_joints.size();
		end_frame_ = config.end_frames[0];
		// 遥测帧选拒 (契约 3 同族): 本实现只能提供末端帧位姿——遥测表出现其他帧名即拒,
		// 不静默缺供 (2026-10-29 合理性审核修: 原先留给 solve 跳过 = 消费方预期落空)
		for (const auto & name : config.telemetry_frames)
		{
			if (name != end_frame_)
			{
				why = "single_arm_dls 遥测帧只支持末端帧 '" + end_frame_ + "' (声明了 '" + name + "')";
				return false;
			}
		}
		telemetry_ = config.telemetry_frames;   // 已验证 ⊆ {末端帧}; 空表 = out.frames 也为空 (语义保真)
		prev_dq_.assign(n_, 0.0);
		ready_ = true;
		return true;
	}

	[[nodiscard]] unistackbot_algorithm::WholeBodyResult solve(const unistackbot_algorithm::WholeBodyInput & in,
		unistackbot_algorithm::WholeBodyOutput & out) const override
	{
		if (!ready_ || in.q.size() != n_ || in.dq.size() != n_ || in.goals.size() != 1)
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		// goal 帧名校验 (按名消费的第一道门; 帧名错 = 调用方契约违例 → NOT_READY)
		if (in.goals[0].frame != end_frame_)
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		const auto & target = in.goals[0].target;
		if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z))
		{
			return unistackbot_algorithm::WholeBodyResult::UNREACHABLE;   // 契约: 目标非有限归此码
		}
		// ① 当前 FK (Scratch 零分配暂存, 实例线程私有)
		unistackbot_algorithm::CartesianPose curr;
		if (!fk_.fk(in.q, curr, scratch_))
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		// ② 几何预检 (粗可达上界, urdf_fk 现成): 目标出界 → UNREACHABLE (诚实拒绝)
		const double target_dist = std::sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
		if (target_dist > fk_.maxReach())
		{
			return unistackbot_algorithm::WholeBodyResult::UNREACHABLE;
		}
		// ③ 误差 6 维: 位置 + 短弧姿态 (q_err = q_tgt ⊗ q_curr⁻¹ → 轴角)
		Eigen::Vector3d e_pos(target.x - curr.x, target.y - curr.y, target.z - curr.z);
		const Eigen::Quaterniond q_curr(curr.qw, curr.qx, curr.qy, curr.qz);
		const Eigen::Quaterniond q_tgt(target.qw, target.qx, target.qy, target.qz);
		Eigen::Quaterniond qe = q_tgt * q_curr.conjugate();
		if (qe.w() < 0.0)
		{
			qe.coeffs() *= -1.0;   // 短弧
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
		// ④ 雅可比 + DLS: dq = Jᵀ(JJᵀ + λ²I)⁻¹ err (LDLT——JJt 对称正定)
		if (!fk_.jacobian(in.q, jac_buf_, scratch_))
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		const Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>> J(jac_buf_.data(), 6,
			static_cast<int>(n_));
		Eigen::MatrixXd JJt = J * J.transpose();
		JJt.diagonal().array() += kDampLambda * kDampLambda;
		const Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);
		if (!dq.allFinite())
		{
			return unistackbot_algorithm::WholeBodyResult::SOLVE_FAILED;
		}
		// ⑤ 速度界 + 限位内积分 (从测量出发积分 = 闭环天生成立)
		int saturated = 0;
		std::vector<double> q_next(n_), dq_clamped(n_);
		for (size_t i = 0; i < n_; ++i)
		{
			double v = std::clamp(dq[static_cast<Eigen::Index>(i)], -kMaxDq, kMaxDq);
			if (std::abs(v) >= kMaxDq)
			{
				++saturated;
			}
			dq_clamped[i] = v;
			q_next[i] = std::clamp(in.q[i] + v * dt_s_, fk_.qMin()[i], fk_.qMax()[i]);
		}
		// 全关节速度饱和且位置误差仍大 = 走不动了 → LIMIT_CONFLICT (诚实拒绝)
		if (saturated == static_cast<int>(n_) && e_pos.norm() > kConvTol)
		{
			return unistackbot_algorithm::WholeBodyResult::LIMIT_CONFLICT;
		}
		// —— 以下才开始写 out (可失败步骤全部在上) ——
		out.q = q_next;
		out.dq = dq_clamped;
		out.ddq.resize(n_);
		if (warmed_)   // 首拍 prev_dq_ 为零初始化, 直接差分 = ddq 假尖峰; 首拍置零 (合理性审核修)
		{
			for (size_t i = 0; i < n_; ++i)
			{
				out.ddq[i] = (dq_clamped[i] - prev_dq_[i]) / dt_s_;
			}
		}
		else
		{
			std::fill(out.ddq.begin(), out.ddq.end(), 0.0);
			warmed_ = true;
		}
		prev_dq_ = dq_clamped;   // 温启动状态 (实例线程私有, 契约 2)
		out.gravity.clear();     // 诚实能力: 运动学栈无动力学源, 永不伪造 (壳按空=无前馈)
		// 遥测帧: end_frames[0] 在帧表内则回当前 FK (其余帧名本实现无来源, 跳过)
		out.frames.clear();
		if (std::find(telemetry_.begin(), telemetry_.end(), in.goals[0].frame) != telemetry_.end())
		{
			out.frames.push_back(unistackbot_algorithm::FramePose{in.goals[0].frame, curr});
		}
		// 任务诊断: 位置误差范数 (姿态可加权并入, 定稿时入 vendor 参数)
		unistackbot_algorithm::TaskDiag d;
		d.task = in.goals[0].frame;
		d.error = e_pos.norm();
		d.state = d.error < kConvTol ? unistackbot_algorithm::TaskState::CONVERGED
			: unistackbot_algorithm::TaskState::APPROACHING;
		out.tasks.clear();
		out.tasks.push_back(d);
		return unistackbot_algorithm::WholeBodyResult::OK;
	}

private:
	// —— 实现参数 (定稿时入 vendor 参数面) ——
	static constexpr double kGainPos = 2.0;      // 位置误差 → 速度增益 [1/s]
	static constexpr double kGainRot = 2.0;      // 姿态误差 → 角速度增益 [1/s]
	static constexpr double kDampLambda = 0.05;  // DLS 阻尼 (奇异附近牺牲精度换有界解)
	static constexpr double kMaxDq = 1.5;        // 速度界 [rad/s]
	static constexpr double kConvTol = 1e-3;     // 收敛阈 [m]

	unistackbot_algorithm::UrdfFk fk_;
	double dt_s_{0.0};                                         // worker 节拍 (config.dt_s, 积分/差分共用)
	std::string end_frame_;                                    // goal 帧名校验依据
	mutable bool warmed_{false};                               // 首拍 ddq 置零守门
	mutable unistackbot_algorithm::UrdfFk::Scratch scratch_;   // const solve + 零分配暂存 (实例线程私有)
	mutable std::vector<double> prev_dq_;                      // 温启动 (ddq 差分)
	mutable std::vector<double> jac_buf_;                      // 雅可比行主序缓冲
	std::vector<std::string> telemetry_;
	bool ready_{false};
	size_t n_{0};
};

#endif  // UNISTACKBOT_ALGORITHM__SINGLE_ARM_DLS_HPP_

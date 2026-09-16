#include "unistackbot_controller/dls_ik.hpp"

#include <cmath>
#include <random>

namespace unistackbot_controller
{

namespace
{

constexpr double kEps = 1e-12;

}  // namespace

bool DlsIk::init(const UrdfFk * fk, std::string & message)
{
	if (fk == nullptr || !fk->ready())
	{
		message = "DlsIk: UrdfFk 未就绪";
		return false;
	}
	fk_ = fk;
	message = "DlsIk 就绪 (" + std::to_string(fk_->jointCount()) + " 关节)";
	return true;
}

Eigen::Matrix<double, 6, 1> DlsIk::poseError(const CartesianPose & a, const CartesianPose & b)
{
	Eigen::Matrix<double, 6, 1> e;
	e << (b.x - a.x), (b.y - a.y), (b.z - a.z), 0, 0, 0;

	// 姿态误差 = log(R_aᵀ·R_b) —— 在 a 的局部系表达, 必须转回基座系再与
	// KDL 雅可比 (基座系几何雅可比) 相乘; 混系 = 姿态项方向偏差, 迭代发散
	// (本函数两代踩坑: v1 四元数手推符号错, v2 忘转系——都表现为不收敛, 2026-09-17)
	const Eigen::Quaterniond qa(a.qw, a.qx, a.qy, a.qz);
	const Eigen::Quaterniond qb(b.qw, b.qx, b.qy, b.qz);
	const Eigen::Matrix3d Ra = qa.toRotationMatrix();
	const Eigen::Matrix3d dR = Ra.transpose() * qb.toRotationMatrix();
	const double tr = dR.trace();
	const double th = std::acos(std::clamp((tr - 1.0) / 2.0, -1.0, 1.0));
	if (th > 1e-9)
	{
		Eigen::Matrix<double, 3, 1> w;   // 局部系轴角
		w << (dR(2, 1) - dR(1, 2)), (dR(0, 2) - dR(2, 0)), (dR(1, 0) - dR(0, 1));
		e.tail<3>() = Ra * (w * (th / (2.0 * std::sin(th))));   // -> 基座系
	}
	return e;
}

void DlsIk::nullspaceGradient(
	const std::vector<double> & q, const RedundancyPreference & /*red*/,
	std::vector<double> & grad) const
{
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	const std::size_t n = q.size();
	grad.assign(n, 0.0);

	// H₁ = 限位中心距: ∇H₁ᵢ = -(qᵢ - centerᵢ) / rangeᵢ² (推向中心, 距离越远推力越大)
	for (std::size_t i = 0; i < n; ++i)
	{
		const double c = (lo[i] + hi[i]) / 2.0;
		const double range = std::max(hi[i] - lo[i], 1e-3);
		grad[i] -= cfg_.w_center * (q[i] - c) / (range * range);
	}

	// H₀ = 边界排斥 (标准饱和式避限位): 距边界 < margin 时施加线性推回力。
	// 没有它 DLS 会大量收敛在界外 (2026-09-17 实测 61% 解出界); 每步硬 clamp
	// 则边界踏步更差。斥力进零空间投影, 不污染主任务方向。
	{
		const double margin_ratio = 0.1;   // 边界带宽度 = 10% 行程
		const double k_push = 2.0;         // 斥力强度 (远强于中心吸引)
		for (std::size_t i = 0; i < n; ++i)
		{
			const double lo_m = lo[i] + margin_ratio * (hi[i] - lo[i]);
			const double hi_m = hi[i] - margin_ratio * (hi[i] - lo[i]);
			if (q[i] < lo_m)
			{
				grad[i] += k_push * (lo_m - q[i]) / std::max(hi[i] - lo[i], 1e-3);
			}
			else if (q[i] > hi_m)
			{
				grad[i] += k_push * (q[i] - hi_m) / std::max(hi[i] - lo[i], 1e-3);
			}
		}
	}

	// H₂ = 可操作度 w=√det(JJᵀ): 数值微分 (雅可比已有, 解析梯度留优化)
	{
		const double h = 1e-4;
		std::vector<double> Jp, Jm;
		for (std::size_t i = 0; i < n; ++i)
		{
			std::vector<double> qp = q, qm = q;
			qp[i] += h;
			qm[i] -= h;
			Eigen::MatrixXd Jp_e = Eigen::MatrixXd::Zero(6, n), Jm_e = Eigen::MatrixXd::Zero(6, n);
			if (fk_->jacobian(qp, Jp) && fk_->jacobian(qm, Jm))
			{
				for (int r = 0; r < 6; ++r)
				{
					for (std::size_t c = 0; c < n; ++c)
					{
						Jp_e(r, static_cast<Eigen::Index>(c)) = Jp[r * n + c];
						Jm_e(r, static_cast<Eigen::Index>(c)) = Jm[r * n + c];
					}
				}
				const double wp = std::sqrt(std::max((Jp_e * Jp_e.transpose()).determinant(), 0.0));
				const double wm = std::sqrt(std::max((Jm_e * Jm_e.transpose()).determinant(), 0.0));
				grad[i] += cfg_.w_manip * (wp - wm) / (2.0 * h);
			}
		}
	}
}

IkResult DlsIk::solveOnce(
	const CartesianPose & target, const std::vector<double> & seed,
	const RedundancyPreference & red, std::vector<double> & out_q) const
{
	const std::size_t n = fk_->jointCount();
	if (!ready() || seed.size() != n)
	{
		return IkResult::ITERATION_LIMIT;   // 未就绪/维度错: 无解可给 (对外统一为求解失败)
	}

	// 限位 (迭代内 clamp 用)
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	// LOCK_JOINT: 锁定值取自种子表达 (锁值越限在调用入口已查)
	std::vector<double> q = seed;
	const int locked = (red.type == RedundancyType::LOCK_JOINT &&
		red.joint_index < n) ? static_cast<int>(red.joint_index) : -1;

	std::vector<double> grad, jac_row(6 * n, 0.0);
	Eigen::MatrixXd J(6, static_cast<Eigen::Index>(n));
	double best_err = 1e300;
	int stall = 0;

	for (int iter = 0; iter < cfg_.max_iterations; ++iter)
	{
		CartesianPose cur;
		if (!fk_->fk(q, cur))
		{
			return IkResult::ITERATION_LIMIT;
		}
		const Eigen::Matrix<double, 6, 1> e = poseError(cur, target);
		const double err = e.norm();
		if (e.head<3>().norm() < cfg_.pos_tolerance && e.tail<3>().norm() < cfg_.rot_tolerance)
		{
			// 收敛后限位检查 (不 clamp!): clamp 会把界外解伪装成界内 (FK 回代即
			// 偏移, 2026-09-17 实测踩中) —— 越界 = 该种子失败, 交给阶梯重启找界内解
			bool in_lim = true;
			for (std::size_t i = 0; i < n; ++i)
			{
				if (q[i] < lo[i] || q[i] > hi[i]) {in_lim = false; break;}
			}
			if (!in_lim)
			{
				return IkResult::LIMIT_CONFLICT;
			}
			out_q = q;   // 收敛且限内
			return IkResult::OK;
		}
		// 停滞检测: 误差 25 步不降 5% -> 该种子已陷局部极小, 提前放弃 (省时)
		if (err < best_err * 0.95)
		{
			best_err = err;
			stall = 0;
		}
		else if (++stall >= 15)
		{
			return IkResult::ITERATION_LIMIT;
		}

		if (!fk_->jacobian(q, jac_row))
		{
			return IkResult::ITERATION_LIMIT;
		}
		for (int r = 0; r < 6; ++r)
		{
			for (std::size_t c = 0; c < n; ++c)
			{
				J(r, static_cast<Eigen::Index>(c)) = jac_row[r * n + c];
			}
		}

		// SVD 阻尼伪逆 (数值层=库): J⁺ = V Σ⁺ Uᵀ, σ⁺ = σ/(σ²+λ²)
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
		const auto & sv = svd.singularValues();
		const double smin = sv(sv.size() - 1);
		// λ 自适应: 远离奇异取基值, 近奇异 (smin 小) 加大阻尼
		const double lam = std::clamp(cfg_.lambda_base * (1.0 + 0.1 / std::max(smin, 1e-3)),
			cfg_.lambda_base, cfg_.lambda_max);
		Eigen::VectorXd sv_inv(sv.size());
		for (Eigen::Index i = 0; i < sv.size(); ++i)
		{
			sv_inv(i) = sv(i) / (sv(i) * sv(i) + lam * lam);
		}
		const Eigen::VectorXd dq_main = svd.matrixV() * sv_inv.cwiseProduct(svd.matrixU().transpose() * e);

		// 零空间: N = I − J⁺J (同 SVD), q̇ₙ = N·k∇H
		// 收敛近邻关闭零空间项: ∇H 在解处非零, 持续施加会造成极限环漂移,
		// 永不满足收敛阈 (本仓 2026-09-17 实测踩中; 误差大时才做姿态优化)
		const double err_norm = e.norm();
		Eigen::VectorXd dq_null = Eigen::VectorXd::Zero(n);
		if (err_norm > 1e-3)
		{
			nullspaceGradient(q, red, grad);
			Eigen::VectorXd grad_e = Eigen::VectorXd::Zero(n);
			for (std::size_t i = 0; i < n; ++i)
			{
				grad_e(static_cast<Eigen::Index>(i)) = grad[i];
			}
			const Eigen::MatrixXd Jpinv = svd.matrixV() * sv_inv.asDiagonal() * svd.matrixU().transpose();
			const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - Jpinv * J;
			dq_null = N * (cfg_.nullspace_gain * grad_e);
		}

		// LOCK_JOINT: 锁定关节的更新量清零 (离散投影, 精确锁定)
		Eigen::VectorXd dq = dq_main + dq_null;
		if (locked >= 0)
		{
			dq(locked) = 0.0;
		}

		for (std::size_t i = 0; i < n; ++i)
		{
			q[i] += dq(static_cast<Eigen::Index>(i));
		}

		// isfinite 门: 数值爆炸即停 (安全语义: 绝不输出 NaN/inf)
		for (std::size_t i = 0; i < n; ++i)
		{
			if (!std::isfinite(q[i]))
			{
				return IkResult::ITERATION_LIMIT;
			}
		}
	}
	return IkResult::ITERATION_LIMIT;
}

IkResult DlsIk::solve(
	const CartesianPose & target,
	const std::vector<double> & seed,
	const RedundancyPreference & red,
	std::vector<double> & out_q) const
{
	if (!ready())
	{
		return IkResult::NOT_READY;
	}
	const std::size_t n = fk_->jointCount();
	if (seed.size() != n)
	{
		return IkResult::NOT_READY;
	}
	// ARM_ANGLE: 偏置构型上只是近似语义, 诚实拒绝 (决策卡裁决)
	if (red.type == RedundancyType::ARM_ANGLE)
	{
		return IkResult::UNSUPPORTED;
	}
	// LOCK_JOINT: 锁定值越限直接拒 (锁定值语义上由调用方随命令给出)
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	if (red.type == RedundancyType::LOCK_JOINT)
	{
		if (red.joint_index >= n)
		{
			return IkResult::LIMIT_CONFLICT;
		}
		const double locked_val = seed[red.joint_index];
		if (locked_val < lo[red.joint_index] - 1e-9 || locked_val > hi[red.joint_index] + 1e-9)
		{
			return IkResult::LIMIT_CONFLICT;
		}
	}

	// 几何预检: 目标距离超链长上界 -> UNREACHABLE (诚实失败, 不烧迭代;
	// 粗上界只拦"显然不可达", 姿态级不可达仍走迭代 -> ITERATION_LIMIT)
	{
		const double dist = std::sqrt(target.x * target.x + target.y * target.y +
			target.z * target.z);
		if (dist > fk_->maxReach())
		{
			return IkResult::UNREACHABLE;
		}
	}

	// ---- 种子阶梯 (决策卡 §5.1-1) ----
	// 级 0: 调用方种子 (流式连续性根)
	std::vector<double> sol;
	IkResult r = solveOnce(target, seed, red, sol);
	if (r == IkResult::OK)
	{
		out_q = sol;
		return IkResult::OK;
	}

	// 级 1: 限位感知启发式种子 × N 随机重启 (TRAC-IK 配方)
	// 分支粘性: 重启解距原种子超阈值 = 跳分支, 不输出 (报 ITERATION_LIMIT)
	auto heuristic_seed = [&](std::size_t attempt)
	{
		std::vector<double> s(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			const double c = (lo[i] + hi[i]) / 2.0;
			const double half = (hi[i] - lo[i]) / 2.0;
			if (attempt == 0)
			{
				s[i] = c;   // 首个启发式 = 全限位中心
			}
			else
			{
				// mt19937 均匀撒点覆盖全限位区间 (sin-hash 高度相关, 20 个种子
				// 实际挤在一处 —— 局部极小逃逸失败的主因, 2026-09-17 实测)
				std::mt19937 gen(static_cast<unsigned>(attempt * 977 + i * 31));
				std::uniform_real_distribution<double> dist(lo[i], hi[i]);
				s[i] = dist(gen);
			}
		}
		return s;
	};
	double best_dist = 1e9;
	std::vector<double> best_sol;
	for (int k = 0; k < cfg_.restart_count; ++k)
	{
		const auto s = heuristic_seed(static_cast<std::size_t>(k));
		r = solveOnce(target, s, red, sol);
		if (r == IkResult::OK)
		{
			double dist = 0.0;
			for (std::size_t i = 0; i < n; ++i)
			{
				dist += std::fabs(sol[i] - seed[i]);
			}
			if (dist <= cfg_.jump_threshold)
			{
				out_q = sol;
				return IkResult::OK;
			}
			if (dist < best_dist)
			{
				best_dist = dist;   // 记住最近的, 若全部超阈值则报跳变
				best_sol = sol;
			}
		}
	}
	// 全部重启解都超粘性阈值: 上游拿到的将是"分支跳变"的解, 拒绝输出
	(void)best_sol;
	return IkResult::ITERATION_LIMIT;
}

}  // namespace unistackbot_controller

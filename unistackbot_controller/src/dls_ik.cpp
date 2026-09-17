#include "unistackbot_controller/dls_ik.hpp"

#include "ulog/ulog.hpp"

#include <chrono>
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
	const RedundancyPreference & red, std::vector<double> & out_q,
	int * iterations_used) const
{
	const std::size_t n = fk_->jointCount();
	if (!ready() || seed.size() != n)
	{
		return IkResult::ITERATION_LIMIT;   // 未就绪/维度错: 无解可给 (对外统一为求解失败)
	}

	// 限位 (收敛检查用)
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	// LOCK_JOINT: 锁定值取自种子表达 (锁值越限在调用入口已查)
	std::vector<double> q = seed;
	const int locked = (red.type == RedundancyType::LOCK_JOINT &&
		red.joint_index < n) ? static_cast<int>(red.joint_index) : -1;

	std::vector<double> grad, jac_row(6 * n, 0.0);
	Eigen::MatrixXd J(6, static_cast<Eigen::Index>(n));
	int ls_fail_streak = 0;

	// 加权误差向量: e = [e_p; w_rot·e_r] —— 米与弧度量纲平衡 (w_rot=1 时姿态
	// 主导步长, 位置收敛慢且易震荡, 2026-09-17 实测; 0.5 拖动场景显著更稳)
	auto weightedError = [&](const CartesianPose & cur, Eigen::Matrix<double, 6, 1> & e_out,
		double & pos_err, double & rot_err)
	{
		e_out = poseError(cur, target);
		pos_err = e_out.head<3>().norm();
		rot_err = e_out.tail<3>().norm();
		e_out.tail<3>() *= cfg_.w_rot;
		return e_out.norm();
	};

	for (int iter = 0; iter < cfg_.max_iterations; ++iter)
	{
		CartesianPose cur;
		if (!fk_->fk(q, cur))
		{
			return IkResult::ITERATION_LIMIT;
		}
		Eigen::Matrix<double, 6, 1> e;
		double pos_err = 0.0, rot_err = 0.0;
		const double err = weightedError(cur, e, pos_err, rot_err);
		if (pos_err < cfg_.pos_tolerance && rot_err < cfg_.rot_tolerance)
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
			if (iterations_used) {*iterations_used = iter + 1;}
			return IkResult::OK;
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
		// λ 自适应 (Wampler 式, 2026-09-17 换标准公式): σ_min < 阈值时
		// λ² = λ₀²(1−σ/σ_thr)², 否则 λ=λ₀ —— 阻尼随奇异接近度平滑增长
		const double sigma_thr = 0.1;
		const double ratio = std::min(smin / sigma_thr, 1.0);
		const double lam = std::clamp(cfg_.lambda_base * (1.0 + 3.0 * (1.0 - ratio)),
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
		Eigen::VectorXd dq_null = Eigen::VectorXd::Zero(n);
		if (err > 1e-3)
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

		// 线搜索 (backtracking + 下降容差): 折半找下降点; "下降"含 0.1% 容差
		// (严格 < 在收敛边缘微增即杀种子, 实测成功率崩到 9%, 2026-09-17)。
		// 连续 3 轮全败才放弃 (单轮失败可能是零空间扰动, 不判死)。
		bool accepted = false;
		std::vector<double> q_prev_iter = q;
		// 收敛邻域 (err < 10×容差): 直接信任 DLS 步——此邻域内步长极小、误差
		// 数值持平, 线搜索的严格下降判定会连败杀种子 (实测中心种子 OK 率 53%→9%,
		// 2026-09-17)。经典 backtracking 只在"大步"时保护。
		if (err < 10.0 * (cfg_.pos_tolerance + cfg_.w_rot * cfg_.rot_tolerance))
		{
			for (std::size_t i = 0; i < n; ++i)
			{
				q[i] = std::clamp(q_prev_iter[i] + dq(static_cast<Eigen::Index>(i)), lo[i], hi[i]);
				if (!std::isfinite(q[i]))
				{
					return IkResult::ITERATION_LIMIT;
				}
			}
			accepted = true;
		}
		for (int bt = 0; !accepted && bt <= cfg_.max_backtrack; ++bt)
		{
			const double step = std::pow(0.5, bt);
			std::vector<double> q_try = q_prev_iter;
			bool finite = true;
			for (std::size_t i = 0; i < n; ++i)
			{
				q_try[i] = std::clamp(q_prev_iter[i] + step * dq(static_cast<Eigen::Index>(i)), lo[i], hi[i]);
				if (!std::isfinite(q_try[i]))
				{
					finite = false;
					break;
				}
			}
			if (!finite)
			{
				return IkResult::ITERATION_LIMIT;   // isfinite 门: 绝不输出 NaN/inf
			}
			CartesianPose cur_try;
			if (fk_->fk(q_try, cur_try))
			{
				Eigen::Matrix<double, 6, 1> e_try;
				double p_tmp = 0.0, r_tmp = 0.0;
				const double err_try = weightedError(cur_try, e_try, p_tmp, r_tmp);
				if (err_try < err * 1.001 + 1e-12)
				{
					q = q_try;
					accepted = true;
					break;
				}
			}
		}
		if (!accepted)
		{
			if (++ls_fail_streak >= 3)
			{
				return IkResult::ITERATION_LIMIT;   // 连续 3 轮无下降: 局部极小, 交阶梯
			}
			continue;   // 本轮原地不动, 下轮重算方向 (零空间目标可能已变)
		}
		ls_fail_streak = 0;
	}
	return IkResult::ITERATION_LIMIT;
}

IkResult DlsIk::solve(
	const CartesianPose & target,
	const std::vector<double> & seed,
	const RedundancyPreference & red,
	std::vector<double> & out_q,
	DlsIkStats * stats,
	SolveMode mode) const
{
	const auto t0 = std::chrono::steady_clock::now();
	auto finish = [&](IkResult r)
	{
		if (stats)
		{
			stats->ok = (r == IkResult::OK);
			stats->solve_us = std::chrono::duration<double, std::micro>(
				std::chrono::steady_clock::now() - t0).count();
			if (!stats->ok) {stats->iterations = 0;}
		}
		return r;
	};
	if (!ready())
	{
		return finish(IkResult::NOT_READY);
	}
	const std::size_t n = fk_->jointCount();
	if (seed.size() != n)
	{
		return finish(IkResult::NOT_READY);
	}
	// ARM_ANGLE: 偏置构型上只是近似语义, 诚实拒绝 (决策卡裁决)
	if (red.type == RedundancyType::ARM_ANGLE)
	{
		return finish(IkResult::UNSUPPORTED);
	}
	// LOCK_JOINT: 锁定值越限直接拒 (锁定值语义上由调用方随命令给出)
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	if (red.type == RedundancyType::LOCK_JOINT)
	{
		if (red.joint_index >= n)
		{
			return finish(IkResult::LIMIT_CONFLICT);
		}
		const double locked_val = seed[red.joint_index];
		if (locked_val < lo[red.joint_index] - 1e-9 || locked_val > hi[red.joint_index] + 1e-9)
		{
			return finish(IkResult::LIMIT_CONFLICT);
		}
	}

	// 几何预检: 目标距离超链长上界 -> UNREACHABLE (诚实失败, 不烧迭代;
	// 粗上界只拦"显然不可达", 姿态级不可达仍走迭代 -> ITERATION_LIMIT)
	{
		const double dist = std::sqrt(target.x * target.x + target.y * target.y +
			target.z * target.z);
		if (dist > fk_->maxReach())
		{
			return finish(IkResult::UNREACHABLE);
		}
	}

	// ---- 种子阶梯 (决策卡 §5.1-1) ----
	// 级 0: 调用方种子 (流式连续性根)
	std::vector<double> sol;
	int iters = 0;
	IkResult r = solveOnce(target, seed, red, sol, &iters);
	if (r == IkResult::OK)
	{
		if (stats) {stats->iterations = iters; stats->restarts_used = 0;}
		out_q = sol;
		return finish(IkResult::OK);
	}

	// 级 1: 限位感知启发式 × 分层随机重启 (TRAC-IK 配方 + 分层改良:
	// 前半中心带 ±30% (典型构型邻域), 后半全区间均匀 (逃局部极小) —— 纯均匀
	// 重启浪费在远离解的种子上, 2026-09-17)
	// 分支粘性: 重启解距原种子超阈值 = 跳分支, 不输出 (报 ITERATION_LIMIT)
	auto heuristic_seed = [&](std::size_t attempt)
	{
		std::vector<double> s(n);
		const bool center_band = (attempt <= static_cast<std::size_t>(cfg_.restart_count / 2));
		for (std::size_t i = 0; i < n; ++i)
		{
			const double c = (lo[i] + hi[i]) / 2.0;
			std::mt19937 gen(static_cast<unsigned>(attempt * 977 + i * 31));
			// 四分支代表 (2662 个 ssik 解的 k-means 中心, 2026-09-17 离线提取;
			// 生产构建: FK 均匀采样聚类, 不依赖 ssik —— "预置种子库"的轻量版)
			static constexpr double kBranch[4][7] = {
				{+0.30, +1.44, -0.03, +1.46, +0.87, -0.03, +1.44},
				{-0.60, +1.44, +0.02, +1.39, -0.79, -0.02, -1.44},
				{+0.15, -1.45, +2.73, +1.39, +0.06, -0.03, -0.07},
				{-0.07, -1.44, -2.73, +1.39, -0.07, -0.03, +0.09}};
			if (attempt == 0)
			{
				s[i] = c;   // 限位中心
			}
			else if (attempt <= 4)
			{
				// 分支代表 + 小扰动 (跳出精确中心的局部性)
				std::uniform_real_distribution<double> dist(kBranch[attempt - 1][i] - 0.25,
					kBranch[attempt - 1][i] + 0.25);
				s[i] = std::clamp(dist(gen), lo[i], hi[i]);
			}
			else if (center_band)
			{
				const double band = 0.3 * (hi[i] - lo[i]) / 2.0;
				std::uniform_real_distribution<double> dist(c - band, c + band);
				s[i] = dist(gen);
			}
			else
			{
				std::uniform_real_distribution<double> dist(lo[i], hi[i]);
				s[i] = dist(gen);
			}
		}
		return s;
	};
	double best_dist = 1e9;
	std::vector<double> best_sol;
	double best_final_err = 0.0;
	for (int k = 0; k < cfg_.restart_count; ++k)
	{
		const auto s = heuristic_seed(static_cast<std::size_t>(k));
		r = solveOnce(target, s, red, sol, &iters);
		if (stats) {stats->restarts_used = k + 1;}
		if (r == IkResult::OK)
		{
			double dist = 0.0;
			for (std::size_t i = 0; i < n; ++i)
			{
				dist += std::fabs(sol[i] - seed[i]);
			}
			if (mode == SolveMode::COLD_START || dist <= cfg_.jump_threshold)
			{
				if (stats) {stats->iterations = iters;}
				out_q = sol;
				return finish(IkResult::OK);
			}
			if (dist < best_dist)
			{
				best_dist = dist;   // 记住最近的, 若全部超阈值则报跳变
				best_sol = sol;
			}
		}
	}
	// 全部重启解都超粘性阈值或全败: 拒绝输出 (调用方保持上一解)
	// 失败时的最终误差 (诊断用): 以最近解的 FK 误差近似
	if (stats && !best_sol.empty())
	{
		CartesianPose back;
		if (fk_->fk(best_sol, back))
		{
			const Eigen::Matrix<double, 6, 1> e = poseError(back, target);
			best_final_err = e.head<3>().norm();
			stats->final_err = best_final_err;
		}
	}
	ULOG_WARN("dls_ik 失败: 目标(%.3f,%.3f,%.3f) 重启%d次 最近解距种子%.2frad 位置残差%.4fm",
		target.x, target.y, target.z, stats ? stats->restarts_used : cfg_.restart_count,
		best_dist < 1e8 ? best_dist : -1.0, best_final_err);
	return finish(IkResult::ITERATION_LIMIT);
}

}  // namespace unistackbot_controller

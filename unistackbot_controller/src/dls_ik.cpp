#include "unistackbot_controller/dls_ik.hpp"

#include "ulog/ulog.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

namespace unistackbot_controller
{

namespace
{

constexpr double kEps = 1e-12;

// 种子库查询参数 (2026-09-17 实测定稿: w 扫 0.2/0.35/0.5, 0.35 最优;
// 分支封顶在 k>=4 开始赢, top-6 +2.5pp)
constexpr double kLibRankRotWeight = 0.35;
constexpr std::size_t kLibTopK = 6;
constexpr std::size_t kLibPool = 12;   // partial_sort 候选池 (封顶挑选的余量)
constexpr int kLibBranchCap = 2;
constexpr int kLibBranchCount = 4;

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

bool DlsIk::loadSeedLibrary(const std::string & path, std::string & message)
{
	seed_lib_.clear();
	lib_scratch_.clear();
	if (!ready())
	{
		message = "DlsIk 未就绪, 先 init";
		return false;
	}
	FILE * f = std::fopen(path.c_str(), "r");
	if (!f)
	{
		message = "种子库打开失败: " + path;
		return false;
	}
	const std::size_t n = fk_->jointCount();
	const auto & lo = fk_->qMin();
	const auto & hi = fk_->qMax();
	char line[512];
	while (std::fgets(line, sizeof(line), f))
	{
		if (line[0] == '#' || line[0] == '\n')
		{
			continue;
		}
		const std::string ln(line);
		const auto bar = ln.find('|');
		if (bar == std::string::npos)
		{
			std::fclose(f);
			message = "种子库行缺 '|': " + ln.substr(0, 40);
			return false;
		}
		SeedEntry e;
		e.q.resize(n);
		double j[16] = {0};
		if (std::sscanf(ln.substr(0, bar).c_str(), "%lf %lf %lf %lf %lf %lf %lf",
			&e.pose.qw, &e.pose.qx, &e.pose.qy, &e.pose.qz,
			&e.pose.x, &e.pose.y, &e.pose.z) != 7)
		{
			std::fclose(f);
			message = "种子库位姿段解析失败: " + ln.substr(0, 40);
			return false;
		}
		if (std::sscanf(ln.substr(bar + 1).c_str(),
			"%lf %lf %lf %lf %lf %lf %lf %d",
			&j[0], &j[1], &j[2], &j[3], &j[4], &j[5], &j[6], &e.branch) !=
			static_cast<int>(n) + 1)
		{
			std::fclose(f);
			message = "种子库关节段解析失败 (列数应 = 关节数 + 分支标签): " + ln.substr(0, 40);
			return false;
		}
		for (std::size_t i = 0; i < n; ++i)
		{
			e.q[i] = j[i];
		}
		// 限位 + 分支标签域检查
		if (e.branch < 0 || e.branch >= kLibBranchCount)
		{
			std::fclose(f);
			message = "种子库分支标签越界: " + std::to_string(e.branch);
			return false;
		}
		for (std::size_t i = 0; i < n; ++i)
		{
			if (e.q[i] < lo[i] - 1e-9 || e.q[i] > hi[i] + 1e-9)
			{
				std::fclose(f);
				message = "种子库关节越限 (第 " + std::to_string(seed_lib_.size()) + " 条)";
				return false;
			}
		}
		// FK 一致性 (关节序错配/文件损坏当场暴露; 装载期一次性成本)
		CartesianPose back;
		if (!fk_->fk(e.q, back) ||
			std::sqrt((back.x - e.pose.x) * (back.x - e.pose.x) +
				(back.y - e.pose.y) * (back.y - e.pose.y) +
				(back.z - e.pose.z) * (back.z - e.pose.z)) > 1e-3 ||
			std::fabs(back.qw * e.pose.qw + back.qx * e.pose.qx +
				back.qy * e.pose.qy + back.qz * e.pose.qz) < 0.99999)
		{
			std::fclose(f);
			message = "种子库 FK 一致性失败 (第 " + std::to_string(seed_lib_.size()) + " 条)";
			return false;
		}
		seed_lib_.push_back(std::move(e));
	}
	std::fclose(f);
	if (seed_lib_.empty())
	{
		message = "种子库为空: " + path;
		return false;
	}
	lib_scratch_.reserve(seed_lib_.size());
	message = "种子库就绪: " + std::to_string(seed_lib_.size()) + " 条 (" + path + ")";
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
	std::vector<double> & grad, UrdfFk::Scratch & fk_scratch) const
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
	// w_manip=0 (默认) 时整块跳过: 2n 次雅可比评估只为乘零丢弃, 是迭代主成本之一
	// (2026-09-17 实测砍掉后单迭代约减半, 预算受限档成功率显著上移)
	if (cfg_.w_manip != 0.0)
	{
		const double h = 1e-4;
		std::vector<double> Jp, Jm;
		for (std::size_t i = 0; i < n; ++i)
		{
			std::vector<double> qp = q, qm = q;
			qp[i] += h;
			qm[i] -= h;
			Eigen::MatrixXd Jp_e = Eigen::MatrixXd::Zero(6, n), Jm_e = Eigen::MatrixXd::Zero(6, n);
			if (fk_->jacobian(qp, Jp, fk_scratch) && fk_->jacobian(qm, Jm, fk_scratch))
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
	const std::chrono::steady_clock::time_point & deadline,
	int iter_cap,
	int * iterations_used,
	double * sigma_out,
	double * err_out) const
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

	// ---- 稳态零分配 (RT 纪律 + 迭代成本): 全部暂存一次定容, 迭代内只复用。
	// SVD 对象循环外构造、compute() 复用其存储 (同尺寸 resize 为空操作) ----
	std::vector<double> grad, jac_row(6 * n, 0.0);
	std::vector<double> q_prev_iter(n), q_try(n);
	Eigen::MatrixXd J(6, static_cast<Eigen::Index>(n));
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(6, static_cast<Eigen::Index>(n),
		Eigen::ComputeThinU | Eigen::ComputeThinV);
	const Eigen::Index m = std::min<Eigen::Index>(6, static_cast<Eigen::Index>(n));
	Eigen::VectorXd sv_inv(m), Ute(m), scaled(m);
	Eigen::VectorXd dq(static_cast<Eigen::Index>(n)), dq_main(static_cast<Eigen::Index>(n)),
		dq_null(static_cast<Eigen::Index>(n)), grad_e(static_cast<Eigen::Index>(n)),
		grad_scaled(static_cast<Eigen::Index>(n));
	Eigen::MatrixXd Jpinv(static_cast<Eigen::Index>(n), 6), N(static_cast<Eigen::Index>(n),
		static_cast<Eigen::Index>(n));
	UrdfFk::Scratch fk_scratch;
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

	// 遥测出口: 最新 σ_min / 加权误差 (调用方据此做奇异遥测与失败分类)
	double last_sigma = -1.0, last_err = -1.0;
	auto finish_once = [&](IkResult r)
	{
		if (sigma_out) {*sigma_out = last_sigma;}
		if (err_out) {*err_out = (last_err >= 0.0) ? last_err : 0.0;}
		return r;
	};

	for (int iter = 0; iter < iter_cap; ++iter)
	{
		// 预算门 (RT 周期安全): steady_clock 读取 ~20ns, 占单迭代 <0.5%;
		// 到时按迭代上限失败处理 —— 失败不改输出, 上层据此停止烧重启
		if (std::chrono::steady_clock::now() >= deadline)
		{
			return finish_once(IkResult::ITERATION_LIMIT);
		}
		CartesianPose cur;
		if (!fk_->fk(q, cur, fk_scratch))
		{
			return finish_once(IkResult::ITERATION_LIMIT);
		}
		Eigen::Matrix<double, 6, 1> e;
		double pos_err = 0.0, rot_err = 0.0;
		const double err = weightedError(cur, e, pos_err, rot_err);
		last_err = err;
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
				return finish_once(IkResult::LIMIT_CONFLICT);
			}
			out_q = q;   // 收敛且限内
			if (iterations_used) {*iterations_used = iter + 1;}
			// σ 取上一次迭代值: 收敛退出发生在 SVD 之前, 末步在信任邻域内步长
			// 极小, 前一迭代构型与解的差异可忽略 (迭代 0 直接收敛则保持 -1)
			return finish_once(IkResult::OK);
		}

		if (!fk_->jacobian(q, jac_row, fk_scratch))
		{
			return finish_once(IkResult::ITERATION_LIMIT);
		}
		for (int r = 0; r < 6; ++r)
		{
			for (std::size_t c = 0; c < n; ++c)
			{
				J(r, static_cast<Eigen::Index>(c)) = jac_row[r * n + c];
			}
		}

		// SVD 阻尼伪逆 (数值层=库): J⁺ = V Σ⁺ Uᵀ, σ⁺ = σ/(σ²+λ²)
		// (svd 循环外构造, compute() 复用存储 —— 与逐迭代构造数值逐位一致)
		svd.compute(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
		const auto & sv = svd.singularValues();
		const double smin = sv(sv.size() - 1);
		last_sigma = smin;   // 奇异遥测: SVD 本来就算, 透传零成本
		// λ 自适应 (Wampler 式, 2026-09-17 换标准公式): σ_min < 阈值时
		// λ² = λ₀²(1−σ/σ_thr)², 否则 λ=λ₀ —— 阻尼随奇异接近度平滑增长
		const double sigma_thr = 0.1;
		const double ratio = std::min(smin / sigma_thr, 1.0);
		const double lam = std::clamp(cfg_.lambda_base * (1.0 + 3.0 * (1.0 - ratio)),
			cfg_.lambda_base, cfg_.lambda_max);
		for (Eigen::Index i = 0; i < sv.size(); ++i)
		{
			sv_inv(i) = sv(i) / (sv(i) * sv(i) + lam * lam);
		}
		// 分步 noalias (表达式求值序与原嵌套式一致: 先 Uᵀe, 再 cwise, 再 V·)
		Ute.noalias() = svd.matrixU().transpose() * e;
		scaled = sv_inv.cwiseProduct(Ute);
		dq_main.noalias() = svd.matrixV() * scaled;

		// 零空间: N = I − J⁺J (同 SVD), q̇ₙ = N·k∇H
		// 收敛近邻关闭零空间项: ∇H 在解处非零, 持续施加会造成极限环漂移,
		// 永不满足收敛阈 (本仓 2026-09-17 实测踩中; 误差大时才做姿态优化)
		dq_null.setZero();
		if (err > 1e-3)
		{
			nullspaceGradient(q, red, grad, fk_scratch);
			for (std::size_t i = 0; i < n; ++i)
			{
				grad_e(static_cast<Eigen::Index>(i)) = grad[i];
			}
			Jpinv.noalias() = svd.matrixV() * sv_inv.asDiagonal() * svd.matrixU().transpose();
			N.setIdentity();
			N.noalias() -= Jpinv * J;
			grad_scaled = cfg_.nullspace_gain * grad_e;   // 先标量后矩阵乘 (保持原求值序)
			dq_null.noalias() = N * grad_scaled;
		}

		// LOCK_JOINT: 锁定关节的更新量清零 (离散投影, 精确锁定)
		dq = dq_main + dq_null;
		if (locked >= 0)
		{
			dq(locked) = 0.0;
		}

		// 线搜索 (backtracking + 下降容差): 折半找下降点; "下降"含 0.1% 容差
		// (严格 < 在收敛边缘微增即杀种子, 实测成功率崩到 9%, 2026-09-17)。
		// 连续 3 轮全败才放弃 (单轮失败可能是零空间扰动, 不判死)。
		bool accepted = false;
		std::copy(q.begin(), q.end(), q_prev_iter.begin());
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
			std::copy(q_prev_iter.begin(), q_prev_iter.end(), q_try.begin());
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
				return finish_once(IkResult::ITERATION_LIMIT);   // isfinite 门: 绝不输出 NaN/inf
			}
			CartesianPose cur_try;
			if (fk_->fk(q_try, cur_try, fk_scratch))
			{
				Eigen::Matrix<double, 6, 1> e_try;
				double p_tmp = 0.0, r_tmp = 0.0;
				const double err_try = weightedError(cur_try, e_try, p_tmp, r_tmp);
				if (err_try < err * 1.001 + 1e-12)
				{
					std::copy(q_try.begin(), q_try.end(), q.begin());
					accepted = true;
					break;
				}
			}
		}
		if (!accepted)
		{
			if (++ls_fail_streak >= 3)
			{
				return finish_once(IkResult::ITERATION_LIMIT);   // 连续 3 轮无下降: 局部极小, 交阶梯
			}
			continue;   // 本轮原地不动, 下轮重算方向 (零空间目标可能已变)
		}
		ls_fail_streak = 0;
	}
	return finish_once(IkResult::ITERATION_LIMIT);
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
	// 墙钟预算: timeout_ns≠0 → deadline = t0+预算 (覆盖级0种子+全部重启);
	// 0 → time_point::max() (永不到时, 与历史行为逐位一致)
	const auto deadline = (cfg_.timeout_ns != 0)
		? t0 + std::chrono::nanoseconds(static_cast<int64_t>(cfg_.timeout_ns))
		: std::chrono::steady_clock::time_point::max();
	bool timed_out = false;
	auto finish = [&](IkResult r)
	{
		if (stats)
		{
			stats->ok = (r == IkResult::OK);
			stats->solve_us = std::chrono::duration<double, std::micro>(
				std::chrono::steady_clock::now() - t0).count();
			stats->timed_out = timed_out;
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
	// 预算耗尽判定: 置 timed_out (stats/日志消费) 并返回 true —— 阶梯据此停烧重启
	auto budgetGone = [&]()
	{
		if (cfg_.timeout_ns != 0 && std::chrono::steady_clock::now() >= deadline)
		{
			timed_out = true;
			return true;
		}
		return false;
	};
	// 级 0: 调用方种子 (流式连续性根)
	std::vector<double> sol;
	int iters = 0;
	// 失败归因跟踪 (遥测/分类): att_* = 本尝试退出值; best_fail_* = 加权误差最低的
	// 失败尝试; sticky_* = 粘性拒绝的收敛解 (有解被拒 ≠ 不可解, 分类语义分开)
	double att_sigma = -1.0, att_err = -1.0;
	double best_fail_err = 1e9, best_fail_sigma = -1.0, sticky_sigma = -1.0;
	IkResult r = solveOnce(target, seed, red, sol, deadline, cfg_.max_iterations, &iters,
		&att_sigma, &att_err);
	if (r == IkResult::OK)
	{
		if (stats) {stats->iterations = iters; stats->restarts_used = 0; stats->min_sigma = att_sigma;}
		out_q = sol;
		return finish(IkResult::OK);
	}
	if (att_err >= 0.0 && att_err < best_fail_err)
	{
		best_fail_err = att_err;
		best_fail_sigma = att_sigma;
	}

	// 级 0.5: 种子库 (已加载且 COLD_START) —— 目标导向种子先于分支/随机。
	// 组合度量 dp + w·(1−|q·q'|) 全量打分 (~19µs@8k) + partial_sort 前 12 候选
	// (~50µs) + 分支封顶 ≤2 (查询端多样性, 2026-09-17 实测 k≥4 起赢纯 top-k)。
	// STREAMING 不走: warm 种子 + 粘性已是主路径, 库种子会被 jump 阈值拒掉白烧预算。
	if (!seed_lib_.empty() && mode == SolveMode::COLD_START)
	{
		lib_scratch_.clear();
		for (std::size_t i = 0; i < seed_lib_.size(); ++i)
		{
			const SeedEntry & e = seed_lib_[i];
			const double dp = std::sqrt(
				(e.pose.x - target.x) * (e.pose.x - target.x) +
				(e.pose.y - target.y) * (e.pose.y - target.y) +
				(e.pose.z - target.z) * (e.pose.z - target.z));
			const double qd = std::fabs(target.qw * e.pose.qw + target.qx * e.pose.qx +
				target.qy * e.pose.qy + target.qz * e.pose.qz);
			lib_scratch_.emplace_back(dp + kLibRankRotWeight * (1.0 - qd), i);
		}
		const std::size_t pool = std::min(lib_scratch_.size(), kLibPool);
		std::partial_sort(lib_scratch_.begin(),
			lib_scratch_.begin() + static_cast<std::ptrdiff_t>(pool), lib_scratch_.end());
		int branch_cnt[kLibBranchCount] = {0, 0, 0, 0};
		std::size_t picked = 0;
		for (std::size_t pi = 0; pi < pool && picked < kLibTopK; ++pi)
		{
			if (budgetGone()) {break;}
			const SeedEntry & e = seed_lib_[lib_scratch_[pi].second];
			if (branch_cnt[e.branch] >= kLibBranchCap) {continue;}
			branch_cnt[e.branch] += 1;
			const int restart_cap_eff = (cfg_.restart_max_iterations > 0)
				? cfg_.restart_max_iterations : cfg_.max_iterations;
			r = solveOnce(target, e.q, red, sol, deadline, restart_cap_eff, &iters,
				&att_sigma, &att_err);
			if (stats) {stats->restarts_used = static_cast<int>(picked) + 1;}
			++picked;
			if (r == IkResult::OK)
			{
				// COLD_START 无粘性检查 (与阶梯既有语义一致)
				if (stats) {stats->iterations = iters; stats->min_sigma = att_sigma;}
				out_q = sol;
				return finish(IkResult::OK);
			}
			if (att_err >= 0.0 && att_err < best_fail_err)
			{
				best_fail_err = att_err;
				best_fail_sigma = att_sigma;
			}
		}
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
	const int restart_cap = (cfg_.restart_max_iterations > 0)
		? cfg_.restart_max_iterations : cfg_.max_iterations;
	for (int k = 0; k < cfg_.restart_count; ++k)
	{
		if (budgetGone()) {break;}   // 每次重启前查预算 (级 0 耗尽在此短路)
		const auto s = heuristic_seed(static_cast<std::size_t>(k));
		r = solveOnce(target, s, red, sol, deadline, restart_cap, &iters, &att_sigma, &att_err);
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
				if (stats) {stats->iterations = iters; stats->min_sigma = att_sigma;}
				out_q = sol;
				return finish(IkResult::OK);
			}
			if (dist < best_dist)
			{
				best_dist = dist;   // 记住最近的, 若全部超阈值则报跳变
				best_sol = sol;
				sticky_sigma = att_sigma;   // 被拒解的 σ (遥测: 常是近奇异解被粘性拦下)
			}
		}
		else if (att_err >= 0.0 && att_err < best_fail_err)
		{
			best_fail_err = att_err;
			best_fail_sigma = att_sigma;
		}
	}
	budgetGone();   // 兜底: 最后一种子烧穿预算而阶梯走完的场合 (耗时>预算即如实标注)
	// 失败归因 (2026-09-17): ①粘性拒绝场合 = 有解被拒 (不可解≠被拒, 分类语义分开,
	// 维持 ITERATION_LIMIT, σ 取被拒解); ②全部尝试失败场合 = 按 σ_min 分类
	// NEAR_SINGULAR (奇异邻域, CM 警告保持不重试) / ITERATION_LIMIT (瞬态, 可重试)
	if (!best_sol.empty())
	{
		if (stats)
		{
			CartesianPose back;
			if (fk_->fk(best_sol, back))
			{
				const Eigen::Matrix<double, 6, 1> e = poseError(back, target);
				best_final_err = e.head<3>().norm();
				stats->final_err = best_final_err;
			}
			stats->min_sigma = sticky_sigma;
		}
		ULOG_WARN("dls_ik 失败(粘性拒绝): 目标(%.3f,%.3f,%.3f) 重启%d次 最近解距种子%.2frad 位置残差%.4fm",
			target.x, target.y, target.z,
			stats ? stats->restarts_used : cfg_.restart_count,
			best_dist < 1e8 ? best_dist : -1.0, best_final_err);
		return finish(IkResult::ITERATION_LIMIT);
	}
	if (stats && best_fail_sigma >= 0.0)
	{
		stats->final_err = best_fail_err;
		stats->min_sigma = best_fail_sigma;
	}
	ULOG_WARN("dls_ik 失败%s%s: 目标(%.3f,%.3f,%.3f) 重启%d次 σ_min=%.3f 加权残差%.4f",
		timed_out ? "(超时)" : "",
		(cfg_.near_singular_sigma > 0.0 && best_fail_sigma >= 0.0 &&
			best_fail_sigma < cfg_.near_singular_sigma) ? "(近奇异)" : "",
		target.x, target.y, target.z,
		stats ? stats->restarts_used : cfg_.restart_count,
		best_fail_sigma, best_fail_err < 1e8 ? best_fail_err : -1.0);
	if (cfg_.near_singular_sigma > 0.0 && best_fail_sigma >= 0.0 &&
		best_fail_sigma < cfg_.near_singular_sigma)
	{
		return finish(IkResult::NEAR_SINGULAR);
	}
	return finish(IkResult::ITERATION_LIMIT);
}

}  // namespace unistackbot_controller

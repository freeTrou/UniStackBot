#include "base_kinematics/base_kinematics.hpp"

#include <cmath>

namespace unistackbot_algorithm
{
namespace
{

/*
 * NSteerKinematics —— N 舵轮独立转向 (2026-10-15, 底盘域 B1)。
 * 一份代码覆盖整族: 三舵轮 (r1_pro: 前二后一) / 四舵轮 (galbot) / 双舵轮, N 是参数。
 *
 * cfg.joints 顺序 = 模块序: [steer_1, wheel_1, steer_2, wheel_2, ...] (2N 通道)。
 * cfg.params = [轮半径 r, 零位偏置×N, 安装位 x×N, y×N] (相对 base, 转向轴位置, m)。
 *   注: r1_pro 轮相对转向轴有 -0.094m 偏距 —— 一级运动学按共点处理 (偏距效应=
 *   转向自对中, 属动力学, 不入逆解); 偏距显著机型将来加偏距参数项。
 *
 * 逆解 (每模块): 轮触点速度 = v + ω×p_i
 *   → 模块方向角 α_i = atan2(vy_i, vx_i), 轮速 ω_i = |v_i| / r
 * 正解: v = Σ v_i / N, ω = Σ (p_i × v_i)_z / Σ|p_i|²  (最小二乘, 冗余模块容错)
 *
 * 两件算法坑 (预研在案):
 *   - 过零翻转: α 与 α+π 等效 (轮速反号)。取与上次指令角的角距近者, 翻转时
 *     轮速取反 —— 转向电机少转半圈, 高速域防转向超速
 *   - 低速死区: |v_i| < 死区 且 twist 非零 → 保持上次转角 (只驱动轮速);
 *     twist≈0 → 转角缓回 0? 不 —— 保持 (回零会造成下次起跑转向瞬转)
 */
constexpr double kFlipEps = 1e-9;    // 模块速度幅值低于此 = 无方向, 保持转角
constexpr double kRad2Wrap = 2.0 * M_PI;

double wrapPi(double a)
{
	while (a > M_PI) { a -= kRad2Wrap; }
	while (a < -M_PI) { a += kRad2Wrap; }
	return a;
}

class NSteerKinematics final : public BaseKinematics
{
public:
	[[nodiscard]] bool init(const BaseKinConfig & cfg, std::string & err) override
	{
		if (cfg.joints.size() < 4 || cfg.joints.size() % 2 != 0)
		{
			err = "n_steer 需要 joints=[steer_1, wheel_1, ...] 偶数对通道 (2N, N≥2), 实得 " +
				std::to_string(cfg.joints.size());
			return false;
		}
		const std::size_t n = cfg.joints.size() / 2;
		if (cfg.params.size() != 1 + 3 * n)
		{
			err = "n_steer 需要 params=[轮半径, 零位偏置×N, 安装位x×N, y×N] 共 " +
				std::to_string(1 + 3 * n) + " 个, 实得 " + std::to_string(cfg.params.size());
			return false;
		}
		if (!std::isfinite(cfg.params[0]) || !(cfg.params[0] > 0.0))
		{
			err = "n_steer 轮半径必须为正有限";
			return false;
		}
		for (std::size_t i = 0; i < n; ++i)
		{
			const double off = cfg.params[1 + i];
			const double px = cfg.params[1 + n + i];
			const double py = cfg.params[1 + 2 * n + i];
			if (!std::isfinite(off) || !std::isfinite(px) || !std::isfinite(py))
			{
				err = "n_steer 模块 " + std::to_string(i) + " 偏置/安装位非有限";
				return false;
			}
			if (std::fabs(px) < 1e-6 && std::fabs(py) < 1e-6)
			{
				err = "n_steer 模块 " + std::to_string(i) + " 安装位在质心 —— 力矩臂为零, 正解退化";
				return false;
			}
		}
		n_ = n;
		r_ = cfg.params[0];
		offset_.resize(n);
		px_.resize(n);
		py_.resize(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			offset_[i] = cfg.params[1 + i];
			px_[i] = cfg.params[1 + n + i];
			py_[i] = cfg.params[1 + 2 * n + i];
		}
		specs_.clear();
		last_steer_.assign(n, 0.0);   // 初始转角 = 零位+偏置 (直立向前)
		for (std::size_t i = 0; i < n; ++i)
		{
			specs_.push_back({cfg.joints[2 * i], BaseActuatorType::Position});
			specs_.push_back({cfg.joints[2 * i + 1], BaseActuatorType::Velocity});
			last_steer_[i] = offset_[i];
		}
		return true;
	}

	[[nodiscard]] std::vector<BaseActuatorSpec> specs() const override {return specs_;}
	[[nodiscard]] std::size_t actuatorCount() const override {return 2 * n_;}
	[[nodiscard]] bool holonomic() const override {return true;}   // 舵轮族全向

	void toActuators(const BodyTwist & t, std::vector<double> & out) override
	{
		for (std::size_t i = 0; i < n_; ++i)
		{
			// 轮触点速度 = v + ω × p_i (平面: ω×p = (-ω·py, ω·px))
			const double vx = t.vx - t.wz * py_[i];
			const double vy = t.vy + t.wz * px_[i];
			const double speed = std::sqrt(vx * vx + vy * vy);
			double & steer = out[2 * i];
			double & wheel = out[2 * i + 1];
			if (speed < kFlipEps)
			{
				// 低速死区: 无方向 → 保持上次转角, 轮速零 (原地停: 转角不动防抖)
				steer = last_steer_[i];
				wheel = 0.0;
				continue;
			}
			double alpha = std::atan2(vy, vx);
			double w = speed / r_;
			// 过零翻转: 与上次转角比, 翻转 (α+π, 轮速反号) 若更近
			const double d_direct = std::fabs(wrapPi(alpha - last_steer_[i]));
			const double d_flip = std::fabs(wrapPi(alpha + M_PI - last_steer_[i]));
			if (d_flip < d_direct)
			{
				alpha += M_PI;
				w = -w;
			}
			steer = wrapPi(alpha);
			wheel = w;
			last_steer_[i] = steer;
		}
	}

	void fromActuators(const std::vector<double> & fb, BodyTwist & out) const override
	{
		// 每模块触点速度 v_i = v + ω×p_i → (vx, vy, wz) 三参数最小二乘 (法方程
		// 3×3, Cramer 闭式解零分配)。朴素平均在"模块质心≠base 原点"的机型
		// (r1_pro Σpx=0.011m) 上有结构性偏差 (2026-10-15 实测: wz 0.4→0.3966)
		double svx = 0.0, svy = 0.0, stau = 0.0, spx = 0.0, spy = 0.0, sm = 0.0;
		for (std::size_t i = 0; i < n_; ++i)
		{
			const double a = fb[2 * i] - offset_[i];   // 去零位偏置 → 世界向角
			const double w = fb[2 * i + 1] * r_;       // 轮线速度 (带符号)
			const double cvx = std::cos(a) * w;        // 触点速度 = unit(α)·轮线速度
			const double cvy = std::sin(a) * w;
			svx += cvx;
			svy += cvy;
			stau += px_[i] * cvy - py_[i] * cvx;       // (p × v_i)_z
			spx += px_[i];
			spy += py_[i];
			sm += px_[i] * px_[i] + py_[i] * py_[i];
		}
		// 法方程 M·x = b: [[N,0,-spy],[0,N,spx],[-spy,spx,sm]]·[vx,vy,wz]ᵀ = [svx,svy,stau]ᵀ
		const auto det3 = [](double a11, double a12, double a13, double a21, double a22, double a23, double a31, double a32, double a33)
		{ return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31); };
		const double dn = static_cast<double>(n_);
		const double D = det3(dn, 0.0, -spy, 0.0, dn, spx, -spy, spx, sm);   // N²(S−px̄²−pȳ²) > 0 (init 拒零力矩臂)
		out.vx = det3(svx, 0.0, -spy, svy, dn, spx, stau, spx, sm) / D;
		out.vy = det3(dn, svx, -spy, 0.0, svy, spx, -spy, stau, sm) / D;
		out.wz = det3(dn, 0.0, svx, 0.0, dn, svy, -spy, spx, stau) / D;
	}

	[[nodiscard]] const char * name() const override {return "n_steer";}

private:
	std::size_t n_{3};
	double r_{0.1};
	std::vector<double> offset_, px_, py_;
	std::vector<double> last_steer_;   // 过零翻转记忆 (实例态, 契约 3)
	std::vector<BaseActuatorSpec> specs_;
};

}  // namespace

// 自注册入口 (外链: base_kinematics.cpp 的 ensureBuiltin 首次工厂调用时执行)
void registerNSteerKinematics()
{
	(void)registerBaseKinematics("n_steer", []() -> std::unique_ptr<BaseKinematics> {return std::make_unique<NSteerKinematics>(); });
}

}  // namespace unistackbot_algorithm

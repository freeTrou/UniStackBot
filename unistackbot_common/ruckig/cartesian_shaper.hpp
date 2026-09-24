#ifndef UNISTACKBOT_COMMON__CARTESIAN_SHAPER_HPP_
#define UNISTACKBOT_COMMON__CARTESIAN_SHAPER_HPP_

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>

#include "ruckig/otg_stream.hpp"

namespace unistackbot_common
{

/*
 * CartesianShaper —— 笛卡尔位姿流整形器 (2026-09-23, 组件层; 零新数学纯组合)。
 *
 * 定位: "OtgStream = 全工程平滑唯一入口" 裁决下的笛卡尔侧能力补齐 ——
 * 社区版 Ruckig 无 SE(3) 插值 (Pro 功能), 本类用两个 OtgStream<3> 组合出来:
 *   位置通道: OtgStream<3> 直接整形 (x, y, z) —— C2 全受限
 *   姿态通道: 锚点切空间整形 —— SO(3) 不便流式整形, 映射到锚点 rotation
 *     vector (R³ 线性空间) 后与位置通道完全同构:
 *       r_t = log(anchor⁻¹ · q_target) → OtgStream<3> 流式整形 → q_out = anchor · exp(r_out)
 *     anchor = reset 时姿态 (激活内恒定); 恒定/已到位姿态 r=0 直通。
 *     (设计教训 2026-09-23: "每拍 reset 注入实测 θ" 的闭环方案被否 —— OtgStream
 *      reset() 强制下拍重算且首个 update 输出 t=0 状态, 每拍 reset = 推进/停滞
 *      交替, 速度永远建不起来; 流式整形必须保持底座状态延续, 切空间正是为此。)
 *
 * 契约对齐 OtgStream (消费者心智单一):
 *   update() 只有 Ok / Hold 两种结局, **Hold 下 out 也恒有效** (安全保持位);
 *   输入 NaN/Inf/非法四元数 → Hold (错误码可观测); 观测面同款四件套。
 *
 * 线程: 实例每线程一份 (同 OtgStream 契约)。稳态零 malloc (底座已实证)。
 * 边界: ① 目标姿态与 anchor 夹角接近 π 时 rotation vector 方向退化 (切空间
 * 覆盖不到的对跖点) —— 大范围重定向场景先 reset 换锚; ② 姿态噪声流上层先滤
 * (组件只整形不估计)。
 */

// 组件边界 POD (零 ROS 依赖; 四元数 w 在前, 与 algorithm 包同约定)
struct ShaperPose
{
	double x{0.0}, y{0.0}, z{0.0};
	double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
};

class CartesianShaper
{
public:
	struct Limits
	{
		// 逐轴语义 (与关节 OTG 同构): 合成笛卡尔速度最高 √3·max_velocity。
		// 需要各向同性限速的消费方把 max_velocity 设为 v_iso/√3。
		std::array<double, 3> max_velocity{0.5, 0.5, 0.5};        // 位置 [m/s]
		std::array<double, 3> max_acceleration{2.0, 2.0, 2.0};   // [m/s²]
		std::array<double, 3> max_jerk{10.0, 10.0, 10.0};        // [m/s³]
		double max_angular_velocity{1.0};        // 姿态切空间 [rad/s]
		double max_angular_acceleration{5.0};    // [rad/s²]
		double max_angular_jerk{50.0};           // [rad/s³]
	};

	enum class UpdateResult : uint8_t
	{
		Ok,
		Hold,
	};

	// 错误码: OtgStream 同款约定 (<0 Ruckig 原码 / 4xx 封装自定义)
	enum LastError : int
	{
		NotInitialized = -1,
		NonFiniteTarget = 400,
		NonFiniteOutput = 401,
		InvalidQuaternion = 402,
	};

	// dt: 整形输出周期 [s]; max_target_jump: 位置单拍目标跳变限幅 [m] (0=不限, 同 OtgStream)
	[[nodiscard]] bool init(double dt, const Limits & limits, double max_target_jump = 0.1)
	{
		typename OtgStream<3>::Limits lp;
		lp.max_velocity = limits.max_velocity;
		lp.max_acceleration = limits.max_acceleration;
		lp.max_jerk = limits.max_jerk;
		typename OtgStream<3>::Limits lr;
		lr.max_velocity.fill(limits.max_angular_velocity);
		lr.max_acceleration.fill(limits.max_angular_acceleration);
		lr.max_jerk.fill(limits.max_angular_jerk);
		// 位置跳变限幅透传; 姿态切空间不限幅 (输入已消毒, 大范围重定向是合法目标)
		if (!pos_otg_.init(dt, lp, max_target_jump) || !rot_otg_.init(dt, lr, 0.0))
		{
			return false;   // 限值/dt 校验由底座完成 (正/有限/<1e9)
		}
		dt_ = dt;
		initialized_ = false;
		update_count_ = 0;
		error_count_ = 0;
		last_error_ = NotInitialized;
		return true;
	}

	// 重置到当前位姿 (激活/重激活; 输入消毒与 update 同标准 —— 位置非有限时回退
	// 原点并记错, 姿态四元数规范化, 退化时回退单位四元数; 2026-09-23 审查补:
	// 旧版只消毒姿态不消毒位置, NaN 基准会随 initialized_=true 流进每拍 Hold 输出)
	void reset(const ShaperPose & current)
	{
		double x = current.x;
		double y = current.y;
		double z = current.z;
		if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
		{
			x = 0.0;
			y = 0.0;
			z = 0.0;
			++error_count_;
			last_error_ = NonFiniteTarget;
		}
		Eigen::Quaterniond q = toQuat(current);
		if (q.norm() < 1e-9 || !std::isfinite(q.norm()))
		{
			q = Eigen::Quaterniond::Identity();
			++error_count_;
			last_error_ = InvalidQuaternion;
		}
		else
		{
			q.normalize();
		}
		q_anchor_ = q;
		q_out_ = q;
		pos_otg_.reset({x, y, z});
		rot_otg_.reset({0.0, 0.0, 0.0});
		last_.x = x;
		last_.y = y;
		last_.z = z;
		poseToQuat(q, last_);
		initialized_ = true;
	}

	// 每拍: target = 最新笛卡尔目标 (点流语义); out = 本拍输出 (恒有效)
	[[nodiscard]] UpdateResult update(const ShaperPose & target, ShaperPose & out)
	{
		++update_count_;
		if (!initialized_)
		{
			hold(out);
			return UpdateResult::Hold;   // 未 reset: 无安全基准
		}
		// ---- 输入消毒: 位置有限性 ----
		if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z))
		{
			++error_count_;
			last_error_ = NonFiniteTarget;
			hold(out);
			return UpdateResult::Hold;
		}
		// ---- 输入消毒: 四元数有限 + 非零范数 ----
		Eigen::Quaterniond qt = toQuat(target);
		if (!std::isfinite(qt.w()) || !std::isfinite(qt.x()) ||
			!std::isfinite(qt.y()) || !std::isfinite(qt.z()) || qt.norm() < 1e-9)
		{
			++error_count_;
			last_error_ = InvalidQuaternion;
			hold(out);
			return UpdateResult::Hold;
		}
		qt.normalize();
		if (qt.dot(q_anchor_) < 0.0)
		{
			qt = Eigen::Quaterniond(-qt.w(), -qt.x(), -qt.y(), -qt.z());   // 切空间最短侧
		}

		// ---- 位置通道: OtgStream<3> (Hold 时其 out 恒为安全保持位) ----
		std::array<double, 3> p_out{};
		const auto rp = pos_otg_.update({target.x, target.y, target.z}, p_out);

		// ---- 姿态通道: 锚点切空间 (rotation vector) 流式整形, 与位置同构 ----
		const Eigen::Quaterniond q_rel = q_anchor_.conjugate() * qt;
		const Eigen::Vector3d r_t = rotvec(q_rel);
		std::array<double, 3> r_out{};
		const auto rr = rot_otg_.update({r_t.x(), r_t.y(), r_t.z()}, r_out);
		q_out_ = q_anchor_ * quatFromRotvec(Eigen::Vector3d(r_out[0], r_out[1], r_out[2]));
		q_out_.normalize();

		// ---- 输出组装 + 校验 ----
		out.x = p_out[0];
		out.y = p_out[1];
		out.z = p_out[2];
		poseToQuat(q_out_, out);
		const bool out_finite = std::isfinite(out.x) && std::isfinite(out.y) &&
			std::isfinite(out.z) && std::isfinite(out.qw) && std::isfinite(out.qx) &&
			std::isfinite(out.qy) && std::isfinite(out.qz);
		if (!out_finite)
		{
			++error_count_;
			last_error_ = NonFiniteOutput;
			hold(out);
			return UpdateResult::Hold;
		}
		last_ = out;
		if (rp != OtgStream<3>::UpdateResult::Ok || rr != OtgStream<3>::UpdateResult::Ok)
		{
			last_error_ = (rp != OtgStream<3>::UpdateResult::Ok) ? pos_otg_.lastError()
			                                                     : rot_otg_.lastError();
			return UpdateResult::Hold;
		}
		return UpdateResult::Ok;
	}

	[[nodiscard]] uint64_t updateCount() const {return update_count_;}
	[[nodiscard]] uint64_t errorCount() const {return error_count_;}
	[[nodiscard]] int lastError() const {return last_error_;}

private:
	static Eigen::Quaterniond toQuat(const ShaperPose & p)
	{
		return Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz);
	}

	static void poseToQuat(const Eigen::Quaterniond & q, ShaperPose & p)
	{
		p.qw = q.w();
		p.qx = q.x();
		p.qy = q.y();
		p.qz = q.z();
	}

	// 四元数 → rotation vector (最短侧: w<0 先取反; AngleAxis angle ∈ [0, π])
	static Eigen::Vector3d rotvec(const Eigen::Quaterniond & q_in)
	{
		Eigen::Quaterniond q = q_in;
		if (q.w() < 0.0)
		{
			q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
		}
		const Eigen::AngleAxisd aa(q);
		return aa.axis() * aa.angle();
	}

	// rotation vector → 四元数 (零向量 = 单位)
	static Eigen::Quaterniond quatFromRotvec(const Eigen::Vector3d & r)
	{
		const double n = r.norm();
		if (n < 1e-12)
		{
			return Eigen::Quaterniond::Identity();
		}
		return Eigen::Quaterniond(Eigen::AngleAxisd(n, r / n));
	}

	void hold(ShaperPose & out) const
	{
		out = last_;   // Hold = 保持上一拍安全位 (契约同 OtgStream)
	}

	OtgStream<3> pos_otg_{};
	OtgStream<3> rot_otg_{};
	Eigen::Quaterniond q_anchor_{Eigen::Quaterniond::Identity()};
	Eigen::Quaterniond q_out_{Eigen::Quaterniond::Identity()};
	ShaperPose last_{};
	double dt_{0.002};
	bool initialized_{false};
	uint64_t update_count_{0};
	uint64_t error_count_{0};
	int last_error_{NotInitialized};
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__CARTESIAN_SHAPER_HPP_

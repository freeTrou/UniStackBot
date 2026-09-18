#ifndef UNISTACKBOT_COMMON__OTG_STREAM_HPP_
#define UNISTACKBOT_COMMON__OTG_STREAM_HPP_

#include <array>
#include <cmath>
#include <cstdint>

#include <ruckig/ruckig.hpp>

namespace unistackbot_common
{

/*
 * OtgStream —— Ruckig 社区版的防御性封装 (2026-09-18, 评审"数值稳定性"清单落地;
 * 同日二轮评审修正接口契约: Hold 时输出恒有效)。
 *
 * 社区版已知风险: -101/-110/-111 计算失败、零目标态不稳定、野输入放大。
 * 防御原则: 永不盲信 —— 输入消毒 → 返回码检查 → 输出校验 → 失败保持 + 可观测。
 *
 * 行为契约 (修订版):
 *   update() 返回 Ok / Hold, **out_pos 两种结局下都有效**:
 *     Ok   —— out_pos = 本拍参考位
 *     Hold —— out_pos = 上一拍安全保持位 (调用方无条件写硬件即可, 不可能用错)
 *   错误自愈: otg.reset() + 状态回退, 下拍重新规划。
 *   位置模式假设: Hold 期间硬件伺服保持位 → 臂静止, 重规划假设静止成立
 *   (力矩模式需调用方以 reset(pos, vel) 喂真实状态 —— 文档化边界)。
 *
 * 线程: 实例每线程一份 (内部 Ruckig 同契约), 多线程共享 = 未定义。
 * 稳态 update 零 malloc (T4 实证)。
 */
template <std::size_t DOFs>
class OtgStream
{
public:
	struct Limits
	{
		std::array<double, DOFs> max_velocity;
		std::array<double, DOFs> max_acceleration;
		std::array<double, DOFs> max_jerk;
	};

	enum class UpdateResult : uint8_t
	{
		Ok,
		Hold,
	};

	// 错误码: Ruckig 原码 (<0, 含 -100..-111) / 封装自定义 (400/401/-1)
	enum LastError : int
	{
		NotInitialized = -1,
		NonFiniteTarget = 400,
		NonFiniteOutput = 401,
	};

	// dt: 控制周期 [s]; max_target_jump: 单拍目标跳变限幅 [rad] (0=不限; 默认 0.1 保守值,
	// 只拦"病态跳变", 正常命令流不受约束 —— 与输出侧步长饱和不同层)
	[[nodiscard]] bool init(double dt, const Limits & limits, double max_target_jump = 0.1)
	{
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			if (!std::isfinite(limits.max_velocity[i]) || limits.max_velocity[i] <= 0.0 ||
				!std::isfinite(limits.max_acceleration[i]) || limits.max_acceleration[i] <= 0.0 ||
				!std::isfinite(limits.max_jerk[i]) || limits.max_jerk[i] <= 0.0 ||
				limits.max_velocity[i] > 1e9 || limits.max_acceleration[i] > 1e9 ||
				limits.max_jerk[i] > 1e9)
			{
				return false;   // 限值必须为正、有限、低于 Ruckig 数值范围假设 (1e9)
			}
		}
		if (!std::isfinite(dt) || dt <= 0.0)
		{
			return false;
		}
		otg_.delta_time = dt;
		lim_ = limits;
		max_target_jump_ = max_target_jump;
		initialized_ = false;
		update_count_ = 0;
		error_count_ = 0;
		last_error_ = NotInitialized;
		last_duration_ = 0.0;
		return true;
	}

	// 重置到指定当前状态 (激活/重激活/错误恢复后; 速度默认 0 = 安全静止,
	// 力矩模式/已知运动状态时调用方应喂实测速度)
	void reset(const std::array<double, DOFs> & current_position)
	{
		std::array<double, DOFs> zero{};
		reset(current_position, zero);
	}
	void reset(const std::array<double, DOFs> & current_position,
		const std::array<double, DOFs> & current_velocity)
	{
		std::array<double, DOFs> zero{};
		reset(current_position, current_velocity, zero);
	}
	void reset(const std::array<double, DOFs> & current_position,
		const std::array<double, DOFs> & current_velocity,
		const std::array<double, DOFs> & current_acceleration)
	{
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			in_.current_position[i] = current_position[i];
			in_.current_velocity[i] = current_velocity[i];
			in_.current_acceleration[i] = current_acceleration[i];
			last_out_pos_[i] = current_position[i];
			in_.max_velocity[i] = lim_.max_velocity[i];
			in_.max_acceleration[i] = lim_.max_acceleration[i];
			in_.max_jerk[i] = lim_.max_jerk[i];
			in_.target_position[i] = current_position[i];   // 未接目标前 = 保持
			last_target_[i] = current_position[i];
		}
		otg_.reset();   // 强制下拍重算
		initialized_ = true;
	}

	// 每拍: target = 上层最新目标点 (点流语义); out_pos = 本拍输出 (恒有效, 见契约)。
	// [[nodiscard]]: 忽略 Ok/Hold 的分流在控制语境几乎必错 —— 编译期强制
	[[nodiscard]] UpdateResult update(std::array<double, DOFs> target,
		std::array<double, DOFs> & out_pos)
	{
		++update_count_;
		if (!initialized_)
		{
			hold(NotInitialized, target, out_pos, true);
			return UpdateResult::Hold;   // 未 reset: 无安全基准, 输出 = 未定义调用方状态
		}

		// ---- 输入消毒 1: NaN/Inf 拒绝 (输入问题: 不推进 last_target_, 等上层改正) ----
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			if (!std::isfinite(target[i]))
			{
				hold(NonFiniteTarget, target, out_pos, false);
				return UpdateResult::Hold;
			}
		}
		// ---- 输入消毒 2: 目标跳变限幅 (防病态跳变触发数值边界) ----
		if (max_target_jump_ > 0.0)
		{
			for (std::size_t i = 0; i < DOFs; ++i)
			{
				const double d = target[i] - last_target_[i];
				if (std::fabs(d) > max_target_jump_)
				{
					target[i] = last_target_[i] + std::copysign(max_target_jump_, d);
				}
			}
		}

		// ---- 目标摄入 (消毒+限幅后) → Ruckig 输入 ----
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			in_.target_position[i] = target[i];
		}

		// ---- 调用 Ruckig ----
		const auto r = otg_.update(in_, out_);
		last_duration_ = out_.trajectory.get_duration() - out_.time;   // 剩余时长 (健康指标)
		if (last_duration_ < 0.0)
		{
			last_duration_ = 0.0;
		}

		// ---- 返回码检查: Error* 一律保持 (绝不使用失败计算的输出) ----
		if (r != ruckig::Result::Working && r != ruckig::Result::Finished)
		{
			// Ruckig 内部错误: 目标本身合法 (已消毒), 推进 last_target_ 到本次尝试的
			// (限幅后) 目标 —— 防止"重复失败 + 跳变限幅"把目标卡死在旧位 (二轮评审)
			for (std::size_t i = 0; i < DOFs; ++i)
			{
				last_target_[i] = target[i];
			}
			hold(static_cast<int>(r), target, out_pos, true);
			return UpdateResult::Hold;
		}

		// ---- 输出校验: isfinite 全查 ----
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			if (!std::isfinite(out_.new_position[i]))
			{
				hold(NonFiniteOutput, target, out_pos, false);
				return UpdateResult::Hold;
			}
		}

		// ---- 采纳: 状态前滚 + 观测 ----
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			out_pos[i] = out_.new_position[i];
			last_out_pos_[i] = out_.new_position[i];
			last_target_[i] = target[i];
		}
		in_.current_position = out_.new_position;
		in_.current_velocity = out_.new_velocity;
		in_.current_acceleration = out_.new_acceleration;
		return UpdateResult::Ok;
	}

	// 观测 (健康监控/日志)。lastDuration = Ruckig 剩余时长 [s] —— 持续大于控制周期
	// 说明目标过于激进 (调参信号); Hold 时可能为 0 (算不出即无轨迹)
	[[nodiscard]] uint64_t updateCount() const {return update_count_;}
	[[nodiscard]] uint64_t errorCount() const {return error_count_;}
	[[nodiscard]] int lastError() const {return last_error_;}
	[[nodiscard]] double lastDuration() const {return last_duration_;}
	[[nodiscard]] std::array<double, DOFs> lastVelocity() const {return out_.new_velocity;}
	[[nodiscard]] std::array<double, DOFs> lastAcceleration() const {return out_.new_acceleration;}
	[[nodiscard]] bool isInitialized() const {return initialized_;}

private:
	// 保持处理: 错误码记账 + (可选推进目标) + otg 重置 + 状态回退 + 输出安全值。
	// advance_target: 仅 Ruckig 内部错误为 true (目标已消毒合法, 推进防卡死);
	//                 输入拒绝为 false (等上层改正)。
	// out_pos: 恒写入 last_out_pos_ (安全保持位) —— "调用方不可能用错"契约。
	void hold(int code, const std::array<double, DOFs> & attempted_target,
		std::array<double, DOFs> & out_pos, bool advance_target)
	{
		++error_count_;
		last_error_ = code;
		if (advance_target)
		{
			for (std::size_t i = 0; i < DOFs; ++i)
			{
				last_target_[i] = attempted_target[i];
			}
		}
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			out_pos[i] = last_out_pos_[i];   // 契约: Hold 输出 = 安全保持位
			in_.current_position[i] = last_out_pos_[i];
			in_.current_velocity[i] = 0.0;   // 位置模式: 保持位伺服使臂静止, 近似成立
			in_.current_acceleration[i] = 0.0;
			in_.target_position[i] = last_target_[i];
		}
		otg_.reset();   // 强制下拍重算
	}

	ruckig::Ruckig<DOFs> otg_{};
	ruckig::InputParameter<DOFs> in_;
	ruckig::OutputParameter<DOFs> out_;
	Limits lim_{};
	double max_target_jump_{0.1};
	bool initialized_{false};
	std::array<double, DOFs> last_out_pos_{};
	std::array<double, DOFs> last_target_{};
	uint64_t update_count_{0};
	uint64_t error_count_{0};
	int last_error_{NotInitialized};
	double last_duration_{0.0};
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__OTG_STREAM_HPP_

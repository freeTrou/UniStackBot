#ifndef UNISTACKBOT_COMMON__OTG_STREAM_HPP_
#define UNISTACKBOT_COMMON__OTG_STREAM_HPP_

#include <array>
#include <cmath>
#include <cstdint>

#include <ruckig/ruckig.hpp>

namespace unistackbot_common
{

/*
 * OtgStream —— Ruckig 社区版的防御性封装 (2026-09-18, 评审"数值稳定性"清单落地)。
 *
 * 社区版已知风险 (官方定位"学术项目的最终状态", 边界输入可致计算失败):
 *   -101 轨迹时长超数值极限 / -110,-111 极值时间与同步计算失败 / 零目标态不稳定
 * 上层 (RL/视觉/遥操) 的"野"输入会放大这些风险。防御原则:
 *   永不盲信 —— 输入消毒 → 返回码检查 → 输出校验 → 失败保持 + 可观测。
 *
 * 行为契约:
 *   update() 只有两种结局:
 *     Ok   —— 输出本拍参考位 (经全部校验)
 *     Hold —— 输出保持上一拍安全位 (失败/拒绝, 不抛不崩), 错误可观测
 *   错误后自愈: otg.reset() + 状态回退到"保持位静止", 下拍从静止重新规划。
 *
 * 线程: 实例每线程一份 (内部 Ruckig 同契约)。稳态 update 零 malloc (T4 实证)。
 */
template <std::size_t DOFs>
class OtgStream
{
public:
	struct Limits
	{
		std::array<double, DOFs> max_velocity;    // [rad/s]
		std::array<double, DOFs> max_acceleration; // [rad/s²]
		std::array<double, DOFs> max_jerk;         // [rad/s³]
	};

	enum class UpdateResult : uint8_t
	{
		Ok,     // 输出本拍参考位
		Hold,   // 保持上一拍输出 (输入拒绝/计算失败/输出非法)
	};

	// dt: 控制周期 [s]; max_target_jump: 单拍目标跳变限幅 [rad] (0=不限 —— 防病态
	// 跳变的软限, 默认 0.5; 限的是"目标本身"的变化率, 与输出侧步长饱和不同层)
	[[nodiscard]] bool init(double dt, const Limits & limits, double max_target_jump = 0.5)
	{
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			if (!std::isfinite(limits.max_velocity[i]) || limits.max_velocity[i] <= 0.0 ||
				!std::isfinite(limits.max_acceleration[i]) || limits.max_acceleration[i] <= 0.0 ||
				!std::isfinite(limits.max_jerk[i]) || limits.max_jerk[i] <= 0.0 ||
				limits.max_velocity[i] > 1e9 || limits.max_acceleration[i] > 1e9 ||
				limits.max_jerk[i] > 1e9)
			{
				return false;   // 限值必须为正、有限、且低于 Ruckig 数值范围假设 (1e9)
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
		last_error_ = 0;
		return true;
	}

	// 重置到指定当前位 (激活/重激活/错误恢复后调用; 速度/加速度清零 = 安全静止态)
	void reset(const std::array<double, DOFs> & current_position)
	{
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			in_.current_position[i] = current_position[i];
			in_.current_velocity[i] = 0.0;
			in_.current_acceleration[i] = 0.0;
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

	// 每拍: target = 上层最新目标点 (点流语义); out_pos = 本拍参考位。
	// Hold 时 out_pos 未被写入 (调用方保持既有命令 = 安全语义)
	UpdateResult update(std::array<double, DOFs> target, std::array<double, DOFs> & out_pos)
	{
		++update_count_;
		if (!initialized_)
		{
			++error_count_;
			last_error_ = -1;
			return UpdateResult::Hold;   // 未 reset 过: 无安全基准
		}

		// ---- 输入消毒 1: NaN/Inf 拒绝 ----
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			if (!std::isfinite(target[i]))
			{
				hold(400);   // 自定义码: 非有限目标
				return UpdateResult::Hold;
			}
		}
		// ---- 输入消毒 2: 目标跳变限幅 (防病态跳变触发数值边界; 与输出步长饱和不同层) ----
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

		// ---- 返回码检查: Error* 一律保持 (绝不使用失败计算的输出) ----
		if (r != ruckig::Result::Working && r != ruckig::Result::Finished)
		{
			hold(static_cast<int>(r));
			return UpdateResult::Hold;
		}

		// ---- 输出校验: isfinite 全查 ----
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			if (!std::isfinite(out_.new_position[i]))
			{
				hold(401);   // 自定义码: 输出非有限
				return UpdateResult::Hold;
			}
		}

		// ---- 采纳: 状态前滚 (pass_to_input) + 观测 ----
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

	// 观测 (健康监控/日志)
	[[nodiscard]] uint64_t updateCount() const {return update_count_;}
	[[nodiscard]] uint64_t errorCount() const {return error_count_;}
	[[nodiscard]] int lastError() const {return last_error_;}
	// 错误码语义: Ruckig 原码 (<0) / 400=非有限目标 / 401=非有限输出 / -1=未 reset

private:
	void hold(int code)
	{
		++error_count_;
		last_error_ = code;
		// 自愈: 强制下拍重算 + 内部状态回退到"保持位静止" (物理上臂确实被保持了)
		otg_.reset();
		for (std::size_t i = 0; i < DOFs; ++i)
		{
			in_.current_position[i] = last_out_pos_[i];
			in_.current_velocity[i] = 0.0;
			in_.current_acceleration[i] = 0.0;
			in_.target_position[i] = last_target_[i];
		}
	}

	ruckig::Ruckig<DOFs> otg_{};   // dt 在 init() 设定 (delta_time 为 public 成员)
	ruckig::InputParameter<DOFs> in_;
	ruckig::OutputParameter<DOFs> out_;
	Limits lim_{};
	double max_target_jump_{0.5};
	bool initialized_{false};
	std::array<double, DOFs> last_out_pos_{};
	std::array<double, DOFs> last_target_{};
	uint64_t update_count_{0};
	uint64_t error_count_{0};
	int last_error_{0};
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__OTG_STREAM_HPP_

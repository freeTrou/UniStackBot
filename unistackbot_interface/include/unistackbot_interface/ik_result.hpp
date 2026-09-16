#ifndef UNISTACKBOT_INTERFACE__IK_RESULT_HPP_
#define UNISTACKBOT_INTERFACE__IK_RESULT_HPP_

#include <cstdint>

namespace unistackbot_interface
{

/*
 * IK 求解结果 —— 算法层自持的结果枚举 (2026-09-17 分层裁决:
 * 错误跟着产生它的那一层走; 设备状态住 RobotFeedback, 仿真层用 SimResult)。
 * 长文本留给日志 (ik_result_message), 短码给上层程序逻辑。
 */
enum class IkResult : uint8_t
{
	OK = 0,               // 求解成功
	UNREACHABLE = 1,      // 目标位姿超出工作空间 (可达性判定失败)
	NEAR_SINGULAR = 2,    // 目标邻近奇异, 解不连续或关节速度超限
	ITERATION_LIMIT = 3,  // 迭代上限内未收敛 (种子过远或无解)
	LIMIT_CONFLICT = 4,   // 目标与关节限位冲突 (含冗余锁定值越限)
	NOT_READY = 5,        // 求解器未就绪或调用维度不符 (契约违例)
	UNSUPPORTED = 6,      // 请求的冗余偏好本求解器不支持 (如偏置构型的 ARM_ANGLE)
};

// 每码对应的标准信息 (单一事实源, 日志侧调用)
[[nodiscard]] constexpr const char * ik_result_message(IkResult r) noexcept
{
	switch (r)
	{
		case IkResult::OK:
		{
			return "求解成功";
		}
		case IkResult::UNREACHABLE:
		{
			return "目标位姿超出工作空间";
		}
		case IkResult::NEAR_SINGULAR:
		{
			return "目标邻近奇异, 解不连续或关节速度超限";
		}
		case IkResult::ITERATION_LIMIT:
		{
			return "迭代上限内未收敛 (种子过远或无解)";
		}
		case IkResult::LIMIT_CONFLICT:
		{
			return "目标与关节限位冲突 (含冗余锁定值越限)";
		}
		case IkResult::NOT_READY:
		{
			return "求解器未就绪或调用维度不符";
		}
		case IkResult::UNSUPPORTED:
		{
			return "请求的冗余偏好本求解器不支持";
		}
	}
	return "未知 IkResult";
}

}  // namespace unistackbot_interface
#endif  // UNISTACKBOT_INTERFACE__IK_RESULT_HPP_

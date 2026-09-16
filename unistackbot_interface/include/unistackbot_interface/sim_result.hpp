#ifndef UNISTACKBOT_INTERFACE__SIM_RESULT_HPP_
#define UNISTACKBOT_INTERFACE__SIM_RESULT_HPP_

#include <cstdint>

namespace unistackbot_interface
{

/*
 * 仿真后端结果 —— 仿真层的结果枚举 (2026-09-17 分层裁决: 错误跟着产生它的层走,
 * 但结果码是纯类型, 文件居所随类型包)。/sim_control 应答迁到此码时引用。
 * 算法层用 IkResult; 设备状态住 RobotFeedback; 总线层结果等 hardware 落地随需另立。
 */
enum class SimResult : uint8_t
{
	OK = 0,               // 同步成功
	ACCEPTED = 1,         // 命令已接受 (异步执行中, 不代表完成 —— /sim_control 契约语义)
	UNSUPPORTED = 2,      // 当前后端不支持该命令或参数 (能力矩阵拒绝)
	NOT_READY = 3,        // 链路未就绪 (服务/仿真器/桥接未起)
	TIMEOUT = 4,          // 等待应答超时
	RESET_DANGEROUS = 5,  // reset 在该后端实测有毒 (Fortress: 返回成功但世界停摆)
};

// 每码对应的标准信息 (单一事实源, 日志侧调用)
[[nodiscard]] constexpr const char * sim_result_message(SimResult r) noexcept
{
	switch (r)
	{
		case SimResult::OK:
		{
			return "成功";
		}
		case SimResult::ACCEPTED:
		{
			return "命令已接受 (异步执行中, 不代表完成)";
		}
		case SimResult::UNSUPPORTED:
		{
			return "当前后端不支持该命令或参数";
		}
		case SimResult::NOT_READY:
		{
			return "链路未就绪 (服务/仿真器/桥接未起)";
		}
		case SimResult::TIMEOUT:
		{
			return "等待应答超时";
		}
		case SimResult::RESET_DANGEROUS:
		{
			return "reset 在该后端实测有毒 (Fortress: 返回成功但世界停摆)";
		}
	}
	return "未知 SimResult";
}

}  // namespace unistackbot_interface
#endif  // UNISTACKBOT_INTERFACE__SIM_RESULT_HPP_

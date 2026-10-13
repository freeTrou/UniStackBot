#ifndef UNISTACKBOT_INTERFACE__ERROR_CODES_HPP_
#define UNISTACKBOT_INTERFACE__ERROR_CODES_HPP_

#include <cstdint>

namespace unistackbot_interface
{

/*
 * 错误码表 —— 统一错误发布的机器可读码 (SystemAlert.code; 2026-10-12 立)。
 * 先例: IkResult (短码给上层程序逻辑, 长文本留给 message)。
 * 分域: DEVICE (关节电机/驱动器) / BUS (总线通信) / SYSTEM (控制器/软件域)。
 * 分级: severity 返回值对齐 SystemAlertArray 的 STATE_* 数值
 *       (1=仅 DEGRADED 级, 3=FAULT 级); 加码时同步补 severity 分支 —— 漏 case
 *       在编译期 switch 警告下可见 (与 statemachine 粗态映射同款纪律)。
 *
 * 段位规划 (加码按段取号, 勿跳段):
 *   1-9     SYSTEM 域 (链上软件, 仿真链即产)
 *   10-19   BUS 域 (真机批次启用)
 *   20-29   DEVICE 域通用 (真机批次启用)
 *   30+     DEVICE 域协议特有 (协议轴 error_bits 映射, 各协议分段)
 *   0xFFFE  域静默 (跨域保留)
 */
enum class ErrCode : uint16_t
{
	NONE = 0,             // 该域该定位正常
	// ---- SYSTEM 域 (1-9) ----
	CM_DEGRADED = 1,      // CM 求解连续失败降级 (DEGRADED 级: 臂保持, 可收新目标)
	CM_SILENT = 2,        // CM status active 后静默 (DEGRADED 级)
	EE_DROPOUT = 3,       // /ee_state 断流 (DEGRADED 级: /tf 仍可用)
	JS_DROPOUT = 4,       // /joint_states 断流 (FAULT 级: 核心反馈丢失)
	// ---- BUS 域 (10-19, 真机批次) ----
	BUS_TIMEOUT = 10,     // 传输超时 (DEGRADED 级: 偶发看趋势)
	BUS_LINK_DOWN = 11,   // 链路断 (FAULT 级)
	// ---- DEVICE 域 (20-29 通用 / 30+ 协议特有, 真机批次) ----
	// (协议轴 error_bits 映射落此; 任一非零码 → FAULT, 聚合规则见 SystemAlertArray.msg)
	// ---- 跨域保留 ----
	DOMAIN_STALE = 0xFFFE,  // 该域源静默, 死活不明 (恒 FAULT 级)
};

// 每码标准人话 (单一事实源, supervisor 组装 message 用; 消费者可复用)
[[nodiscard]] constexpr const char * err_code_message(ErrCode c) noexcept
{
	switch (c)
	{
		case ErrCode::NONE: {return "正常";}
		case ErrCode::CM_DEGRADED: {return "CM 求解连续失败降级";}
		case ErrCode::CM_SILENT: {return "CM status 静默 (active 后)";}
		case ErrCode::EE_DROPOUT: {return "/ee_state 断流";}
		case ErrCode::JS_DROPOUT: {return "/joint_states 断流 (核心反馈)";}
		case ErrCode::BUS_TIMEOUT: {return "总线传输超时";}
		case ErrCode::BUS_LINK_DOWN: {return "总线链路断";}
		case ErrCode::DOMAIN_STALE: {return "域源静默";}
		default: {return "?";}
	}
}

// 码 → 系统聚合态严重度 (数值对齐 SystemAlertArray.STATE_*; 加码同步补分支)
//   1 = 仅贡献 STATE_DEGRADED / 3 = 贡献 STATE_FAULT / 0 = 不贡献 (正常)
[[nodiscard]] constexpr uint8_t err_code_severity(ErrCode c) noexcept
{
	switch (c)
	{
		case ErrCode::NONE: {return 0;}
		case ErrCode::CM_DEGRADED: {return 1;}
		case ErrCode::CM_SILENT: {return 1;}
		case ErrCode::EE_DROPOUT: {return 1;}
		case ErrCode::JS_DROPOUT: {return 3;}
		case ErrCode::BUS_TIMEOUT: {return 1;}
		case ErrCode::BUS_LINK_DOWN: {return 3;}
		case ErrCode::DOMAIN_STALE: {return 3;}
		default: {return 3;}   // 未知码保守按 FAULT 级 (宁误停不误跑)
	}
}

}  // namespace unistackbot_interface

#endif  // UNISTACKBOT_INTERFACE__ERROR_CODES_HPP_

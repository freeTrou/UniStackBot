#ifndef UNISTACKBOT_PROTOCOL__PROTOCOL_CORE_HPP_
#define UNISTACKBOT_PROTOCOL__PROTOCOL_CORE_HPP_

#include <cstddef>
#include <cstdint>

namespace unistackbot_protocol
{

// 中立节点命令模式 (协议原值由内芯翻译: kRun → unitree FOC=1 等)。
enum class NodeMode : std::uint8_t
{
	kStop = 0,
	kRun = 1,
};

// 节点命令 —— MIT 五元组 (输出端物理单位)。
// 转子侧换算在内芯内做, ratio (减速比) 由机器配置按节点注入 —— 内芯不持机器知识。
struct NodeCommand
{
	NodeMode mode = NodeMode::kStop;
	bool watchdog_enable = true;   // 固件命令看门狗位 (协议无此能力则内芯忽略)
	double tau = 0.0;              // 前馈力矩 [Nm]
	double speed = 0.0;            // 目标速度 [rad/s]
	double position = 0.0;         // 目标位置 [rad] (多圈)
	double kp = 0.0;               // 位置刚度 [Nm/rad]
	double kd = 0.0;               // 速度刚度 [Nm·s/rad]
};

// 节点反馈 —— 通用物理量 (输出端) + 遥测 + 原始错误码。
// 错误位码的人话化 (interpret 完整错误表) 为延后项: 先原样携带, FAULT 语义已够最小集。
struct NodeFeedback
{
	std::uint8_t node_id = 0;
	NodeMode mode = NodeMode::kStop;
	bool timeout_triggered = false;
	std::int8_t temp_driver = 0;     // 驱动温度 [°C]
	std::uint8_t temp_winding = 0;   // 绕组温度 [°C]
	double voltage = 0.0;            // [V]
	double tau = 0.0;                // [Nm]
	double speed = 0.0;              // [rad/s]
	double position = 0.0;           // [rad] 多圈
	std::uint32_t error_raw = 0;     // 协议位码原样
	std::uint8_t warning_raw = 0;
};

// RX 帧规格 —— Framer 所需的全部协议知识 (定长帧族;
// 长度域/静默间隔模式为延后项, 等 Dynamixel/Modbus 类协议)。
struct FrameSpec
{
	std::uint8_t header0 = 0;
	std::uint8_t header1 = 0;
	std::size_t frame_len = 0;
	std::size_t crc_offset = 0;        // 帧内 CRC 起点
	std::size_t crc_cover_offset = 0;  // CRC 覆盖区起点
	std::size_t crc_cover_len = 0;     // 覆盖长度
};

// 协议内芯最小接口 (纯 codec 轴, 3 方法; 三面的完整形状与延后清单见架构文档 §2.1/§2.3)。
// 纪律: 纯函数语义、零分配、零异常 —— encode/decode 均不 new/malloc, 供 RT 泵直呼。
class ProtocolCore
{
public:
	virtual ~ProtocolCore() = default;

	// Framer 要的元数据 (RX 反馈帧)
	[[nodiscard]] virtual FrameSpec framespec() const = 0;

	// 控制面: 命令 → 帧。返回写入字节数; buf 不足 / ratio 非法 / 任一值非有限 → 0。
	[[nodiscard]] virtual std::size_t encode(std::uint8_t *buf, std::size_t cap, const NodeCommand &cmd, std::uint8_t node_id, double ratio) const = 0;

	// 反馈面: 帧 → 反馈。长度/帧头/CRC 任一不符 → false (不抛不写半截)。
	[[nodiscard]] virtual bool decode(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio) const = 0;

};

// 注 (2026-09-24 拆包): 状态翻译移居 unistackbot_statemachine 包 (StateTranslator 轴,
// 同款父类+子类+工厂组织) —— 本包瘦身为纯 codec 轴 (字节 ↔ 物理量)。

}  // namespace unistackbot_protocol

#endif

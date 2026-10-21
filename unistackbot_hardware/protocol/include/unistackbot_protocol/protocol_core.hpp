#ifndef UNISTACKBOT_PROTOCOL__PROTOCOL_CORE_HPP_
#define UNISTACKBOT_PROTOCOL__PROTOCOL_CORE_HPP_

#include <cstddef>
#include <cstdint>

namespace unistackbot_protocol
{

// 中立节点命令模式 (协议原值由内芯翻译: kRun → unitree FOC=1 等)。
// kClear = 清故障 (2026-10-10 解除 special() 延后): unitree 语义 = mode 6 +
// tor_raw=-256 魔数 (内芯强制, 调用方只给模式)。厂商 encode 只支持 0/1,
// 此扩展为加性 (oracle 对拍不受影响)。
// kEnable = 显式使能 (2026-10-21 达妙接入加): dm_motor = 独立使能特殊帧 0xFC
// (失能态下 MIT 帧不生效, 使能必须是独立动作); unitree = kRun 同义 (使能即发
// MIT 首帧); cia402 = 使能链 (经 cmd_raw)。加性词汇, 老内芯各自解释。
enum class NodeMode : std::uint8_t
{
	kStop = 0,
	kRun = 1,
	kEnable = 2,
	kClear = 6,
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
	std::uint32_t cmd_raw = 0;     // 协议原生命令载荷 (细分格命令层, 2026-10-10 加):
	                               // CiA402=controlword 0x6040 原样 (多步使能链的每步);
	                               // unitree=忽略 (粗态三模式已够)。内芯各自定义解释。
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
	std::uint32_t state_raw = 0;     // 协议原生态载荷 (细分格层, 2026-10-10 加):
	                                 // CiA402=statusword 0x6041 原样 (官方七态全信息);
	                                 // unitree=0 (无此概念, 粗态已够)。0=无此概念约定。
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

// 节点类别 (2026-10-21 组合模型): 执行器=命令+反馈+状态机 (statemachine 配置必填);
// 传感器=只读无状态机 (statemachine 必空, 数据走数值槽, 实现挂起待第一个真实传感器)。
// 协议注册表自声明, bus 层按此做配置交叉校验与调度分槽 (命令槽 vs 只收槽)。
enum class ProtocolKind : std::uint8_t
{
	kActuator = 0,
	kSensor = 1,
};

// 协议内芯最小接口 (纯 codec 轴, 4 方法; 三面的完整形状与延后清单见架构文档 §2.1/§2.3)。
// 纪律: 纯函数语义、零分配、零异常 —— encode/decode 均不 new/malloc, 供 RT 泵直呼。
// 契约 (总线侧消费, 2026-10-21 文档化): 帧合法时 decode 必须正确填充 node_id,
// 且 node_id 的解码不依赖 ratio/量程档 —— 串口段的探路路由 (先 decode 取 id 定槽,
// 再按槽位用正确 ratio 重解码) 依赖此契约, 各内芯有测试硬化。
class ProtocolCore
{
public:
	virtual ~ProtocolCore() = default;

	// 节点类别 (组合模型 kind 交叉校验与调度分槽依据)
	[[nodiscard]] virtual ProtocolKind kind() const = 0;

	// ratio 语义声明 (2026-10-21 审查 P1): start 期 fail-fast 校验用——配置错误的
	// 减速比此前静默死节点 (encode 每帧拒收, 无诊断)。unitree: 任意 >0 (转子↔输出
	// 换算); dm_motor: 恒等 1.0 (减速比内含于量程, 非 1 = 配置错误)。
	[[nodiscard]] virtual bool ratio_valid(double ratio) const = 0;

	// 节点 id 域自声明 (2026-10-21 二轮审查: 宇树 4bit id 域 (≤14, 15=广播) 曾以
	// 硬编码检查泄漏进串口主站通用校验——协议知识进协议)。unitree=≤14;
	// dm=CAN 11bit 全域 (≤0x7FF)。
	[[nodiscard]] virtual bool id_valid(std::uint8_t node_id) const = 0;

	// Framer 要的元数据 (RX 反馈帧)
	[[nodiscard]] virtual FrameSpec framespec() const = 0;

	// 控制面: 命令 → 帧。返回写入字节数; buf 不足 / ratio 非法 / 任一值非有限 → 0。
	// **encode 返回 0 = 本拍无帧可发, 主站跳过该节点发送** (2026-10-21 契约) ——
	// 传感器协议 (decode-only) encode 恒返 0, 泵的调度循环由此对节点类型免疫
	// (主站稳定性契约: 不为传感器改主站)。执行器协议命令帧恒返定长。
	[[nodiscard]] virtual std::size_t encode(std::uint8_t *buf, std::size_t cap, const NodeCommand &cmd, std::uint8_t node_id, double ratio) const = 0;

	// 反馈面: 帧 → 反馈。长度/帧头/CRC 任一不符 → false (不抛不写半截)。
	[[nodiscard]] virtual bool decode(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio) const = 0;

};

// 注 (2026-09-24 拆包): 状态翻译移居 unistackbot_statemachine 包 (NodeSafety 安全机轴,
// 同款父类+子类+工厂组织) —— 本包瘦身为纯 codec 轴 (字节 ↔ 物理量)。

}  // namespace unistackbot_protocol

#endif

#ifndef UNISTACKBOT_PROTOCOL__UNITREE_IM_CORE_HPP_
#define UNISTACKBOT_PROTOCOL__UNITREE_IM_CORE_HPP_

#include <cstddef>
#include <cstdint>

#include "unistackbot_protocol/protocol_core.hpp"

// unitree_im —— 宇树 IM 系电机协议内芯 (第一个实现)。
//
// 依据: R1_7A 说明文档 (协议五节) + IM6014 C++ demo 源码逐行核实 (2026-09-24)。
// 帧格式 (三款电机 6010/6014/5010 通用; 减速比由调用方按节点注入):
//   命令 20B: FE EE | 模式字节(id4b+mode3b+timeout1b) | res | tau/spd int16 | pos int32
//             | kp/kd int16 | CRC32 (LE, 覆盖前 16B)
//   反馈 26B: FC EE (不入CRC) | 模式字节 | 双温度 | 电压(2LSB=1V) | tau/spd int16
//             | pos int32 | error u32 | 保留+警告 | res | CRC32 (LE, 覆盖 [2..21])
//   定点 (转子端): 2560=1Nm · 64/2π=1rad/s · 32768/2π=1rad · 12800=1Nm/rad · 51200=1Nm·s/rad
//   输出↔转子: tau/ratio · pos·ratio · spd·ratio · kp/kd·ratio²
//   CRC32: 多项式 0x04C11DB7, 初值 0xFFFFFFFF, 不反射, 无终异或 —— **宇树字序变体**:
//   按厂商参考实现, 每 4 字节按小端字加载、字内 MSB 先行 (即内存序 b3,b2,b1,b0)。
//
// 同族扩展位 (MotorType 概念, 参考 unitree_actuator_sdk): A1/B1/GO 为另一代帧格式,
// 届时在本包另立内芯, 不改本文件 —— 最小集纪律: 第二个消费者出现才动。
//
// 测试 golden: test/test_unitree_im.cpp 以厂商 demo 源码为甲骨文逐字节对拍。

namespace unistackbot_protocol
{
namespace unitree_im
{

constexpr std::uint8_t kCmdHeader0 = 0xFE;
constexpr std::uint8_t kCmdHeader1 = 0xEE;
constexpr std::uint8_t kFbHeader0 = 0xFC;
constexpr std::uint8_t kFbHeader1 = 0xEE;
constexpr std::size_t kCmdFrameLen = 20;
constexpr std::size_t kFbFrameLen = 26;
constexpr std::uint8_t kBroadcastId = 15;   // 广播 (无返回) —— special() 延后项, 先记常量

// CRC32 (宇树字序变体)。len 须为 4 的倍数 (本协议覆盖区 16/20B 均满足)。
[[nodiscard]] std::uint32_t crc32(const std::uint8_t *data, std::size_t len);

// 纯函数: 命令 → 20B 帧 (语义/失败条件同 ProtocolCore::encode)。
[[nodiscard]] std::size_t encodeFrame(std::uint8_t *buf, std::size_t cap,
	const NodeCommand &cmd, std::uint8_t node_id, double ratio);

// 纯函数: 26B 帧 → 反馈 (语义同 ProtocolCore::decode)。
[[nodiscard]] bool decodeFrame(const std::uint8_t *frame, std::size_t len,
	NodeFeedback &fb, double ratio);

}  // namespace unitree_im

// ProtocolCore 的 unitree_im 实现 (薄封装, 泵经接口多态调用; 测试直打纯函数)。
class UnitreeImCore final : public ProtocolCore
{
public:
	[[nodiscard]] FrameSpec framespec() const override;
	[[nodiscard]] std::size_t encode(std::uint8_t *buf, std::size_t cap, 
		const NodeCommand &cmd, std::uint8_t node_id, double ratio) const override;
	[[nodiscard]] bool decode(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio) const override;
};

}  // namespace unistackbot_protocol

#endif

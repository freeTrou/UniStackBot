#include "unistackbot_protocol/dm_motor_core.hpp"

#include <cmath>

namespace unistackbot_protocol
{
namespace
{

// 全幅定点换算 (手册+SDK 双源: 分母 (2^bits)-1, 先钳位后归一)
[[nodiscard]] std::uint16_t doubleToUint(double x, double x_min, double x_max, int bits)
{
	if (x < x_min)
	{
		x = x_min;
	}
	else if (x > x_max)
	{
		x = x_max;
	}
	const double span = x_max - x_min;
	const double norm = (x - x_min) / span;
	const double full = static_cast<double>((1u << bits) - 1u);
	return static_cast<std::uint16_t>(norm * full);
}

[[nodiscard]] double uintToDouble(std::uint16_t x, double x_min, double x_max, int bits)
{
	const double span = x_max - x_min;
	const double norm = static_cast<double>(x) / static_cast<double>((1u << bits) - 1u);
	return norm * span + x_min;
}

// 特殊命令帧: FF×7 + cmd (使能 FC / 失能 FD / 清故障 FB)
[[nodiscard]] std::size_t packSpecial(std::uint8_t *buf, std::uint8_t cmd)
{
	for (std::size_t i = 0; i < 7; ++i)
	{
		buf[i] = 0xFFu;
	}
	buf[7] = cmd;
	return dm_motor::kFrameLen;
}

}  // namespace

namespace dm_motor
{

LimitParam limits(DmMotorType type)
{
	switch (type)
	{
	case DmMotorType::kDm4310:
		return {12.5, 30.0, 10.0};
	case DmMotorType::kDm4340:
		return {12.5, 8.0, 28.0};
	}
	return {0.0, 0.0, 0.0};   // 不可达 (枚举全覆盖); 保险返回非法量程
}

std::size_t encodeFrame(std::uint8_t *buf, std::size_t cap, const NodeCommand &cmd, std::uint8_t /*node_id*/, double ratio, DmMotorType type)
{
	if (buf == nullptr || cap < kFrameLen || ratio != 1.0 || (type != DmMotorType::kDm4310 && type != DmMotorType::kDm4340))
	{
		return 0;
	}
	switch (cmd.mode)
	{
	case NodeMode::kEnable:
		return packSpecial(buf, 0xFCu);
	case NodeMode::kStop:
		return packSpecial(buf, 0xFDu);
	case NodeMode::kClear:
		return packSpecial(buf, 0xFBu);
	case NodeMode::kRun:
		break;   // MIT 帧在 switch 外装配 (字段多)
	default:
		return 0;
	}
	// 非有限输入一票拒绝 (codec 级第一道诚实性; write 层防线在后)
	if (!std::isfinite(cmd.tau) || !std::isfinite(cmd.speed) || !std::isfinite(cmd.position) || !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd))
	{
		return 0;
	}
	const LimitParam lim = limits(type);
	const std::uint16_t q_u = doubleToUint(cmd.position, -lim.pmax, lim.pmax, 16);
	const std::uint16_t dq_u = doubleToUint(cmd.speed, -lim.vmax, lim.vmax, 12);
	const std::uint16_t kp_u = doubleToUint(cmd.kp, 0.0, kKpMax, 12);
	const std::uint16_t kd_u = doubleToUint(cmd.kd, 0.0, kKdMax, 12);
	const std::uint16_t tau_u = doubleToUint(cmd.tau, -lim.tmax, lim.tmax, 12);
	// 位拼接 (手册 MIT 控制帧表逐位): D0=q_hi D1=q_lo D2=dq_hi D3=dq_lo|kp_hi
	// D4=kp_lo D5=kd_hi D6=kd_lo|tau_hi D7=tau_lo
	buf[0] = static_cast<std::uint8_t>((q_u >> 8) & 0xFFu);
	buf[1] = static_cast<std::uint8_t>(q_u & 0xFFu);
	buf[2] = static_cast<std::uint8_t>((dq_u >> 4) & 0xFFu);
	buf[3] = static_cast<std::uint8_t>(((dq_u & 0xFu) << 4) | ((kp_u >> 8) & 0xFu));
	buf[4] = static_cast<std::uint8_t>(kp_u & 0xFFu);
	buf[5] = static_cast<std::uint8_t>((kd_u >> 4) & 0xFFu);
	buf[6] = static_cast<std::uint8_t>(((kd_u & 0xFu) << 4) | ((tau_u >> 8) & 0xFu));
	buf[7] = static_cast<std::uint8_t>(tau_u & 0xFFu);
	return kFrameLen;
}

bool decodeFrame(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio, DmMotorType type)
{
	if (frame == nullptr || len != kFrameLen || ratio != 1.0 || (type != DmMotorType::kDm4310 && type != DmMotorType::kDm4340))
	{
		return false;
	}
	const std::uint8_t d0 = frame[0];
	const LimitParam lim = limits(type);
	const std::uint16_t q_u = (static_cast<std::uint16_t>(frame[1]) << 8) | frame[2];
	const std::uint16_t dq_u = (static_cast<std::uint16_t>(frame[3]) << 4) | (frame[4] >> 4);
	const std::uint16_t tau_u = (static_cast<std::uint16_t>(frame[4] & 0xFu) << 8) | frame[5];
	fb.node_id = static_cast<std::uint8_t>(d0 & 0x0Fu);
	fb.position = uintToDouble(q_u, -lim.pmax, lim.pmax, 16);
	fb.speed = uintToDouble(dq_u, -lim.vmax, lim.vmax, 12);
	fb.tau = uintToDouble(tau_u, -lim.tmax, lim.tmax, 12);
	fb.temp_driver = static_cast<std::int8_t>(frame[6]);    // T_MOS 直读℃
	fb.temp_winding = static_cast<std::uint8_t>(frame[7]);  // T_Rotor 直读℃
	fb.error_raw = static_cast<std::uint32_t>((d0 >> 4) & 0x0Fu);   // ERR 半字节 (0/1/3-5/8-E)
	fb.state_raw = d0;   // D0 原样 (ID|ERR 细分格层全信息)
	fb.mode = NodeMode::kStop;   // 协议无模式反馈, 保持默认 (状态信息在 error_raw)
	fb.voltage = 0.0;            // 协议无电压反馈
	fb.timeout_triggered = false;   // 看门狗触发表现为 ERR=D (通讯丢失) → FAULT, 无独立位
	return true;
}

}  // namespace dm_motor

DmMotorCore::DmMotorCore(dm_motor::DmMotorType type) : type_(type) {}

FrameSpec DmMotorCore::framespec() const
{
	// CAN 协议无帧头/CRC (寻址住 CAN ID, 校验住链路层) —— Framer 概念不适用,
	// 仅报帧长 (SerialMaster 之外的 CAN 主站不经 Framer)
	FrameSpec spec;
	spec.frame_len = dm_motor::kFrameLen;
	return spec;
}

std::size_t DmMotorCore::encode(std::uint8_t *buf, std::size_t cap, const NodeCommand &cmd, std::uint8_t node_id, double ratio) const
{
	return dm_motor::encodeFrame(buf, cap, cmd, node_id, ratio, type_);
}

bool DmMotorCore::decode(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio) const
{
	return dm_motor::decodeFrame(frame, len, fb, ratio, type_);
}

}  // namespace unistackbot_protocol

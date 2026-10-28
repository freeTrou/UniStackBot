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
	/*
	 kd=0 手册明示禁止 (位置控制震荡/失控, damiao_protocol.md §4.1) —— 协议知识住协议
	 (2026-10-27 裁决); kRun 且 kp>0 时 kd 必须为正 (纯力矩前馈 tau-only 帧 kp=kd=0 合法)
	*/
	if (cmd.mode == NodeMode::kRun && cmd.kp > 0.0 && cmd.kd <= 0.0)
	{
		return 0;
	}
	const LimitParam lim = limits(type);
	const std::uint16_t q_u = doubleToUint(cmd.position, -lim.pmax, lim.pmax, 16);
	const std::uint16_t dq_u = doubleToUint(cmd.speed, -lim.vmax, lim.vmax, 12);
	const std::uint16_t kp_u = doubleToUint(cmd.kp, 0.0, kKpMax, 12);
	const std::uint16_t kd_u = doubleToUint(cmd.kd, 0.0, kKdMax, 12);
	const std::uint16_t tau_u = doubleToUint(cmd.tau, -lim.tmax, lim.tmax, 12);
	// kd 量化后复检 (2026-10-28 审核修: 浮点 kd∈(0, 1LSB≈1.2e-3) 通过上面的检查但
	// 编码为 kd 半字节 0——手册禁令在量化后验证才闭环; tau-only 帧 kp_u=0 豁免)
	if (cmd.mode == NodeMode::kRun && kp_u != 0u && kd_u == 0u)
	{
		return 0;
	}
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

bool unpackCommand(const std::uint8_t *frame, std::size_t len, NodeCommand &cmd, double ratio, DmMotorType type)
{
	if (frame == nullptr || len != kFrameLen || ratio != 1.0 || (type != DmMotorType::kDm4310 && type != DmMotorType::kDm4340))
	{
		return false;
	}
	// 特殊帧还原: FF×7 + cmd → 模式命令 (与 encodeFrame 的 packSpecial 对偶)。
	// 顶格歧义消解 (2026-10-28 审核修): 合法 MIT 命令全字段饱和时也产出 7×FF 开头
	// (q=pmax,dq=vmax,kp=500,kd=5 全编码 0xFFF/0xFFFF)——byte7 ∈ {FC,FD,FB} 且
	// byte1=q 低字节也是 FF 时仍可能撞车, 但真特殊帧 byte1..6 恒 FF 而 MIT 帧的
	// byte4=kp 低字节 = FF 仅当 kp=500 顶格。判据: 特殊帧候选 + 全 8 字节 FF 的
	// MIT 顶格命令 (q+/dq+/kp/kd/tau 全饱和) 按 MIT 解 (协议侧三特殊命令 byte7
	// 恒为 FC/FD/FB, 而 MIT 顶格 tau=+tmax 时 byte7=FF——冲突仅在 byte7 恰为
	// FC/FD/FB 的部分顶格, 此帧按 MIT 解是更安全的选择: 电机端同样先嗅探)
	bool special = true;
	for (std::size_t i = 0; i < 7; ++i)
	{
		special = special && frame[i] == 0xFFu;
	}
	if (special)
	{
		switch (frame[7])
		{
		case 0xFCu:
		case 0xFDu:
		case 0xFBu:
			// 与 MIT 顶格的碰撞消解: byte0=q 高字节 = FF 要求 q ≥ pmax−1LSB, 且
			// byte7 作为 tau 低字节 = FC/FD/FB 要求 tau 接近 ±tmax——同时满足时
			// 按协议嗅探优先 (与厂商 SDK 行为一致: 特殊命令是显式意图)
			cmd.mode = frame[7] == 0xFCu ? NodeMode::kEnable :
				(frame[7] == 0xFDu ? NodeMode::kStop : NodeMode::kClear);
			return true;
		case 0xFFu:
			// 全 FF = MIT 顶格 (q+,dq+,kp,kd,tau 全饱和)——按 MIT 解
			break;
		default:
			return false;   // 未知特殊命令 (0x33/0x55/0xAA/0xFE 暂缓范围外)
		}
	}
	// MIT 帧解位拼接 (encodeFrame 位序对偶)
	cmd.mode = NodeMode::kRun;
	const LimitParam lim = limits(type);
	const std::uint16_t q_u = (static_cast<std::uint16_t>(frame[0]) << 8) | frame[1];
	const std::uint16_t dq_u = (static_cast<std::uint16_t>(frame[2]) << 4) | (frame[3] >> 4);
	const std::uint16_t kp_u = (static_cast<std::uint16_t>(frame[3] & 0xFu) << 8) | frame[4];
	const std::uint16_t kd_u = (static_cast<std::uint16_t>(frame[5]) << 4) | (frame[6] >> 4);
	const std::uint16_t tau_u = (static_cast<std::uint16_t>(frame[6] & 0xFu) << 8) | frame[7];
	cmd.position = uintToDouble(q_u, -lim.pmax, lim.pmax, 16);
	cmd.speed = uintToDouble(dq_u, -lim.vmax, lim.vmax, 12);
	cmd.kp = uintToDouble(kp_u, 0.0, kKpMax, 12);
	cmd.kd = uintToDouble(kd_u, 0.0, kKdMax, 12);
	cmd.tau = uintToDouble(tau_u, -lim.tmax, lim.tmax, 12);
	return true;
}

std::size_t packFeedback(std::uint8_t *buf, std::size_t cap, const NodeFeedback &fb, double ratio, DmMotorType type)
{
	if (buf == nullptr || cap < kFrameLen || ratio != 1.0 || (type != DmMotorType::kDm4310 && type != DmMotorType::kDm4340))
	{
		return 0;
	}
	if (!std::isfinite(fb.position) || !std::isfinite(fb.speed) || !std::isfinite(fb.tau))
	{
		return 0;
	}
	const LimitParam lim = limits(type);
	// D0 = (ERR<<4)|ID (ERR 半字节截断, ID 低 4 位截断 —— 反馈帧协议形态)
	buf[0] = static_cast<std::uint8_t>(((fb.error_raw & 0x0Fu) << 4) | (fb.node_id & 0x0Fu));
	const std::uint16_t q_u = doubleToUint(fb.position, -lim.pmax, lim.pmax, 16);
	const std::uint16_t dq_u = doubleToUint(fb.speed, -lim.vmax, lim.vmax, 12);
	const std::uint16_t tau_u = doubleToUint(fb.tau, -lim.tmax, lim.tmax, 12);
	buf[1] = static_cast<std::uint8_t>((q_u >> 8) & 0xFFu);
	buf[2] = static_cast<std::uint8_t>(q_u & 0xFFu);
	buf[3] = static_cast<std::uint8_t>((dq_u >> 4) & 0xFFu);
	buf[4] = static_cast<std::uint8_t>(((dq_u & 0xFu) << 4) | ((tau_u >> 8) & 0xFu));
	buf[5] = static_cast<std::uint8_t>(tau_u & 0xFFu);
	// 双温度直读截断 (int8 MOS / uint8 Rotor)
	buf[6] = static_cast<std::uint8_t>(static_cast<std::int8_t>(fb.temp_driver));
	buf[7] = static_cast<std::uint8_t>(fb.temp_winding);
	return kFrameLen;
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

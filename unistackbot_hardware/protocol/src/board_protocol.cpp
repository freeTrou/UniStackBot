#include "unistackbot_protocol/board_protocol.hpp"

#include <cmath>

namespace unistackbot_protocol
{

bool encodeControl(const ControlFrameBits & in, uint8_t * out, size_t len)
{
	if (out == nullptr || len != kControlFrameSize)
	{
		return false;
	}
	ControlFrameBits f = in;
	f.id = kControlFrameId;   // 帧头 id 由编码器统一落章, 调用方不负责
	std::memcpy(out, &f, sizeof(f));
	return true;
}

bool decodeControl(const uint8_t * buf, size_t len, ControlFrameBits & out)
{
	if (buf == nullptr || len != kControlFrameSize || buf[1] != kControlFrameId)
	{
		return false;
	}
	std::memcpy(&out, buf, sizeof(out));
	return true;
}

bool encodeResponse(const ResponseFrameBits & in, uint8_t * out, size_t len)
{
	if (out == nullptr || len != kResponseFrameSize)
	{
		return false;
	}
	ResponseFrameBits f = in;
	f.id = kResponseFrameId;
	std::memcpy(out, &f, sizeof(f));
	return true;
}

bool decodeResponse(const uint8_t * buf, size_t len, ResponseFrameBits & out)
{
	if (buf == nullptr || len != kResponseFrameSize || buf[1] != kResponseFrameId)
	{
		return false;
	}
	std::memcpy(&out, buf, sizeof(out));
	return true;
}

const MotorCmdBits * chassisCmdMotor(const ControlFrameBits & f, size_t i)
{
	return i < kChassisMotorCount ? &f.bus_chassis.motor[i] : nullptr;
}

const MotorStateChassisBits * chassisStateMotor(const ResponseFrameBits & f, size_t i)
{
	return i < kChassisMotorCount ? &f.bus_chassis.motor[i] : nullptr;
}

bool encodeBoardControl(const NodeCommand * cmds, std::size_t n, uint8_t * out, std::size_t cap, uint8_t ctrl_word)
{
	constexpr std::size_t kMaxSlots = kArmBusCount * kArmMotorsPerBus + kChassisMotorCount;
	if (cmds == nullptr || out == nullptr || cap < kControlFrameSize || n > kMaxSlots)
	{
		return false;
	}
	for (std::size_t i = 0; i < n; ++i)
	{
		if (!std::isfinite(cmds[i].position) || !std::isfinite(cmds[i].speed))
		{
			return false;   // 非有限值一票拒收 (与 unitree encode 同纪律)
		}
	}
	constexpr std::size_t kArmSlots = kArmBusCount * kArmMotorsPerBus;
	ControlFrameBits f{};
	f.ts = 0;                  // 时戳由主站发送侧填 (板按帧到达节拍工作)
	f.id = kControlFrameId;
	f.bus_chassis.ctrl_word = ctrl_word;   // 底盘槽 ctrl_word 同臂总线 (语义 M4)
	for (std::size_t i = 0; i < n; ++i)
	{
		MotorCmdBits & m = i < kArmSlots
			? f.bus[i / kArmMotorsPerBus].motor[i % kArmMotorsPerBus]
			: f.bus_chassis.motor[i - kArmSlots];
		const bool enable = cmds[i].mode == NodeMode::kRun || cmds[i].mode == NodeMode::kEnable;
		m.enable = enable ? 1 : 0;
		m.tar_pos = static_cast<std::int16_t>(cmds[i].position * kBoardScale);
		m.tar_vel = static_cast<std::int16_t>(cmds[i].speed * kBoardScale);
		m.tar_tor = 0;   // tau 定标无实录证据 (M4 HAL yaml 对齐后开)
		m.kp = 0;
		m.kd_ki = 0;
	}
	for (std::size_t b = 0; b < kArmBusCount; ++b)
	{
		f.bus[b].ctrl_word = ctrl_word;
	}
	std::memcpy(out, &f, sizeof(f));
	return true;
}

bool decodeBoardResponse(const uint8_t * buf, std::size_t len, NodeFeedback * fbs, std::size_t n)
{
	constexpr std::size_t kMaxSlots = kArmBusCount * kArmMotorsPerBus + kChassisMotorCount;
	ResponseFrameBits f{};
	if (!decodeResponse(buf, len, f) || fbs == nullptr || n > kMaxSlots)
	{
		return false;
	}
	constexpr std::size_t kArmSlots = kArmBusCount * kArmMotorsPerBus;
	for (std::size_t i = 0; i < n; ++i)
	{
		NodeFeedback & fb = fbs[i];
		fb.node_id = static_cast<std::uint8_t>(i);
		if (i >= kArmSlots)
		{
			// 底盘槽 (MotorStateChassisBits): 定标/语义 M4 真板批次, 暂同臂槽 ×1000 口径
			const MotorStateChassisBits & m = f.bus_chassis.motor[i - kArmSlots];
			fb.position = static_cast<double>(m.cur_pos) / kBoardScale;
			fb.speed = static_cast<double>(m.cur_vel) / kBoardScale;
			fb.error_raw = m.error;
			fb.state_raw = m.status;
			continue;
		}
		const MotorStateBits & m = f.bus[i / kArmMotorsPerBus].motor[i % kArmMotorsPerBus];
		fb.position = static_cast<double>(m.cur_pos) / kBoardScale;
		fb.speed = static_cast<double>(m.cur_vel) / kBoardScale;
		fb.tau = 0.0;                                   // 定标未开 (编码侧对称)
		fb.error_raw = m.error;
		fb.state_raw = (static_cast<std::uint32_t>(m.enable ? 1 : 0) << 8) | m.status;
		fb.mode = m.enable ? NodeMode::kRun : NodeMode::kStop;
	}
	return true;
}

}  // namespace unistackbot_protocol

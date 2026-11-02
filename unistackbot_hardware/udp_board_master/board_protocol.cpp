#include "unistackbot_udp_board/board_protocol.hpp"

namespace unistackbot_udp_board
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

}  // namespace unistackbot_udp_board

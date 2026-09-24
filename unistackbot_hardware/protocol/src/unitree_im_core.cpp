#include "unistackbot_protocol/unitree_im_core.hpp"

#include <cmath>
#include <cstring>

namespace unistackbot_protocol
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint32_t kCrcPoly = 0x04C11DB7u;
constexpr double kInt16MaxD = 32767.0;
constexpr double kInt16MinD = -32768.0;
constexpr double kInt32MaxD = 2147483647.0;
constexpr double kInt32MinD = -2147483648.0;

// 小端读写 (手工字节装配, 无对齐/别名问题)
void putU16(std::uint8_t *p, std::uint16_t v)
{
	p[0] = static_cast<std::uint8_t>(v & 0xFFu);
	p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
}

void putU32(std::uint8_t *p, std::uint32_t v)
{
	p[0] = static_cast<std::uint8_t>(v & 0xFFu);
	p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
	p[2] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
	p[3] = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
}

[[nodiscard]] std::uint16_t getU16(const std::uint8_t *p)
{
	return static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[0]) |
		static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[1]) << 8));
}

[[nodiscard]] std::uint32_t getU32(const std::uint8_t *p)
{
	return static_cast<std::uint32_t>(p[0]) |
		(static_cast<std::uint32_t>(p[1]) << 8) |
		(static_cast<std::uint32_t>(p[2]) << 16) |
		(static_cast<std::uint32_t>(p[3]) << 24);
}

// 定点钳位取整 (调用方已保证有限值)
[[nodiscard]] std::int16_t quantizeI16(double raw)
{
	if (raw > kInt16MaxD)
	{
		raw = kInt16MaxD;
	}
	else if (raw < kInt16MinD)
	{
		raw = kInt16MinD;
	}
	return static_cast<std::int16_t>(std::lround(raw));
}

[[nodiscard]] std::int32_t quantizeI32(double raw)
{
	if (raw > kInt32MaxD)
	{
		raw = kInt32MaxD;
	}
	else if (raw < kInt32MinD)
	{
		raw = kInt32MinD;
	}
	return static_cast<std::int32_t>(std::llround(raw));
}

}  // namespace

namespace unitree_im
{

std::uint32_t crc32(const std::uint8_t *data, std::size_t len)
{
	// 厂商参考实现的字序: 每 4 字节按小端字加载, 字内 bit31→bit0 (即内存序 b3..b0, MSB 先行)。
	// 本协议两个覆盖区 (命令 16B / 反馈 20B) 均为 4 的倍数, 无尾字节问题。
	std::uint32_t crc = 0xFFFFFFFFu;
	for (std::size_t off = 0; off + 4 <= len; off += 4)
	{
		const std::uint32_t word = getU32(data + off);
		for (int bit = 31; bit >= 0; --bit)
		{
			const bool crc_msb = (crc & 0x80000000u) != 0u;
			crc <<= 1;
			if (crc_msb)
			{
				crc ^= kCrcPoly;
			}
			if (((word >> bit) & 1u) != 0u)
			{
				crc ^= kCrcPoly;
			}
		}
	}
	return crc;
}

std::size_t encodeFrame(std::uint8_t *buf, std::size_t cap,
	const NodeCommand &cmd, std::uint8_t node_id, double ratio)
{
	if (buf == nullptr || cap < kCmdFrameLen || ratio <= 0.0 ||
		node_id > 14)   // 15=广播, special() 延后项 —— 周期命令路径只允许单播 0..14
	{
		return 0;
	}
	// 非有限输入一票拒绝 (codec 级第一道诚实性; write 层防线在后)
	if (!std::isfinite(cmd.tau) || !std::isfinite(cmd.speed) ||
		!std::isfinite(cmd.position) || !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd))
	{
		return 0;
	}

	const double r2 = ratio * ratio;
	const std::uint8_t mode_byte = static_cast<std::uint8_t>(
		(node_id & 0x0Fu) |
		((cmd.mode == NodeMode::kRun ? 1u : 0u) << 4) |
		(cmd.watchdog_enable ? 0x80u : 0x00u));

	// 输出端 → 转子定点 (协议系数)
	const std::int16_t tau_raw = quantizeI16(cmd.tau / ratio * 2560.0);
	const std::int16_t spd_raw = quantizeI16(cmd.speed * ratio * 64.0 / (2.0 * kPi));
	const std::int32_t pos_raw = quantizeI32(cmd.position * ratio * 32768.0 / (2.0 * kPi));
	const std::int16_t kp_raw = quantizeI16(cmd.kp / r2 * 12800.0);
	const std::int16_t kd_raw = quantizeI16(cmd.kd / r2 * 51200.0);

	buf[0] = kCmdHeader0;
	buf[1] = kCmdHeader1;
	buf[2] = mode_byte;
	buf[3] = 0x00;   // 保留, 固定 0
	putU16(buf + 4, static_cast<std::uint16_t>(tau_raw));
	putU16(buf + 6, static_cast<std::uint16_t>(spd_raw));
	putU32(buf + 8, static_cast<std::uint32_t>(pos_raw));
	putU16(buf + 12, static_cast<std::uint16_t>(kp_raw));
	putU16(buf + 14, static_cast<std::uint16_t>(kd_raw));
	putU32(buf + 16, crc32(buf, 16));
	return kCmdFrameLen;
}

bool decodeFrame(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio)
{
	if (frame == nullptr || len != kFbFrameLen || ratio <= 0.0)
	{
		return false;
	}
	if (frame[0] != kFbHeader0 || frame[1] != kFbHeader1)
	{
		return false;
	}
	if (getU32(frame + 22) != crc32(frame + 2, 20))
	{
		return false;
	}

	const std::uint8_t mode_byte = frame[2];
	fb.node_id = static_cast<std::uint8_t>(mode_byte & 0x0Fu);
	fb.mode = ((mode_byte >> 4) & 0x07u) == 1u ? NodeMode::kRun : NodeMode::kStop;
	fb.timeout_triggered = (mode_byte >> 7) != 0u;
	fb.temp_driver = static_cast<std::int8_t>(frame[3]);
	fb.temp_winding = frame[4];
	fb.voltage = static_cast<double>(frame[5]) / 2.0;
	const std::int16_t tau_raw = static_cast<std::int16_t>(getU16(frame + 6));
	const std::int16_t spd_raw = static_cast<std::int16_t>(getU16(frame + 8));
	const std::int32_t pos_raw = static_cast<std::int32_t>(getU32(frame + 10));
	fb.tau = static_cast<double>(tau_raw) / 2560.0 * ratio;
	fb.speed = static_cast<double>(spd_raw) * 2.0 * kPi / (64.0 * ratio);
	fb.position = static_cast<double>(pos_raw) * 2.0 * kPi / (32768.0 * ratio);
	fb.error_raw = getU32(frame + 14);
	const std::uint16_t res_exflag = getU16(frame + 18);
	fb.warning_raw = static_cast<std::uint8_t>((res_exflag >> 13) & 0x07u);
	return true;
}

}  // namespace unitree_im

FrameSpec UnitreeImCore::framespec() const
{
	FrameSpec spec;
	spec.header0 = unitree_im::kFbHeader0;
	spec.header1 = unitree_im::kFbHeader1;
	spec.frame_len = unitree_im::kFbFrameLen;
	spec.crc_offset = 22;
	spec.crc_cover_offset = 2;
	spec.crc_cover_len = 20;
	return spec;
}

std::size_t UnitreeImCore::encode(std::uint8_t *buf, std::size_t cap,
	const NodeCommand &cmd, std::uint8_t node_id, double ratio) const
{
	return unitree_im::encodeFrame(buf, cap, cmd, node_id, ratio);
}

bool UnitreeImCore::decode(const std::uint8_t *frame, std::size_t len,
	NodeFeedback &fb, double ratio) const
{
	return unitree_im::decodeFrame(frame, len, fb, ratio);
}

}  // namespace unistackbot_protocol

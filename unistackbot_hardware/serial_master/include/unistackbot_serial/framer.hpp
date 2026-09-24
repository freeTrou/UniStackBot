#ifndef UNISTACKBOT_SERIAL__FRAMER_HPP_
#define UNISTACKBOT_SERIAL__FRAMER_HPP_

#include <cstddef>
#include <cstdint>

#include "unistackbot_protocol/protocol_core.hpp"

// Framer —— 字节流 → 帧提取 (定长帧族; 长度域/静默间隔模式为延后项)。
// 职责: 帧头同步 + 定长切帧 + 跨调用拼接 (USB 攒批决定的常态) + 坏帧向前重同步。
// CRC 校验归 decode (定长帧无重同步歧义: 帧头已定, 消费恰 frame_len 字节;
// 坏 CRC 帧由 decode 拒收计数)。回显免疫: 命令帧头 (FE EE) 与反馈帧头 (FC EE)
// 首字节不同, 盲找反馈帧头天然跳过自发回显。

namespace unistackbot_serial
{

constexpr std::size_t kMaxFrameLen = 64;   // 反馈帧上界 (unitree 26B, 留余量)

struct ExtractedFrame
{
	std::uint8_t bytes[kMaxFrameLen] = {};
	std::size_t len = 0;
};

class Framer
{
public:
	explicit Framer(const unistackbot_protocol::FrameSpec &spec);

	// 喂字节流, 吐完整帧到 out (至多 max_out 帧), 返回吐出帧数。
	// 残缺帧留内部缓冲等下批; 纯噪声只保留可能是帧头首字节的 1 字节。
	std::size_t push(const std::uint8_t *data, std::size_t n,
		ExtractedFrame *out, std::size_t max_out);

	// 丢弃内部缓冲 (段复位用)
	void reset();

private:
	void compact();

	unistackbot_protocol::FrameSpec spec_;
	std::uint8_t buf_[kMaxFrameLen * 4] = {};
	std::size_t len_ = 0;
};

}  // namespace unistackbot_serial

#endif

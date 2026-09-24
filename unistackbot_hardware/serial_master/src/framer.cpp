#include "unistackbot_serial/framer.hpp"

#include <cstring>

namespace unistackbot_serial
{

Framer::Framer(const unistackbot_protocol::FrameSpec &spec) : spec_(spec) {}

void Framer::reset()
{
	len_ = 0;
}

void Framer::compact()
{
	// 缓冲满且无帧: 保底保留末 1 字节 (可能是帧头首字节), 其余丢弃
	if (len_ > sizeof(buf_) - kMaxFrameLen)
	{
		const std::uint8_t last = buf_[len_ - 1];
		buf_[0] = last;
		len_ = 1;
	}
}

std::size_t Framer::push(const std::uint8_t *data, std::size_t n,
	ExtractedFrame *out, std::size_t max_out)
{
	std::size_t produced = 0;
	// 追加输入 (超过容量截断——防呆, 正常水位远低于界)
	for (std::size_t i = 0; i < n && len_ < sizeof(buf_); ++i)
	{
		buf_[len_++] = data[i];
	}

	std::size_t scan = 0;
	while (produced < max_out)
	{
		// 找帧头
		bool found = false;
		while (scan + 1 < len_)
		{
			if (buf_[scan] == spec_.header0 && buf_[scan + 1] == spec_.header1)
			{
				found = true;
				break;
			}
			++scan;
		}
		if (!found)
		{
			// 无帧头: 保留末 1 字节 (可能为帧头首字节), 丢弃其余
			const std::uint8_t last = (len_ > 0) ? buf_[len_ - 1] : 0;
			buf_[0] = last;
			len_ = (len_ > 0) ? 1 : 0;
			return produced;
		}
		// 帧头前有垃圾: 丢弃
		if (scan > 0)
		{
			std::memmove(buf_, buf_ + scan, len_ - scan);
			len_ -= scan;
			scan = 0;
		}
		// 帧不完整: 等下批
		if (len_ < spec_.frame_len)
		{
			compact();
			return produced;
		}
		// 吐帧
		out[produced].len = spec_.frame_len;
		std::memcpy(out[produced].bytes, buf_, spec_.frame_len);
		++produced;
		std::memmove(buf_, buf_ + spec_.frame_len, len_ - spec_.frame_len);
		len_ -= spec_.frame_len;
	}
	compact();
	return produced;
}

}  // namespace unistackbot_serial

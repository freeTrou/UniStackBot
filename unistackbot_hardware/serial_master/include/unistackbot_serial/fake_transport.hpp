#ifndef UNISTACKBOT_SERIAL__FAKE_TRANSPORT_HPP_
#define UNISTACKBOT_SERIAL__FAKE_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>

#include "unistackbot_serial/io_transport.hpp"

// FakeTransport —— 假总线 (测试设施, 不进生产链路): 写入的字节交给 responder
// (假电机: 解析命令帧 → 返回应答字节), 应答入 RX 队列供 read 排干。
// 故障注入旋钮: 丢应答 / 破坏下一帧一字节 / 分片应答 (测 framer 跨拍拼接)。
// 这是 P14 "fake master 是接口的第一实现" 的镜像——fake slave 让泵全链零硬件可测。

namespace unistackbot_serial
{

class FakeTransport final : public IoTransport
{
public:
	// responder: 输入本批写入字节, 向 out 缓冲产出应答字节, 返回应答字节数
	using Responder = std::function<std::size_t(const std::uint8_t *, std::size_t,
		std::uint8_t *)>;

	explicit FakeTransport(Responder responder) : responder_(std::move(responder)) {}

	// ── 故障注入旋钮 ──
	void set_drop_responses(std::size_t n) { drop_remaining_ = n; }
	void set_corrupt_next_response() { corrupt_next_ = true; }
	void set_fragment_next_response(std::size_t first_chunk)
	{
		fragment_at_ = first_chunk;
	}

	// ── 统计 (测试断言用) ──
	[[nodiscard]] std::uint64_t write_calls() const { return write_calls_; }
	[[nodiscard]] std::uint64_t bytes_written() const { return bytes_written_; }

	ssize_t read(std::uint8_t *buf, std::size_t cap) override
	{
		std::size_t n = 0;
		while (n < cap && !rx_.empty())
		{
			buf[n++] = rx_.front();
			rx_.pop_front();
		}
		return static_cast<ssize_t>(n);
	}

	ssize_t write(const std::uint8_t *buf, std::size_t n) override
	{
		++write_calls_;
		bytes_written_ += n;
		std::uint8_t resp[512];
		const std::size_t r = responder_(buf, n, resp);
		if (r == 0 || drop_remaining_ > 0)
		{
			if (drop_remaining_ > 0)
			{
				--drop_remaining_;
			}
			return static_cast<ssize_t>(n);
		}
		append_response(resp, r);
		return static_cast<ssize_t>(n);
	}

	[[nodiscard]] std::size_t bytes_available() const override { return rx_.size(); }

	void close() override
	{
		rx_.clear();
		closed_ = true;
	}

	[[nodiscard]] bool closed() const { return closed_; }

private:
	void append_response(const std::uint8_t *data, std::size_t n)
	{
		std::size_t emit = n;
		if (fragment_at_ > 0 && fragment_at_ < n)
		{
			emit = fragment_at_;
			fragment_at_ = 0;
			pending_.assign(data + emit, data + n);
		}
		else if (fragment_at_ > 0)
		{
			fragment_at_ = 0;
		}
		if (corrupt_next_ && emit > 3)
		{
			corrupt_next_ = false;
			rx_.push_back(data[0]);
			rx_.push_back(data[1]);
			rx_.push_back(static_cast<std::uint8_t>(data[2] ^ 0x5Au));   // 破坏载荷首字节
			for (std::size_t i = 3; i < emit; ++i)
			{
				rx_.push_back(data[i]);
			}
			return;
		}
		for (std::size_t i = 0; i < emit; ++i)
		{
			rx_.push_back(data[i]);
		}
		if (!pending_.empty())
		{
			for (const std::uint8_t b : pending_)
			{
				rx_.push_back(b);
			}
			pending_.clear();
		}
	}

	Responder responder_;
	std::deque<std::uint8_t> rx_;
	std::vector<std::uint8_t> pending_;
	std::size_t drop_remaining_ = 0;
	bool corrupt_next_ = false;
	std::size_t fragment_at_ = 0;
	bool closed_ = false;
	std::uint64_t write_calls_ = 0;
	std::uint64_t bytes_written_ = 0;
};

}  // namespace unistackbot_serial

#endif

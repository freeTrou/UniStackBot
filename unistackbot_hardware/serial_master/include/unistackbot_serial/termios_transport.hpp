#ifndef UNISTACKBOT_SERIAL__TERMIOS_TRANSPORT_HPP_
#define UNISTACKBOT_SERIAL__TERMIOS_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "unistackbot_serial/io_transport.hpp"

// TermiosTransport —— Linux 真串口 (RS-485 适配器的字节管道)。
// 全部 Linux 坑封死在此 (设计文档 rs485_master_design.md §2):
//   8N1 raw · BOTHER 任意波特 (6M 不在标准 B 常量表) · O_NONBLOCK · TIOCEXCL 独占
//   · FIONREAD 排干辅助 · by-id 设备路径由调用方传入。
// 真机到货后补: FTDI latency_timer 自调 / 回显检测 (接口不变, 内部事)。

namespace unistackbot_serial
{

class TermiosTransport final : public IoTransport
{
public:
	// 打开并配置串口; 失败置 err 并返回 nullptr (显式错误流, 零异常)
	[[nodiscard]] static std::unique_ptr<TermiosTransport> open(const std::string &device,
		int baudrate, std::string &err);

	~TermiosTransport() override;

	ssize_t read(std::uint8_t *buf, std::size_t cap) override;
	ssize_t write(const std::uint8_t *buf, std::size_t n) override;
	[[nodiscard]] std::size_t bytes_available() const override;
	void close() override;

private:
	TermiosTransport() = default;
	int fd_ = -1;
};

}  // namespace unistackbot_serial

#endif

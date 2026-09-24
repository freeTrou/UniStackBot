#ifndef UNISTACKBOT_SERIAL__IO_TRANSPORT_HPP_
#define UNISTACKBOT_SERIAL__IO_TRANSPORT_HPP_

#include <cstddef>

// IoTransport —— 串口骨架的传输层接口 (只管字节, 协议词汇零出现)。
// 全部非阻塞语义: read 有多少读多少 (无数据返回 0), write 全量尽力 (返回写入数)。
// 实现: TermiosTransport (真串口) / FakeTransport (测试/假从站)。
// 参考: DynamixelSDK PortHandler (timeout 归 transport 的先例——本最小集不引入超时,
// 时序归泵线程)。

namespace unistackbot_serial
{

class IoTransport
{
public:
	virtual ~IoTransport() = default;

	// 非阻塞读: 返回读到的字节数 (0=无数据), 出错返回 -1
	[[nodiscard]] virtual ssize_t read(std::uint8_t *buf, std::size_t cap) = 0;
	// 写: 返回写入字节数, 出错返回 -1 (部分写由调用方检查重试——泵侧单帧 20B)
	[[nodiscard]] virtual ssize_t write(const std::uint8_t *buf, std::size_t n) = 0;
	// 可读字节数 (排干读辅助)
	[[nodiscard]] virtual std::size_t bytes_available() const = 0;
	// 关闭并释放 (幂等)
	virtual void close() = 0;
};

}  // namespace unistackbot_serial

#endif

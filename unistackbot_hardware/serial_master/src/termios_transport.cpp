#include "unistackbot_serial/termios_transport.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// BOTHER 任意波特率: 宏与 struct 全部本地定义 (kernel ABI 稳定值)——
// glibc <termios.h> 与 kernel 头 (linux/termios.h, asm/termbits.h) 的 struct termios
// 双双撞名, 经典解法 = 一个都不 include, 手工复刻 ioctl 所需最小面。

namespace unistackbot_serial
{

namespace
{

// ── kernel ABI 手工复刻 (值来源 asm-generic/termbits.h, 布局稳定) ──
constexpr unsigned int kIoctlTCGETS2 = 0x802C542Au;   // _IOR('T', 0x2A, struct termios2)
constexpr unsigned int kIoctlTCSETS2 = 0x402C542Au;   // _IOW('T', 0x2B, struct termios2)
constexpr unsigned int kCbaudMask = 0x100F;           // CBAUD(0x100F incl BOTHER 位)
constexpr unsigned int kBother = 0x1000;              // BOTHER

// glibc <termios.h> 与 <linux/termios.h> 都定义 termios 结构——后者用到的
// TCGETS/TCSETS 系宏会撞 glibc 版本。这里只借用 termios2 + BOTHER + TCSETS2。
// (若工具链报重定义, 用 #undef 防线在本翻译单元内隔离。)

[[nodiscard]] bool configureRaw8N1(int fd, std::string &err)
{
	termios tio{};
	if (tcgetattr(fd, &tio) != 0)
	{
		err = std::string("tcgetattr: ") + std::strerror(errno);
		return false;
	}
	cfmakeraw(&tio);                       // raw: 无行处理/无流控/无回显
	tio.c_cflag |= static_cast<tcflag_t>(CS8) | CLOCAL | CREAD;   // 8N1
	tio.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CRTSCTS));
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 0;
	if (tcsetattr(fd, TCSANOW, &tio) != 0)
	{
		err = std::string("tcsetattr(raw): ") + std::strerror(errno);
		return false;
	}
	return true;
}

// glibc <termios.h> 与 <asm/termbits.h> 都定义 struct termios——本地复刻 kernel ABI
// 的 termios2 (字段布局稳定, ioctl 载体; tcflag_t = unsigned int)。
struct LocalTermios2
{
	unsigned int c_iflag;
	unsigned int c_oflag;
	unsigned int c_cflag;
	unsigned int c_lflag;
	unsigned char c_line;
	unsigned char c_cc[19];
	unsigned int c_ispeed;
	unsigned int c_ospeed;
};

[[nodiscard]] bool setCustomBaud(int fd, int baudrate, std::string &err)
{
	// termios2 + BOTHER: 任意速率 (6M 等不在标准 B 常量表)
	LocalTermios2 tio2{};
	memset(&tio2, 0, sizeof(tio2));
	if (ioctl(fd, kIoctlTCGETS2, &tio2) != 0)
	{
		err = std::string("TCGETS2: ") + std::strerror(errno);
		return false;
	}
	tio2.c_cflag &= ~kCbaudMask;
	tio2.c_cflag |= kBother;
	tio2.c_ospeed = static_cast<unsigned int>(baudrate);
	tio2.c_ispeed = static_cast<unsigned int>(baudrate);
	if (ioctl(fd, kIoctlTCSETS2, &tio2) != 0)
	{
		err = std::string("TCSETS2(BOTHER): ") + std::strerror(errno);
		return false;
	}
	return true;
}

}  // namespace

std::unique_ptr<TermiosTransport> TermiosTransport::open(const std::string &device,
	int baudrate, std::string &err)
{
	if (device.empty() || baudrate <= 0)
	{
		err = "device/baudrate 非法";
		return nullptr;
	}
	const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
	{
		err = std::string("open(") + device + "): " + std::strerror(errno);
		return nullptr;
	}
	// 独占 (防双主站)
	if (ioctl(fd, TIOCEXCL, nullptr) != 0)
	{
		err = std::string("TIOCEXCL: ") + std::strerror(errno);
		::close(fd);
		return nullptr;
	}
	auto t = std::unique_ptr<TermiosTransport>(new TermiosTransport());
	t->fd_ = fd;
	if (!configureRaw8N1(fd, err) || !setCustomBaud(fd, baudrate, err))
	{
		t.reset();
		::close(fd);
		return nullptr;
	}
	// 清残留 (上一次会话的在途字节)
	tcflush(fd, TCIOFLUSH);
	return t;
}

TermiosTransport::~TermiosTransport()
{
	close();
}

ssize_t TermiosTransport::read(std::uint8_t *buf, std::size_t cap)
{
	if (fd_ < 0)
	{
		return -1;
	}
	const ssize_t n = ::read(fd_, buf, cap);
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
	{
		return 0;   // 非阻塞无数据
	}
	return n;
}

ssize_t TermiosTransport::write(const std::uint8_t *buf, std::size_t n)
{
	if (fd_ < 0)
	{
		return -1;
	}
	return ::write(fd_, buf, n);
}

std::size_t TermiosTransport::bytes_available() const
{
	if (fd_ < 0)
	{
		return 0;
	}
	int n = 0;
	if (ioctl(fd_, FIONREAD, &n) != 0 || n < 0)
	{
		return 0;
	}
	return static_cast<std::size_t>(n);
}

void TermiosTransport::close()
{
	if (fd_ >= 0)
	{
		::close(fd_);
		fd_ = -1;
	}
}

}  // namespace unistackbot_serial

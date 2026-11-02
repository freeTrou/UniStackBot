/*
 * fake_board_daemon —— 虚拟电机板 UDP 守护进程 (Phase 3)
 * 绑定 10086, 收 278B Control → FakeBoard::process → 回 474B Response。
 * 用法: ros2 run unistackbot_udp_board fake_board_daemon [port] (默认 10086)
 */
#include <arpa/inet.h>
#include <csignal>
#include <cstdio>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>

#include "unistackbot_udp_board/board_protocol.hpp"
#include "unistackbot_udp_board/fake_board.hpp"

using namespace unistackbot_udp_board;

static std::atomic<bool> g_run{true};
static void on_sig(int) { g_run.store(false); }

int main(int argc, char ** argv)
{
	const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 10086;
	std::signal(SIGINT, on_sig);
	std::signal(SIGTERM, on_sig);

	const int s = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	addr.sin_port = htons(port);
	if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
	{
		std::fprintf(stderr, "fake_board: bind 端口 %u 失败\n", port);
		return 1;
	}
	std::printf("fake_board: 监听 127.0.0.1:%u (Ctrl+C 退出)\n", port);
	std::fflush(stdout);

	FakeBoard board;
	uint8_t rxBuf[kControlFrameSize];
	uint8_t txBuf[kResponseFrameSize];
	uint64_t total = 0;
	while (g_run.load())
	{
		sockaddr_in from{};
		socklen_t flen = sizeof(from);
		const ssize_t n = recvfrom(s, rxBuf, sizeof(rxBuf), 0,
			reinterpret_cast<sockaddr *>(&from), &flen);
		if (n != static_cast<ssize_t>(kControlFrameSize))
			continue;
		ControlFrameBits cmd{};
		if (!decodeControl(rxBuf, static_cast<size_t>(n), cmd))
			continue;
		ResponseFrameBits rsp = board.process(cmd);
		if (encodeResponse(rsp, txBuf, sizeof(txBuf)))
		{
			sendto(s, txBuf, sizeof(txBuf), 0,
				reinterpret_cast<const sockaddr *>(&from), flen);
		}
		++total;
		if (total % 1000 == 0)
		{
			std::printf("fake_board: %llu 帧已处理\n", static_cast<unsigned long long>(total));
			std::fflush(stdout);
		}
	}
	close(s);
	std::printf("fake_board: 退出 (%llu 帧总计)\n", static_cast<unsigned long long>(total));
	return 0;
}

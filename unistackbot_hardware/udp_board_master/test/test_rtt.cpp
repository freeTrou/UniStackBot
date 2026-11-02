/*
 * test_rtt —— FakeBoard UDP 回环往返时延自测 (g++ 直编; A 列 "RTT 自测" 项)
 * 形态: 同进程内 client socket ↔ FakeBoard 线程 (127.0.0.1), N 帧往返统计 p50/p99/max。
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "unistackbot_udp_board/board_protocol.hpp"
#include "unistackbot_udp_board/fake_board.hpp"

using namespace std::chrono;
using namespace unistackbot_udp_board;

int main()
{
	constexpr int kFrames = 2000;
	constexpr uint16_t kBoardPort = 21086;   // 板侧 (FakeBoard 线程)
	constexpr uint16_t kClientPort = 21090;  // 主机侧

	std::atomic<bool> run{true};
	std::thread board_thread([&] {
		const int s = socket(AF_INET, SOCK_DGRAM, 0);
		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr("127.0.0.1");
		addr.sin_port = htons(kBoardPort);
		bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
		FakeBoard board;
		uint8_t rxBuf[kControlFrameSize];
		while (run.load())
		{
			sockaddr_in from{};
			socklen_t flen = sizeof(from);
			const ssize_t n = recvfrom(s, rxBuf, sizeof(rxBuf), MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&from), &flen);
			if (n <= 0)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(50));
				continue;
			}
			ControlFrameBits cmd{};
			if (decodeControl(rxBuf, static_cast<size_t>(n), cmd))
			{
				uint8_t tx[kResponseFrameSize];
				if (encodeResponse(board.process(cmd), tx, sizeof(tx)))
				{
					sendto(s, tx, sizeof(tx), 0, reinterpret_cast<sockaddr *>(&from), flen);
				}
			}
		}
		close(s);
	});

	const int c = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	addr.sin_port = htons(kClientPort);
	bind(c, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
	sockaddr_in board_addr{};
	board_addr.sin_family = AF_INET;
	board_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	board_addr.sin_port = htons(kBoardPort);

	std::vector<double> us;
	us.reserve(kFrames);
	uint8_t tx[kControlFrameSize];
	uint8_t rx[kResponseFrameSize * 2];
	for (int i = 0; i < kFrames; ++i)
	{
		ControlFrameBits cmd{};
		cmd.ts = static_cast<uint8_t>(i & 0xFF);
		cmd.bus_chassis.motor[0].tar_vel = static_cast<int16_t>(i);
		if (!encodeControl(cmd, tx, sizeof(tx)))
		{
			continue;
		}
		const auto t0 = steady_clock::now();
		sendto(c, tx, sizeof(tx), 0, reinterpret_cast<sockaddr *>(&board_addr), sizeof(board_addr));
		const ssize_t n = recv(c, rx, sizeof(rx), 0);
		const auto t1 = steady_clock::now();
		ResponseFrameBits rsp{};
		if (n == static_cast<ssize_t>(kResponseFrameSize) && decodeResponse(rx, static_cast<size_t>(n), rsp) &&
			rsp.ts == cmd.ts && rsp.bus_chassis.motor[0].cur_vel == 1000)
		{
			us.push_back(duration_cast<duration<double, std::micro>>(t1 - t0).count());
		}
	}
	run.store(false);
	board_thread.join();
	close(c);

	std::sort(us.begin(), us.end());
	std::printf("udp_board RTT: %zu/%d frames ok, p50=%.0fus p99=%.0fus max=%.0fus\n", us.size(), kFrames,
		us[us.size() / 2], us[us.size() * 99 / 100], us.back());
	return us.size() == static_cast<size_t>(kFrames) ? 0 : 1;
}

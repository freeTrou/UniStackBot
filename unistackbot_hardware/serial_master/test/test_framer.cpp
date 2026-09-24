/*
 * test_framer —— 帧提取器测试 (2026-09-24, 切片1步3)。
 *
 * 覆盖: 噪声中找帧 / 跨调用拼接 (USB 攒批常态) / 坏帧重同步 / 一批多帧 /
 * 命令回显免疫 (FE EE 不误认 FC EE) / 帧头首字节残尾保留。
 *
 * 编译运行 (零 ROS):
 *   cd unistackbot_hardware/serial_master/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_framer.cpp ../src/framer.cpp \
 *       -I../include -I../../protocol/include -o /tmp/test_framer && /tmp/test_framer
 */
#include <cstdio>
#include <cstring>

#include "unistackbot_protocol/unitree_im_core.hpp"
#include "unistackbot_serial/framer.hpp"

using unistackbot_protocol::FrameSpec;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeMode;
using unistackbot_serial::ExtractedFrame;
using unistackbot_serial::Framer;
using unistackbot_serial::kMaxFrameLen;
using Cmd = std::uint8_t[20];

namespace
{

int g_fail = 0;
int g_case = 0;

void check(bool ok, const char *what)
{
	++g_case;
	if (!ok)
	{
		++g_fail;
		std::printf("FAIL [%d] %s\n", g_case, what);
	}
}

// 造一条有效反馈帧 (26B, CRC 用协议实现)
void makeFb(Cmd, std::uint8_t out[26], std::uint8_t id, std::uint8_t mode)
{
	std::memset(out, 0, 26);
	out[0] = 0xFC;
	out[1] = 0xEE;
	out[2] = static_cast<std::uint8_t>((id & 0x0Fu) | ((mode & 0x07u) << 4));
	const std::uint32_t crc = unistackbot_protocol::unitree_im::crc32(out + 2, 20);
	out[22] = static_cast<std::uint8_t>(crc & 0xFF);
	out[23] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
	out[24] = static_cast<std::uint8_t>((crc >> 16) & 0xFF);
	out[25] = static_cast<std::uint8_t>((crc >> 24) & 0xFF);
}

FrameSpec spec()
{
	unistackbot_protocol::UnitreeImCore core;
	return core.framespec();
}

}  // namespace

int main()
{
	Framer fr(spec());
	ExtractedFrame out[8];
	std::uint8_t fb[26];
	Cmd cmd;

	// ① 噪声前缀 + 有效帧
	makeFb(cmd, fb, 3, 1);
	std::uint8_t noisy[32];
	std::memset(noisy, 0xAB, sizeof(noisy));
	std::memcpy(noisy + 6, fb, 26);
	std::size_t n = fr.push(noisy, sizeof(noisy), out, 8);
	check(n == 1 && out[0].len == 26 && std::memcmp(out[0].bytes, fb, 26) == 0,
		"frame found in noise");

	// ② 跨调用拼接: 帧劈两半
	Framer fr2(spec());
	n = fr2.push(fb, 10, out, 8);
	check(n == 0, "partial frame not emitted");
	n = fr2.push(fb + 10, 16, out, 8);
	check(n == 1 && std::memcmp(out[0].bytes, fb, 26) == 0, "stitched frame");

	// ③ 一批多帧 + 尾部残缺
	Framer fr3(spec());
	std::uint8_t two[26 * 2 + 5];
	makeFb(cmd, fb, 1, 1);
	std::memcpy(two, fb, 26);
	makeFb(cmd, fb, 2, 1);
	std::memcpy(two + 26, fb, 26);
	std::memcpy(two + 52, "\xFC\xEE\x01", 3);
	n = fr3.push(two, sizeof(two), out, 8);
	check(n == 2, "two frames one push");
	std::uint8_t tail[23] = {};   // 全零补完残缺帧
	n = fr3.push(tail, 23, out, 8);
	check(n == 1, "tail completed next push");

	// ④ 命令回显免疫: FE EE 开头不误认
	Framer fr4(spec());
	std::uint8_t echo[20];
	std::memset(echo, 0, sizeof(echo));
	echo[0] = 0xFE;
	echo[1] = 0xEE;
	n = fr4.push(echo, 20, out, 8);
	check(n == 0, "cmd echo ignored");
	makeFb(cmd, fb, 5, 1);
	n = fr4.push(fb, 26, out, 8);
	check(n == 1, "fb after echo still found");

	// ⑤ 纯噪声: 只留残尾, 后续帧仍可收
	Framer fr5(spec());
	std::uint8_t junk[64];
	std::memset(junk, 0x11, sizeof(junk));
	junk[63] = 0xFC;
	n = fr5.push(junk, 64, out, 8);
	check(n == 0, "pure junk no frame");
	std::uint8_t rest[25];
	rest[0] = 0xEE;
	makeFb(cmd, fb, 7, 0);
	// 残尾 FC + 后续 EE...0xEE 开头不构成完整头——造一个头从残尾续上的帧
	std::memcpy(rest + 1, fb + 2, 24);
	n = fr5.push(rest, 25, out, 8);
	check(n == 1, "frame spanning junk-tail recovered");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

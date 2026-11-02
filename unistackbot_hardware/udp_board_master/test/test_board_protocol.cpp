/*
 * test_board_protocol —— 帧契约黄金测试 (g++ 直编, 零 ROS; hardware 域先例)
 *
 * golden 口径: debug/example_run_frames.md 实录 (@single-motor-control)
 *   锚 1: C++ example rx#200 底盘 m0..3 tar_vel(raw) = [540, 540, 540, 540]
 *   锚 2: Python example rx#400 = [1122, 1122, 1122, 1122]
 *   锚 3: 假数据规则 底盘电机 i: vel=1000+i, pos=5000+i*100, tor=i, status=3, error=0
 * python 位解释对照 (fake_motor.py decode_tar_vel): 8 字节小端整数按位域取 [34,49) 位
 */
#include <cstdio>
#include <cstring>

#include "unistackbot_udp_board/board_protocol.hpp"
#include "unistackbot_udp_board/fake_board.hpp"

using namespace unistackbot_udp_board;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, name)                                                                                              \
	do                                                                                                                  \
	{                                                                                                                   \
		if (cond)                                                                                                       \
		{                                                                                                               \
			++g_pass;                                                                                                   \
		}                                                                                                               \
		else                                                                                                            \
		{                                                                                                               \
			++g_fail;                                                                                                   \
			std::printf("FAIL: %s (line %d)\n", name, __LINE__);                                                         \
		}                                                                                                               \
	} while (0)

// fake_motor.py 同款位解释 (小端整数取位 + 符号扩展) —— 独立于位域 struct 的第二视角
static int64_t pyBits(const uint8_t * p, size_t bitOff, size_t width)
{
	uint64_t raw = 0;
	std::memcpy(&raw, p, 8);
	int64_t v = static_cast<int64_t>((raw >> bitOff) & ((uint64_t{1} << width) - 1));
	if ((v & (int64_t{1} << (width - 1))) != 0)
	{
		v -= int64_t{1} << width;   // python: raw & 0x4000 → raw -= 0x8000 同款
	}
	return v;
}

int main()
{
	// ── 1. 位域黄金: 命令电机 tar_vel 编码 → python 视角解码 (实录锚 540/1122 + 负值符号锚) ──
	for (const int16_t anchor : {int16_t{540}, int16_t{1122}, int16_t{-540}, int16_t{-16384}, int16_t{16383}})
	{
		ControlFrameBits cmd{};
		cmd.ts = 1;
		cmd.bus_chassis.motor[0].tar_vel = anchor;
		cmd.bus_chassis.motor[3].tar_vel = anchor;
		uint8_t buf[kControlFrameSize];
		CHECK(encodeControl(cmd, buf, sizeof(buf)), "encodeControl");
		// python 视角: 底盘 m0 = 偏移 198 起 8 字节, tar_vel 位 [34,49)
		CHECK(pyBits(buf + 198, 34, 15) == anchor, "tar_vel py-view anchor");
		CHECK(pyBits(buf + 198 + 3 * 8, 34, 15) == anchor, "tar_vel py-view m3");
		ControlFrameBits back{};
		CHECK(decodeControl(buf, sizeof(buf), back), "decodeControl");
		CHECK(back.bus_chassis.motor[0].tar_vel == anchor && back.bus_chassis.motor[3].tar_vel == anchor,
			"tar_vel roundtrip");
	}

	// ── 2. _MotorCmd 全字段往返 (位域无稀释) ──
	{
		MotorCmdBits c{};
		c.enable = 1;
		c.kd_ki = 712;
		c.kp = 1397;
		c.tar_tor = -321;
		c.tar_vel = -16001;
		c.tar_pos = 16001;
		uint8_t buf[kControlFrameSize] = {0};
		ControlFrameBits cmd{};
		cmd.bus[2].motor[5] = c;
		CHECK(encodeControl(cmd, buf, sizeof(buf)), "encode full");
		ControlFrameBits back{};
		CHECK(decodeControl(buf, sizeof(buf), back), "decode full");
		const MotorCmdBits & r = back.bus[2].motor[5];
		CHECK(r.enable == 1 && r.kd_ki == 712 && r.kp == 1397 && r.tar_tor == -321 && r.tar_vel == -16001 &&
				r.tar_pos == 16001, "MotorCmd all-fields roundtrip (含负值符号位)");
	}

	// ── 3. 帧级合法性门 ──
	{
		uint8_t buf[kControlFrameSize] = {0};
		ControlFrameBits cmd{};
		CHECK(encodeControl(cmd, buf, sizeof(buf)), "encode for gate");
		CHECK(buf[1] == kControlFrameId, "control frame id stamped");
		ControlFrameBits back{};
		CHECK(!decodeControl(buf, sizeof(buf) - 1, back), "len gate rejects");
		buf[1] = 0x7F;
		CHECK(!decodeControl(buf, sizeof(buf), back), "id gate rejects");
		CHECK(!decodeControl(nullptr, sizeof(buf), back), "null gate");
	}

	// ── 4. FakeBoard: 底盘假数据规则 (实录锚 3) + 臂回声 ──
	{
		FakeBoard board;
		ControlFrameBits cmd{};
		cmd.ts = 42;
		cmd.bus[0].ctrl_word = kCtrlWordPdMit;
		cmd.bus[0].motor[3].enable = 1;
		cmd.bus[0].motor[3].tar_pos = 2984;
		cmd.bus[0].motor[3].tar_vel = 540;
		cmd.bus_chassis.motor[0].tar_vel = 540;
		const ResponseFrameBits rsp = board.process(cmd);
		CHECK(rsp.ts == 42, "ts echo");
		// 底盘 golden 假数据
		CHECK(rsp.bus_chassis.motor[0].cur_vel == 1000 && rsp.bus_chassis.motor[0].cur_pos == 5000 &&
				rsp.bus_chassis.motor[0].cur_tor == 0 && rsp.bus_chassis.motor[0].status == 3 &&
				rsp.bus_chassis.motor[0].error == 0, "chassis fake m0");
		CHECK(rsp.bus_chassis.motor[9].cur_vel == 1009 && rsp.bus_chassis.motor[9].cur_pos == 5900 &&
				rsp.bus_chassis.motor[9].cur_tor == 9, "chassis fake m9");
		// 臂回声
		CHECK(rsp.bus[0].motor[3].cur_pos == 2984 && rsp.bus[0].motor[3].cur_vel == 540 &&
				rsp.bus[0].motor[3].enable == 1 && rsp.bus[0].motor[3].status == kCtrlWordPdMit,
			"arm echo");
		// Response 编码 → 按实录偏移复查底盘 m0 (290 + 0*16 的 int32 视角)
		uint8_t rbuf[kResponseFrameSize];
		CHECK(encodeResponse(rsp, rbuf, sizeof(rbuf)) && rbuf[1] == kResponseFrameId, "encode response");
		int32_t vel0 = 0;
		std::memcpy(&vel0, rbuf + kChassisStateOffset, sizeof(vel0));
		CHECK(vel0 == 1000, "chassis m0 vel at golden offset 290");
		// 编解码整体往返
		ResponseFrameBits rback{};
		CHECK(decodeResponse(rbuf, sizeof(rbuf), rback) && rback.bus[0].motor[3].cur_pos == 2984,
			"response roundtrip");
		CHECK(board.framesHandled() == 1, "frame counter");
	}

	// ── 5. 全帧零值往返 (所有字节确定性) ──
	{
		ControlFrameBits zero{};
		zero.ts = 255;
		uint8_t buf[kControlFrameSize];
		CHECK(encodeControl(zero, buf, sizeof(buf)), "encode zero");
		uint8_t expect[kControlFrameSize] = {0};
		expect[0] = 255;
		expect[1] = kControlFrameId;
		CHECK(std::memcmp(buf, expect, sizeof(buf)) == 0, "zero frame deterministic bytes");
	}

	std::printf("board_protocol: %d pass, %d fail\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}

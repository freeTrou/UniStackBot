/*
 * test_unitree_im —— unitree_im 内芯测试 (2026-09-24, 切片1步2)。
 *
 * 黄金标准 = 厂商 demo 源码甲骨文 (test/oracle_unitree_demo/, test-only):
 *   ①encode 方向: 我的 encodeFrame vs 厂商 buildControlPacket **逐字节比对**
 *     (CRC 字序变体对不对, 此测试一锤定音)
 *   ②decode 方向: 我造的反馈帧 (CRC 用我的实现) 喂厂商 parseFeedbackPacket——
 *     厂商解析器用它自己的 CRC 校验并解出全部字段, 与我的 decodeFrame 互校
 *   ③CRC 表: 生成表前 40 项与厂商发布表逐项比对
 *   ④非 38/3 减速比的换算复式记账 (6010=32 / 5010=16, 厂商 demo 硬编码 38/3 不适用)
 *   ⑤状态映射四态 / 拒收路径 (坏头/坏CRC/坏长度/非法ratio/非有限输入)
 *
 * 编译运行 (零 ROS, g++ 直编——"脱开 ROS 现成可用"的可执行证明):
 *   cd unistackbot_protocol/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_unitree_im.cpp \
 *       oracle_unitree_demo/MotorProtocol.cpp ../src/unitree_im_core.cpp ../src/protocol_factory.cpp \
 *       -I../include -Ioracle_unitree_demo -o /tmp/test_unitree_im && /tmp/test_unitree_im
 */
// 厂商甲骨文头必须最先 include——它自带 constexpr M_PI, 若 <cmath> 先到,
// glibc 的 M_PI 宏会让它的成员定义炸掉 (顺序敏感, 只影响测试编译)
#include "MotorProtocol.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "unistackbot_protocol/unitree_im_core.hpp"
#include "unistackbot_protocol/protocol_factory.hpp"

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_protocol::UnitreeImCore;
using unistackbot_protocol::unitree_im::decodeFrame;
using unistackbot_protocol::unitree_im::encodeFrame;

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

// 厂商 float 字段对拍: float 存储精度容差
void checkF(double a, float b, const char *what)
{
	const double scale = std::max(1.0, std::max(std::fabs(a), std::fabs(static_cast<double>(b))));
	++g_case;
	if (!(std::fabs(a - static_cast<double>(b)) <= 1e-6 * scale))
	{
		++g_fail;
		std::printf("FAIL [%d] %s: %.12g vs %.12g\n", g_case, what, a, static_cast<double>(b));
	}
}

// 厂商 demo 硬编码减速比 (其换算/限位仅在此值域有效)
constexpr double kOracleRatio = 38.0 / 3.0;

// ①encode 甲骨文逐字节对拍
void testEncodeOracle()
{
	MotorProtocol oracle;
	struct Vals
	{
		double tau, spd, pos, kp, kd;
	};
	const std::vector<Vals> cases = {
		{0, 0, 0, 0, 0},
		{1.5, -0.7, 2.345, 8.0, 0.4},
		{-10.0, 100.0, -50.0, 40.0, 5.0},
		{0.001, -0.001, 0.0001, 0.01, 0.001},
		{160.0, 250.0, 32000.0, 410.0, 100.0},
		{-160.0, -250.0, -32000.0, 410.0, 100.0},
	};
	const int ids[] = {1, 3, 14};
	for (int id : ids)
	{
		for (int mode = 0; mode <= 1; ++mode)
		{
			for (int tmo = 0; tmo <= 1; ++tmo)
			{
				for (const Vals &v : cases)
				{
					NodeCommand cmd;
					cmd.mode = mode == 1 ? NodeMode::kRun : NodeMode::kStop;
					cmd.watchdog_enable = tmo == 1;
					cmd.tau = v.tau;
					cmd.speed = v.spd;
					cmd.position = v.pos;
					cmd.kp = v.kp;
					cmd.kd = v.kd;

					std::uint8_t mine[20] = {};
					std::vector<std::uint8_t> ref = oracle.buildControlPacket(
						static_cast<std::uint8_t>(id), static_cast<std::uint8_t>(mode),
						tmo == 1, static_cast<float>(v.tau), static_cast<float>(v.spd),
						static_cast<float>(v.pos), static_cast<float>(v.kp),
						static_cast<float>(v.kd));

					const std::size_t n = encodeFrame(mine, sizeof(mine), cmd,
						static_cast<std::uint8_t>(id), kOracleRatio);
					char what[96];
					std::snprintf(what, sizeof(what), "encode id=%d mode=%d tmo=%d", id, mode, tmo);
					check(n == 20 && ref.size() == 20, what);
					if (n == 20 && ref.size() == 20)
					{
						check(std::memcmp(mine, ref.data(), 20) == 0, what);
					}
				}
			}
		}
	}
}

// ②decode 甲骨文互校: 我造帧 (我的CRC) → 厂商解析器 (厂商CRC校验) + 我的 decode
void testDecodeOracle()
{
	struct Synth
	{
		std::uint8_t id;
		std::uint8_t mode;
		bool timeout;
		std::int8_t t1;
		std::uint8_t t2;
		std::uint8_t vol;
		std::int16_t tau_raw;
		std::int16_t spd_raw;
		std::int32_t pos_raw;
		std::uint32_t err;
		std::uint8_t warn;
	};
	const std::vector<Synth> cases = {
		{1, 1, false, 45, 60, 48, 2560, 640, 32768, 0, 0},
		{7, 0, true, -10, 120, 100, -5120, -640, -65536, 0x00000100u, 0},
		{14, 1, false, 80, 200, 90, 12345, -4321, 12345678, 0, 5},
	};
	for (const Synth &s : cases)
	{
		std::uint8_t f[26] = {};
		f[0] = 0xFC;
		f[1] = 0xEE;
		f[2] = static_cast<std::uint8_t>((s.id & 0x0Fu) | ((s.mode & 0x07u) << 4) |
			(s.timeout ? 0x80u : 0u));
		f[3] = static_cast<std::uint8_t>(s.t1);
		f[4] = s.t2;
		f[5] = s.vol;
		f[6] = static_cast<std::uint8_t>(s.tau_raw & 0xFF);
		f[7] = static_cast<std::uint8_t>((static_cast<std::uint16_t>(s.tau_raw) >> 8) & 0xFF);
		f[8] = static_cast<std::uint8_t>(s.spd_raw & 0xFF);
		f[9] = static_cast<std::uint8_t>((static_cast<std::uint16_t>(s.spd_raw) >> 8) & 0xFF);
		const std::uint32_t pos_u = static_cast<std::uint32_t>(s.pos_raw);
		f[10] = static_cast<std::uint8_t>(pos_u & 0xFF);
		f[11] = static_cast<std::uint8_t>((pos_u >> 8) & 0xFF);
		f[12] = static_cast<std::uint8_t>((pos_u >> 16) & 0xFF);
		f[13] = static_cast<std::uint8_t>((pos_u >> 24) & 0xFF);
		f[14] = static_cast<std::uint8_t>(s.err & 0xFF);
		f[15] = static_cast<std::uint8_t>((s.err >> 8) & 0xFF);
		f[16] = static_cast<std::uint8_t>((s.err >> 16) & 0xFF);
		f[17] = static_cast<std::uint8_t>((s.err >> 24) & 0xFF);
		const std::uint16_t res_warn = static_cast<std::uint16_t>(
			(static_cast<std::uint16_t>(s.warn & 0x07u) << 13));
		f[18] = static_cast<std::uint8_t>(res_warn & 0xFF);
		f[19] = static_cast<std::uint8_t>((res_warn >> 8) & 0xFF);
		f[20] = 0;
		f[21] = 0;
		const std::uint32_t crc = unistackbot_protocol::unitree_im::crc32(f + 2, 20);
		f[22] = static_cast<std::uint8_t>(crc & 0xFF);
		f[23] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
		f[24] = static_cast<std::uint8_t>((crc >> 16) & 0xFF);
		f[25] = static_cast<std::uint8_t>((crc >> 24) & 0xFF);

		// 厂商解析器 (内含厂商自己的 CRC 校验) 必须接受
		MotorProtocol oracle;
		std::vector<std::uint8_t> vec(f, f + 26);
		auto ref = oracle.parseFeedbackPacket(vec);
		check(ref != nullptr, "oracle parse accepts my CRC");

		// 我的 decode 与厂商解出的字段互校
		NodeFeedback fb;
		check(decodeFrame(f, sizeof(f), fb, kOracleRatio), "my decode accepts");
		if (ref != nullptr)
		{
			check(fb.node_id == ref->motor_id, "fb id");
			check(fb.timeout_triggered == (ref->timeout != 0), "fb timeout");
			check(fb.temp_driver == ref->temp_driver, "fb temp1");
			check(fb.temp_winding == ref->temp_winding, "fb temp2");
			// 厂商 MotorFeedback 字段为 float (~7 位有效数字), 用 float 级容差
			checkF(fb.voltage, ref->voltage, "fb voltage");
			checkF(fb.tau, ref->torque, "fb tau");
			checkF(fb.speed, ref->speed, "fb speed");
			checkF(fb.position, ref->position, "fb position");
			check(fb.error_raw == ref->error_code, "fb error");
			check(fb.warning_raw == ref->warning_code, "fb warning");
			check(fb.mode == ((ref->mode == 1) ? NodeMode::kRun : NodeMode::kStop), "fb mode");
		}
	}
}

// ③CRC 表前 40 项 (厂商 MotorProtocol.cpp CRC32_TABLE 的前 5 行原样)
void testCrcTable()
{
	// 标准字节的表 (与厂商表同构——帧 CRC 用字序变体, 表本身按字节序生成)
	auto tableEntry = [](std::uint8_t idx) {
		std::uint32_t crc = static_cast<std::uint32_t>(idx) << 24;   // 字节置于 bit31 顶
		for (int i = 0; i < 8; ++i)
		{
			crc = (crc & 0x80000000u) != 0u ? (crc << 1) ^ 0x04C11DB7u : crc << 1;
		}
		return crc;
	};
	const std::uint32_t vendor[] = {
		0x00000000u, 0x04C11DB7u, 0x09823B6Eu, 0x0D4326D9u, 0x130476DCu, 0x17C56B6Bu,
		0x1A864DB2u, 0x1E475005u, 0x2608EDB8u, 0x22C9F00Fu, 0x2F8AD6D6u, 0x2B4BCB61u,
		0x350C9B64u, 0x31CD86D3u, 0x3C8EA00Au, 0x384FBDBDu, 0x4C11DB70u, 0x48D0C6C7u,
		0x4593E01Eu, 0x4152FDA9u, 0x5F15ADACu, 0x5BD4B01Bu, 0x569796C2u, 0x52568B75u,
		0x6A1936C8u, 0x6ED82B7Fu, 0x639B0DA6u, 0x675A1011u, 0x791D4014u, 0x7DDC5DA3u,
		0x709F7B7Au, 0x745E66CDu, 0x9823B6E0u, 0x9CE2AB57u, 0x91A18D8Eu, 0x95609039u,
		0x8B27C03Cu, 0x8FE6DD8Bu, 0x82A5FB52u, 0x8664E6E5u,
	};
	for (std::size_t i = 0; i < sizeof(vendor) / sizeof(vendor[0]); ++i)
	{
		check(tableEntry(static_cast<std::uint8_t>(i)) == vendor[i], "crc table entry");
	}
}

// ④非 38/3 减速比: 复式记账 (测试内独立重算期望定点值, 与帧字节比对)
void testRatios()
{
	for (double ratio : {32.0, 12.666, 16.0})
	{
		NodeCommand cmd;
		cmd.mode = NodeMode::kRun;
		cmd.tau = 3.75;
		cmd.speed = -1.25;
		cmd.position = 1.5;
		cmd.kp = 20.0;
		cmd.kd = 1.5;
		std::uint8_t f[20] = {};
		check(encodeFrame(f, sizeof(f), cmd, 2, ratio) == 20, "ratio encode");
		const double rr = ratio;
		const auto expect_i16 = [](double raw) {
			return static_cast<std::int16_t>(std::lround(std::min(32767.0, std::max(-32768.0, raw))));
		};
		const std::int16_t e_tau = expect_i16(cmd.tau / rr * 2560.0);
		const std::int16_t e_spd = expect_i16(cmd.speed * rr * 64.0 / (2.0 * 3.14159265358979323846));
		const double kp_raw = cmd.kp / (rr * rr) * 12800.0;
		const double kd_raw = cmd.kd / (rr * rr) * 51200.0;
		const std::int16_t e_kp = expect_i16(kp_raw);
		const std::int16_t e_kd = expect_i16(kd_raw);
		check(f[4] == static_cast<std::uint8_t>(e_tau & 0xFF) &&
			f[5] == static_cast<std::uint8_t>((static_cast<std::uint16_t>(e_tau) >> 8) & 0xFF),
			"ratio tau bytes");
		check(f[6] == static_cast<std::uint8_t>(e_spd & 0xFF) &&
			f[7] == static_cast<std::uint8_t>((static_cast<std::uint16_t>(e_spd) >> 8) & 0xFF),
			"ratio spd bytes");
		check(f[12] == static_cast<std::uint8_t>(e_kp & 0xFF), "ratio kp bytes");
		check(f[14] == static_cast<std::uint8_t>(e_kd & 0xFF), "ratio kd bytes");
	}
}

// ⑤拒收路径 / 钳位 / 接口封装 (状态映射已拆居 unistackbot_statemachine 包)
void testStateAndRejects()
{
	UnitreeImCore core;
	NodeCommand cmd;
	cmd.mode = NodeMode::kRun;
	std::uint8_t f[20] = {};
	check(encodeFrame(f, 19, cmd, 1, kOracleRatio) == 0, "reject cap");
	NodeCommand bad = cmd;
	bad.tau = std::nan("");
	check(encodeFrame(f, sizeof(f), bad, 1, kOracleRatio) == 0, "reject nan");
	check(encodeFrame(f, sizeof(f), cmd, 1, 0.0) == 0, "reject ratio");
	check(encodeFrame(f, sizeof(f), cmd, 15, kOracleRatio) == 0, "reject broadcast id");

	// 钳位: 超转子域 → int16 顶格 (LE)
	NodeCommand big = cmd;
	big.tau = 1.0e6;
	check(encodeFrame(f, sizeof(f), big, 1, kOracleRatio) == 20, "clamp encode ok");
	check(f[4] == 0xFF && f[5] == 0x7F, "clamp tau to int16 max");

	// decode 拒收
	NodeFeedback out;
	check(!decodeFrame(f, 25, out, kOracleRatio), "reject len");
	std::uint8_t g[26] = {};
	std::memcpy(g, f, 20);
	g[0] = 0xFE;
	g[1] = 0xEE;
	check(!decodeFrame(g, 26, out, kOracleRatio), "reject header");
	std::uint8_t h[26] = {};
	h[0] = 0xFC;
	h[1] = 0xEE;
	h[2] = 0x01;
	const std::uint32_t c = unistackbot_protocol::unitree_im::crc32(h + 2, 20);
	h[22] = static_cast<std::uint8_t>(c & 0xFF);
	h[23] = static_cast<std::uint8_t>((c >> 8) & 0xFF);
	h[24] = static_cast<std::uint8_t>((c >> 16) & 0xFF);
	h[25] = static_cast<std::uint8_t>((c >> 24) & 0xFF);
	check(decodeFrame(h, 26, out, kOracleRatio), "handmade fb ok");
	h[23] ^= 0x01;   // 破 CRC
	check(!decodeFrame(h, 26, out, kOracleRatio), "reject crc");
}

// ⑥工厂 + 多态消费: 经基类指针全程使用 (骨架将来就这么消费——接口不是摆设)
void testFactoryAndInterface()
{
	using unistackbot_protocol::ProtocolCore;
	using unistackbot_protocol::availableProtocolCores;
	using unistackbot_protocol::createProtocolCore;

	std::unique_ptr<ProtocolCore> core = createProtocolCore("unitree_im");
	check(core != nullptr, "factory creates unitree_im");
	check(createProtocolCore("no_such_protocol") == nullptr, "unknown name -> null (fail-fast)");
	const std::vector<std::string> names = availableProtocolCores();
	check(names.size() == 1 && names[0] == "unitree_im", "registry lists protocols");

	if (core == nullptr)
	{
		return;
	}
	ProtocolCore &p = *core;   // 全程只握基类指针

	// framespec 经接口
	const auto spec = p.framespec();
	check(spec.header0 == 0xFC && spec.header1 == 0xEE && spec.frame_len == 26 &&
		spec.crc_offset == 22 && spec.crc_cover_offset == 2 && spec.crc_cover_len == 20,
		"poly framespec");

	// encode 经接口, 与甲骨文再对一拍
	MotorProtocol oracle;
	NodeCommand cmd;
	cmd.mode = NodeMode::kRun;
	cmd.watchdog_enable = true;
	cmd.tau = 2.5;
	cmd.speed = -3.25;
	cmd.position = 1.75;
	cmd.kp = 15.0;
	cmd.kd = 2.0;
	std::uint8_t mine[20] = {};
	check(p.encode(mine, sizeof(mine), cmd, 5, kOracleRatio) == 20, "poly encode");
	const std::vector<std::uint8_t> ref = oracle.buildControlPacket(5, 1, true, 2.5f,
		-3.25f, 1.75f, 15.0f, 2.0f);
	check(ref.size() == 20 && std::memcmp(mine, ref.data(), 20) == 0, "poly encode == oracle");

	// decode + map_state 经接口 (复用②的手造帧)
	std::uint8_t f[26] = {};
	f[0] = 0xFC;
	f[1] = 0xEE;
	f[2] = 0x11;   // id=1, mode=1(FOC)
	f[3] = 45;
	f[5] = 96;
	const std::uint32_t c = unistackbot_protocol::unitree_im::crc32(f + 2, 20);
	f[22] = static_cast<std::uint8_t>(c & 0xFF);
	f[23] = static_cast<std::uint8_t>((c >> 8) & 0xFF);
	f[24] = static_cast<std::uint8_t>((c >> 16) & 0xFF);
	f[25] = static_cast<std::uint8_t>((c >> 24) & 0xFF);
	NodeFeedback fb;
	check(p.decode(f, sizeof(f), fb, kOracleRatio), "poly decode");
	check(fb.node_id == 1 && fb.mode == NodeMode::kRun, "poly decode fields");
}

}  // namespace

int main()
{
	testCrcTable();
	testEncodeOracle();
	testDecodeOracle();
	testRatios();
	testStateAndRejects();
	testFactoryAndInterface();
	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

/*
 * test_dm_motor —— dm_motor 内芯测试 (2026-10-21, stationlite 真机接入 A1)。
 *
 * 黄金标准 = startouchlib codec 甲骨文 (test/oracle_startouch/, test-only):
 *   ①encode 方向: 我的 encodeFrame(MIT) vs 厂商 pack_mit 逐字节比对
 *     (位拼接序对不对, 此测试一锤定音) —— 两型号 × 零/半程/顶格/负值/越界钳位
 *   ②decode 方向: 我的 encodeFrame 产物喂厂商 parse_state, 与我的 decodeFrame 互校
 *   ③特殊帧字节: kEnable=FF×7+FC / kStop=FD / kClear=FB (手册控制帧表直读)
 *   ④定点数学直算: p=0→0x8000 (中点), kp=500→0xFFF, kd=5→0xFFF, p=-pMax→0 (全幅口径)
 *   ⑤全码表: ERR 0/1/3/4/5/8/9/A/B/C/D/E 拆分 + ID 半字节 + 温度直读
 *   ⑥拒收路径: buf 不足 / ratio≠1 / len≠8 / MIT 非有限输入
 *   ⑦工厂: dm4310/dm4340 创建 + 接口多态 encode + 未知名 nullptr + 清单含新名
 *   ⑧宇树 kEnable 别名回归: unitree_im 对 kEnable 产 kRun 同字节 (词汇加性不改老行为)
 *
 * 编译运行 (零 ROS, g++ 直编):
 *   cd unistackbot_hardware/protocol/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_dm_motor.cpp \
 *       oracle_startouch/oracle_startouch.cpp ../src/dm_motor_core.cpp ../src/protocol_factory.cpp \
 *       ../src/unitree_im_core.cpp -I../include -Ioracle_startouch -o /tmp/test_dm_motor && /tmp/test_dm_motor
 */
#include "oracle_startouch.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <cstdio>
#include <cstring>
#include <vector>

#include "unistackbot_protocol/dm_motor_core.hpp"
#include "unistackbot_protocol/protocol_factory.hpp"
#include "unistackbot_protocol/unitree_im_core.hpp"

using unistackbot_protocol::DmMotorCore;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_protocol::dm_motor::DmMotorType;
using unistackbot_protocol::dm_motor::decodeFrame;
using unistackbot_protocol::dm_motor::encodeFrame;

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

void checkD(double a, double b, const char *what)
{
	++g_case;
	if (!(std::abs(a - b) <= 1e-9 + 1e-6 * std::abs(b)))
	{
		++g_fail;
		std::printf("FAIL [%d] %s (%.9f vs %.9f)\n", g_case, what, a, b);
	}
}

NodeCommand mit(double q, double dq, double kp, double kd, double tau)
{
	NodeCommand c;
	c.mode = NodeMode::kRun;
	c.position = q;
	c.speed = dq;
	c.kp = kp;
	c.kd = kd;
	c.tau = tau;
	return c;
}

}  // namespace

int main()
{
	constexpr double kInf = std::numeric_limits<double>::infinity();
	// ---- ① encode 逐字节对拍 (两型号 × 参数矩阵, 含越界钳位) ----
	{
		const struct
		{
			double q, dq, kp, kd, tau;
		} cases[] = {
			{0, 0, 0, 0, 0},
			{0.5, -1.25, 300, 5, 2.5},
			{-1.0, 2.0, 490, 2.5, -6.0},
			{1.0, 0.0, 64.95, 0.5, 0.0},
			{12.5, 30.0, 500, 5, 10},     // 4310 顶格
			{-12.5, -30.0, 0, 0, -10},    // 4310 负顶格
			{99.0, 99.0, 600, 6, 99},     // 全越界 (钳位语义对拍)
			{-99.0, -99.0, -1, -1, -99},  // 负越界 (kp/kd 负钳 0)
		};
		for (int t = 0; t < 2; ++t)
		{
			const DmMotorType type = t == 0 ? DmMotorType::kDm4310 : DmMotorType::kDm4340;
			const int otype = t == 0 ? oracle_startouch::DM4310 : oracle_startouch::DM4340;
			for (const auto &c : cases)
			{
				std::uint8_t mine[8] = {};
				const std::size_t n = encodeFrame(mine, sizeof mine, mit(c.q, c.dq, c.kp, c.kd, c.tau), 0x01, 1.0, type);
				oracle_startouch::MitParam op{c.kp, c.kd, c.q, c.dq, c.tau};
				const auto ref = oracle_startouch::pack_mit(otype, op);
				check(n == 8 && ref.size() == 8 && std::memcmp(mine, ref.data(), 8) == 0, "encode 逐字节对拍");
			}
		}
	}

	// ---- ② decode 互校 (反馈帧 ≠ 控制帧布局: D0=ID|ERR 后 q 从 D1 起 —— 用 oracle
	//      定点按反馈布局构造帧, 喂厂商 parse ↔ 我的 decode) ----
	{
		for (int t = 0; t < 2; ++t)
		{
			const DmMotorType type = t == 0 ? DmMotorType::kDm4310 : DmMotorType::kDm4340;
			const int otype = t == 0 ? oracle_startouch::DM4310 : oracle_startouch::DM4340;
			const oracle_startouch::Limit lim = oracle_startouch::kLimits[static_cast<std::size_t>(otype)];
			const double qs[] = {0.0, 0.73, -2.41, 12.5, -12.5};
			for (double q : qs)
			{
				const std::uint16_t q_u = oracle_startouch::double_to_uint(q, -lim.pMax, lim.pMax, 16);
				const std::uint16_t dq_u = oracle_startouch::double_to_uint(-3.0, -lim.vMax, lim.vMax, 12);
				const std::uint16_t tau_u = oracle_startouch::double_to_uint(1.5, -lim.tMax, lim.tMax, 12);
				std::uint8_t frame[8] = {0x01,   // D0 = id=1 | err=0
					static_cast<std::uint8_t>((q_u >> 8) & 0xFF), static_cast<std::uint8_t>(q_u & 0xFF),
					static_cast<std::uint8_t>((dq_u >> 4) & 0xFF),
					static_cast<std::uint8_t>(((dq_u & 0xF) << 4) | ((tau_u >> 8) & 0xF)),
					static_cast<std::uint8_t>(tau_u & 0xFF), 33, 21};
				const auto ref = oracle_startouch::parse_state(otype, std::vector<std::uint8_t>(frame, frame + 8));
				NodeFeedback fb;
				check(ref.valid, "厂商 parse ok");
				check(decodeFrame(frame, 8, fb, 1.0, type), "decode ok");
				checkD(fb.position, ref.position, "position 互校");
				checkD(fb.speed, ref.velocity, "speed 互校");
				checkD(fb.tau, ref.torque, "tau 互校");
			}
		}
	}

	// ---- ③ 特殊帧字节 (手册控制帧表直读) ----
	{
		for (auto [mode, cmd_byte] : std::initializer_list<std::pair<NodeMode, std::uint8_t>>{
				 {NodeMode::kEnable, 0xFC}, {NodeMode::kStop, 0xFD}, {NodeMode::kClear, 0xFB}})
		{
			NodeCommand c;
			c.mode = mode;
			std::uint8_t f[8] = {};
			check(encodeFrame(f, sizeof f, c, 0x01, 1.0, DmMotorType::kDm4310) == 8, "特殊帧长度");
			bool ok = f[0] == 0xFF && f[1] == 0xFF && f[2] == 0xFF && f[3] == 0xFF && f[4] == 0xFF && f[5] == 0xFF && f[6] == 0xFF && f[7] == cmd_byte;
			check(ok, "特殊帧字节 FF×7+cmd");
		}
	}

	// ---- ④ 定点数学直算 (全幅口径, 不经 oracle; 分母 (2^bits)-1 → 零点码字 0x7FFF
	//      非 0x8000 —— 全幅编码无精确零码, decode 端 |q|≤1LSB≈0.19mrad) ----
	{
		NodeFeedback fb;
		std::uint8_t f[8];
		check(encodeFrame(f, sizeof f, mit(0, 0, 500, 5, 0), 0, 1.0, DmMotorType::kDm4310) == 8, "直算帧生成");
		check(f[0] == 0x7F && f[1] == 0xFF, "p=0 → 0x7FFF (全幅零点码字)");
		check((f[3] & 0x0F) == 0x0F && f[4] == 0xFF, "kp=500 → 0xFFF 顶格");
		check(f[5] == 0xFF && (f[6] >> 4) == 0xF, "kd=5 → 0xFFF 顶格");
		check(encodeFrame(f, sizeof f, mit(-12.5, -30, 0, 0, -10), 0, 1.0, DmMotorType::kDm4310) == 8, "直算帧生成 2");
		check(f[0] == 0x00 && f[1] == 0x00, "p=-pMax → 0x0000");
		const std::uint8_t mid[8] = {0x01, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
		check(decodeFrame(mid, 8, fb, 1.0, DmMotorType::kDm4310), "反馈 decode");
		check(std::abs(fb.position) < 2e-4, "反馈零点码字 → |q|<1LSB");
	}

	// ---- ⑤ 全码表 + ID 半字节 + 温度直读 ----
	{
		const std::uint8_t err_codes[] = {0, 1, 3, 4, 5, 8, 9, 0xA, 0xB, 0xC, 0xD, 0xE};
		for (std::uint8_t err : err_codes)
		{
			NodeFeedback fb;
			std::uint8_t f[8] = {(std::uint8_t)((err << 4) | 0x07), 0x80, 0x00, 0x00, 0x00, 0x00, 33, 21};
			check(decodeFrame(f, 8, fb, 1.0, DmMotorType::kDm4310), "码表帧 decode");
			check(fb.error_raw == err, "ERR 半字节");
			check(fb.node_id == 0x07, "ID 半字节");
			check(fb.state_raw == static_cast<std::uint32_t>((err << 4) | 0x07), "state_raw=D0 原样");
			check(fb.temp_driver == 33 && fb.temp_winding == 21, "温度直读");
			check(fb.mode == NodeMode::kStop && !fb.timeout_triggered && fb.voltage == 0.0, "无信息字段约定");
		}
	}

	// ---- ⑥ 拒收路径 ----
	{
		NodeCommand c = mit(0, 0, 1, 1, 0);
		std::uint8_t f[8];
		check(encodeFrame(f, 7, c, 1, 1.0, DmMotorType::kDm4310) == 0, "buf 不足");
		check(encodeFrame(f, 8, c, 1, 0.5, DmMotorType::kDm4310) == 0, "ratio≠1 拒");
		NodeCommand bad = mit(kInf, 0, 1, 1, 0);
		check(encodeFrame(f, 8, bad, 1, 1.0, DmMotorType::kDm4310) == 0, "q=NaN/Inf 拒");
		bad = mit(0, 0, kInf, 1, 0);
		check(encodeFrame(f, 8, bad, 1, 1.0, DmMotorType::kDm4310) == 0, "kp=Inf 拒");
		bad = mit(0, 0, 1, 1, -kInf);
		check(encodeFrame(f, 8, bad, 1, 1.0, DmMotorType::kDm4340) == 0, "tau=-Inf 拒");
		NodeFeedback fb;
		const std::uint8_t ok8[8] = {0x11, 0x80, 0, 0, 0, 0, 30, 30};
		check(!decodeFrame(ok8, 7, fb, 1.0, DmMotorType::kDm4310), "len=7 拒");
		check(!decodeFrame(ok8, 9, fb, 1.0, DmMotorType::kDm4310), "len=9 拒");
		check(!decodeFrame(nullptr, 8, fb, 1.0, DmMotorType::kDm4310), "null 拒");
		check(!decodeFrame(ok8, 8, fb, 2.0, DmMotorType::kDm4310), "decode ratio≠1 拒");
		// 特殊帧不查有限性 (无字段); node_id 任意不拒 (CAN ID 住帧头)
		check(encodeFrame(f, 8, c, 0xFF, 1.0, DmMotorType::kDm4310) == 8, "node_id 不入帧不拒");
	}

	// ---- ⑦ 工厂 ----
	{
		auto p4310 = unistackbot_protocol::createProtocolCore("dm4310");
		auto p4340 = unistackbot_protocol::createProtocolCore("dm4340");
		check(p4310 != nullptr && p4340 != nullptr, "工厂创建两名");
		check(unistackbot_protocol::createProtocolCore("dm_motor") == nullptr, "未知名 nullptr (一子类两名)");
		NodeCommand c = mit(1.0, 0, 300, 5, 2);
		std::uint8_t f[8] = {};
		check(p4310->encode(f, sizeof f, c, 1, 1.0) == 8, "接口多态 encode");
		oracle_startouch::MitParam op{300, 5, 1.0, 0, 2};
		const auto ref = oracle_startouch::pack_mit(oracle_startouch::DM4310, op);
		check(std::memcmp(f, ref.data(), 8) == 0, "接口路径逐字节");
		NodeFeedback fb;
		// 接口多态 decode: 反馈布局帧 (D0=ID|ERR, q 从 D1 起) —— q=1.0 的 4310 码字
		const std::uint16_t q_u = oracle_startouch::double_to_uint(1.0, -12.5, 12.5, 16);
		const std::uint8_t fbframe[8] = {0x01, static_cast<std::uint8_t>((q_u >> 8) & 0xFF),
			static_cast<std::uint8_t>(q_u & 0xFF), 0x00, 0x00, 0x00, 30, 30};
		check(p4340->decode(fbframe, 8, fb, 1.0), "接口多态 decode");
		check(std::abs(fb.position - 1.0) < 5e-4, "4340 档位量程生效 (1LSB=3.8e-4 量化容差)");
		// framespec: CAN 无帧头/CRC
		auto spec = p4310->framespec();
		check(spec.frame_len == 8 && spec.header0 == 0 && spec.crc_offset == 0, "framespec CAN 语义");
		bool found = false;
		for (const auto &n : unistackbot_protocol::availableProtocolCores())
		{
			found = found || n == "dm4310" || n == "dm4340";
		}
		check(found, "清单含新名");
		// 型号档位差异: 同字节帧在两档下 v 解码不同 (vMax 30 vs 8)
		std::uint8_t vfull[8] = {0x01, 0x80, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00};
		NodeFeedback fb1, fb3;
		p4310->decode(vfull, 8, fb1, 1.0);
		p4340->decode(vfull, 8, fb3, 1.0);
		checkD(fb1.speed, 30.0, "4310 vMax=30");
		checkD(fb3.speed, 8.0, "4340 vMax=8");
	}

	// ---- ⑧ 宇树 kEnable 别名回归 (词汇加性不改老内芯行为) ----
	{
		NodeCommand c;
		c.mode = NodeMode::kEnable;
		c.position = 0.5;
		c.kp = 60;
		c.kd = 0.5;
		std::uint8_t fe[20] = {}, fr[20] = {};
		NodeCommand r = c;
		r.mode = NodeMode::kRun;
		check(unistackbot_protocol::unitree_im::encodeFrame(fe, sizeof fe, c, 3, 6.33) == 20, "unitree kEnable 可编码");
		check(unistackbot_protocol::unitree_im::encodeFrame(fr, sizeof fr, r, 3, 6.33) == 20, "unitree kRun 可编码");
		check(std::memcmp(fe, fr, 20) == 0, "kEnable 与 kRun 同字节 (别名)");
	}

	// ---- ⑨ 探路契约 (B1, protocol_core.hpp 契约: node_id 解码不依赖量程档) ----
	{
		NodeFeedback fa, fb;
		const std::uint8_t f[8] = {0x27, 0x7F, 0xFF, 0x00, 0x00, 0x00, 30, 31};   // id=7 | err=2
		check(decodeFrame(f, 8, fa, 1.0, DmMotorType::kDm4310), "契约帧 4310 decode");
		check(decodeFrame(f, 8, fb, 1.0, DmMotorType::kDm4340), "契约帧 4340 decode");
		check(fa.node_id == fb.node_id && fa.node_id == 0x07, "同帧不同档 node_id 相同 (探路契约)");
		check(fa.error_raw == fb.error_raw && fa.error_raw == 0x2, "err 半字节亦档位无关");
	}

	// ---- ⑩ 镜像方向 (2026-10-27, can1 虚拟电机/录制分析): unpackCommand/packFeedback ----
	{
		// 命令往返: encode → unpack, 容差 = 各字段 1 LSB 量化 (防"双错自洽"另有甲骨文层①兜底)
		const struct
		{
			double q, dq, kp, kd, tau;
		} rt[] = {
			{0, 0, 45, 2, 0},
			{1.234, -5.6, 300, 2.5, 3.7},
			{-3.14, 20.0, 500, 5, -9.9},
			{12.5, -30.0, 0, 0, 10},      // 顶格
		};
		const double q_lsb = 25.0 / 65535.0, v_lsb = 60.0 / 4095.0;
		const double kp_lsb = 500.0 / 4095.0, kd_lsb = 5.0 / 4095.0, t_lsb = 20.0 / 4095.0;
		for (const auto & c : rt)
		{
			NodeCommand cmd = mit(c.q, c.dq, c.kp, c.kd, c.tau);
			std::uint8_t f[8];
			check(encodeFrame(f, sizeof f, cmd, 0, 1.0, DmMotorType::kDm4310) == 8, "往返: encode");
			NodeCommand back;
			check(unpackCommand(f, 8, back, 1.0, DmMotorType::kDm4310), "往返: unpack");
			check(back.mode == NodeMode::kRun, "往返: mode=kRun");
			check(std::abs(back.position - c.q) <= q_lsb + 1e-9, "往返: |Δq|≤1LSB");
			check(std::abs(back.speed - c.dq) <= v_lsb + 1e-9, "往返: |Δdq|≤1LSB");
			check(std::abs(back.kp - c.kp) <= kp_lsb + 1e-9, "往返: |Δkp|≤1LSB");
			check(std::abs(back.kd - c.kd) <= kd_lsb + 1e-9, "往返: |Δkd|≤1LSB");
			check(std::abs(back.tau - c.tau) <= t_lsb + 1e-9, "往返: |Δtau|≤1LSB");
		}
		// 特殊帧还原
		for (const auto & [mode, cmd_byte] : std::initializer_list<std::pair<NodeMode, std::uint8_t>>{
				 {NodeMode::kEnable, 0xFCu}, {NodeMode::kStop, 0xFDu}, {NodeMode::kClear, 0xFBu}})
		{
			NodeCommand cmd;
			cmd.mode = mode;
			std::uint8_t f[8];
			check(encodeFrame(f, sizeof f, cmd, 0, 1.0, DmMotorType::kDm4310) == 8, "特殊帧 encode");
			NodeCommand back;
			check(unpackCommand(f, 8, back, 1.0, DmMotorType::kDm4310), "特殊帧 unpack");
			check(back.mode == mode, "特殊帧 mode 还原");
		}
		// 未知特殊命令拒绝
		{
			const std::uint8_t f[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x33};   // 参数帧 (范围外)
			NodeCommand back;
			check(!unpackCommand(f, 8, back, 1.0, DmMotorType::kDm4310), "未知特殊命令拒绝 (0x33 暂缓范围)");
		}
		// 反馈往返: pack → decode
		{
			NodeFeedback src;
			src.node_id = 0x05;
			src.error_raw = 0x01;   // 使能态
			src.position = 2.345;
			src.speed = -7.8;
			src.tau = 4.2;
			src.temp_driver = -10;
			src.temp_winding = 66;
			std::uint8_t f[8];
			check(packFeedback(f, sizeof f, src, 1.0, DmMotorType::kDm4340) == 8, "反馈 pack");
			NodeFeedback fb;
			check(decodeFrame(f, 8, fb, 1.0, DmMotorType::kDm4340), "反馈 decode");
			check(fb.node_id == 0x05 && fb.error_raw == 0x01, "D0 还原 (ERR|ID)");
			check(std::abs(fb.position - src.position) <= 25.0 / 65535.0 + 1e-9, "反馈 |Δq|≤1LSB");
			check(std::abs(fb.speed - src.speed) <= 16.0 / 4095.0 + 1e-9, "反馈 |Δdq|≤1LSB (4340 档)");
			check(std::abs(fb.tau - src.tau) <= 56.0 / 4095.0 + 1e-9, "反馈 |Δtau|≤1LSB (4340 档)");
			check(fb.temp_driver == -10 && fb.temp_winding == 66, "温度直读往返");
		}
		// pack 非有限拒绝
		{
			NodeFeedback bad;
			bad.position = kInf;
			std::uint8_t f[8];
			check(packFeedback(f, sizeof f, bad, 1.0, DmMotorType::kDm4310) == 0, "pack 非有限拒绝");
		}
	}

	// ---- ⑪ kd=0 手册禁令 (2026-10-27 裁决: 协议知识住协议) ----
	{
		std::uint8_t f[8];
		check(encodeFrame(f, sizeof f, mit(0, 0, 45, 0, 0), 0, 1.0, DmMotorType::kDm4310) == 0,
			"kp>0 且 kd=0 拒绝 (手册: 位置控制震荡)");
		check(encodeFrame(f, sizeof f, mit(0, 0, 0, 0, 2.0), 0, 1.0, DmMotorType::kDm4310) == 8,
			"tau-only (kp=kd=0) 合法 —— 纯力矩前馈");
	}

	std::printf("%s: %d cases, %d fail\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

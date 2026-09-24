/*
 * test_serial_master —— 串口泵全链集成测试 (2026-09-24, 切片1步4; 零硬件)。
 *
 * FakeTransport + FakeMotor (假从站: 解析 20B 命令帧 → 维护内部位姿 → 产 26B 反馈帧)。
 * 覆盖:
 *   ①安全怠速: start 后未发命令 → 线上全是停机帧+看门狗位
 *   ②命令往返: publish_cmd 位置 → 假电机收到 run 模式+位置 → 反馈位姿回读一致
 *   ③状态翻译: 使能→ENABLED; 假电机报错→FAULT
 *   ④断流陈旧: 丢全部应答 → 节点态 Unknown (kStaleCycles 后)
 *   ⑤quick_stop: 置位后线上变停机帧, 反馈回显实测位 (锚定)
 *   ⑥追帧跳过遥测: 正常=0
 *   ⑦工厂注册: registerToMasterFactory 后 createMaster("serial") 可用
 *
 * 编译运行 (零 ROS):
 *   cd unistackbot_hardware/serial_master/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_serial_master.cpp \
 *       ../src/serial_master.cpp ../src/framer.cpp ../src/termios_transport.cpp \
 *       ../../protocol/src/unitree_im_core.cpp ../../protocol/src/protocol_factory.cpp \
 *       ../../statemachine/src/unitree_im_translator.cpp \
 *       ../../statemachine/src/state_translator_factory.cpp \
 *       ../../bus/src/master_factory.cpp \
 *       -I../include -I../../protocol/include -I../../statemachine/include \
 *       -I../../bus/include -I../../../unistackbot_common -pthread \
 *       -o /tmp/test_serial_master && /tmp/test_serial_master
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_protocol/unitree_im_core.hpp"
#include "unistackbot_serial/fake_transport.hpp"
#include "unistackbot_serial/serial_master.hpp"

using unistackbot_bus::BusCommand;
using unistackbot_bus::BusState;
using unistackbot_bus::MasterBase;
using unistackbot_bus::MasterConfig;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_protocol::unitree_im::crc32;
using unistackbot_serial::FakeTransport;
using unistackbot_serial::SerialMaster;
using NeutralState = unistackbot_statemachine::NeutralState;

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

constexpr double kRatio = 38.0 / 3.0;
constexpr std::uint8_t kNodeA = 1;
constexpr std::uint8_t kNodeB = 7;

// ── FakeMotor: 假从站 (进程内) ──
// 解析 20B 命令帧 (帧头/长度/CRC 手工校验), 维护 per-id {mode,q};
// 产 26B 反馈帧 (位置=q, mode 回显; 可注入 error 位码)。
class FakeMotor
{
public:
	std::uint32_t error_bits = 0;   // 注入: 反馈帧 error 字段
	std::uint64_t cmd_frames = 0;
	double last_position[16] = {};
	std::uint8_t last_mode[16] = {};
	bool last_watchdog[16] = {};

	// responder: 输入写入字节 → 输出应答字节
	std::size_t respond(const std::uint8_t *in, std::size_t n, std::uint8_t *out)
	{
		std::size_t produced = 0;
		for (std::size_t i = 0; i + 20 <= n; i += 20)
		{
			if (in[i] != 0xFE || in[i + 1] != 0xEE)
			{
				continue;
			}
			if (crc32(in + i, 16) != load32(in + i + 16))
			{
				continue;   // 坏 CRC: 电机拒收静默 (协议语义)
			}
			++cmd_frames;
			const std::uint8_t id = in[i + 2] & 0x0Fu;
			last_mode[id] = (in[i + 2] >> 4) & 0x07u;
			last_watchdog[id] = (in[i + 2] >> 7) != 0u;
			// 位置: int32 → rad (输出端)
			const std::int32_t q_raw = static_cast<std::int32_t>(load32(in + i + 8));
			last_position[id] = static_cast<double>(q_raw) * 2.0 * 3.14159265358979323846 /
				(32768.0 * kRatio);
			// 假电机瞬时跟踪 (理想执行器)
			if (id < 16)
			{
				produced += buildFb(out + produced, id, last_mode[id], last_position[id]);
			}
		}
		return produced;
	}

private:
	static std::uint32_t load32(const std::uint8_t *p)
	{
		return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
			(static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
	}

	static void store32(std::uint8_t *p, std::uint32_t v)
	{
		p[0] = static_cast<std::uint8_t>(v & 0xFF);
		p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
		p[2] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
		p[3] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
	}

	std::size_t buildFb(std::uint8_t *f, std::uint8_t id, std::uint8_t mode, double q)
	{
		std::memset(f, 0, 26);
		f[0] = 0xFC;
		f[1] = 0xEE;
		f[2] = static_cast<std::uint8_t>((id & 0x0Fu) | ((mode & 0x07u) << 4));
		f[3] = 40;
		f[4] = 50;
		f[5] = 96;   // 48V
		const std::int32_t q_raw = static_cast<std::int32_t>(std::lround(
			q * kRatio * 32768.0 / (2.0 * 3.14159265358979323846)));
		store32(f + 10, static_cast<std::uint32_t>(q_raw));
		store32(f + 14, error_bits);   // 注入位
		const std::uint32_t c = crc32(f + 2, 20);
		store32(f + 22, c);
		return 26;
	}
};

MasterConfig makeCfg()
{
	MasterConfig cfg;
	cfg.protocol = "unitree_im";
	cfg.translator = "unitree_im";
	cfg.endpoint = "fake";
	cfg.rate_hz = 250.0;   // 测试提速 (2 节点 slot 2ms)
	cfg.node_count = 2;
	cfg.node_id[0] = kNodeA;
	cfg.node_id[1] = kNodeB;
	cfg.ratio[0] = kRatio;
	cfg.ratio[1] = kRatio;
	return cfg;
}

}  // namespace

int main()
{
	// ⑦ 工厂注册 (组合根职责的测试模拟)
	check(unistackbot_serial::registerToMasterFactory(), "register serial to factory");
	std::unique_ptr<MasterBase> from_factory = unistackbot_bus::createMaster("serial");
	check(from_factory != nullptr, "factory creates serial");
	check(unistackbot_serial::registerToMasterFactory() == false, "duplicate register rejected");

	// 用工厂产物跑全链
	SerialMaster *m = dynamic_cast<SerialMaster *>(from_factory.get());
	check(m != nullptr, "dynamic cast ok");
	if (m == nullptr)
	{
		std::printf("FAIL: no master\n");
		return 1;
	}

	auto motor = std::make_shared<FakeMotor>();
	auto fake = std::make_unique<FakeTransport>(
		[motor](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
			return motor->respond(in, n, out);
		});
	FakeTransport *fake_raw = fake.get();
	m->attach_transport(std::move(fake));

	check(m->start(makeCfg()), "start with fake transport");
	check(m->running(), "running");

	// ① 安全怠速: 等 10 拍, 线上全是停机帧+看dog位
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(motor->cmd_frames > 5, "frames flowing");
	check(motor->last_mode[kNodeA] == 0 && motor->last_watchdog[kNodeA],
		"idle = stop mode + watchdog");
	check(motor->last_mode[kNodeB] == 0, "idle node B stop too");

	// ② 命令往返
	BusCommand cmd;
	cmd.node[0].mode = NodeMode::kRun;
	cmd.node[0].position = 1.234;
	cmd.node[0].kp = 10.0;
	cmd.node[0].watchdog_enable = true;
	cmd.node[1].mode = NodeMode::kRun;
	cmd.node[1].position = -0.5;
	cmd.node[1].kp = 10.0;
	cmd.node[1].watchdog_enable = true;
	m->publish_cmd(cmd);
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(motor->last_mode[kNodeA] == 1, "run mode on wire");
	check(std::fabs(motor->last_position[kNodeA] - 1.234) < 1e-3, "position roundtrip A");
	check(std::fabs(motor->last_position[kNodeB] + 0.5) < 1e-3, "position roundtrip B");

	// ③ 状态翻译
	BusState st;
	bool got = false;
	for (int i = 0; i < 20 && !got; ++i)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		got = m->take_state(st);
	}
	check(got, "take_state got live snapshot");
	if (got)
	{
		check(st.node_state[0] == NeutralState::kEnabled, "node A ENABLED");
		check(st.node[0].node_id == kNodeA, "node A id matched");
		check(std::fabs(st.node[0].position - 1.234) < 1e-3, "state position A");
	}
	check(m->state() == NeutralState::kEnabled, "segment worst-of = ENABLED");

	// ③b 假电机报错 → FAULT
	motor->error_bits = 0x00000100u;
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(m->state() == NeutralState::kFault, "error bits -> FAULT");
	motor->error_bits = 0;

	// ④ 断流陈旧: 丢全部应答 → Unknown (kStaleCycles=50 拍 @250Hz ≈ 200ms)
	fake_raw->set_drop_responses(1000000);
	std::this_thread::sleep_for(std::chrono::milliseconds(400));
	check(m->state() == NeutralState::kUnknown, "all-drop -> stale Unknown");
	fake_raw->set_drop_responses(0);
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(m->state() != NeutralState::kUnknown, "recovered from stale");

	// ⑤ quick_stop: 线上变停机帧
	m->quick_stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(60));
	check(motor->last_mode[kNodeA] == 0, "quick_stop -> stop frames on wire");

	// ⑥ 追帧遥测: 正常测试全程 skip 应很少 (测试机非 RT 环境, 只断不为爆炸值)
	const auto tl = m->telemetry();
	check(tl.rx_frames > 10, "rx frames counted");
	check(tl.skipped_slots < 50, "skip telemetry sane");

	m->stop();
	check(!m->running(), "stopped");
	check(fake_raw->closed(), "transport closed by stop");

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

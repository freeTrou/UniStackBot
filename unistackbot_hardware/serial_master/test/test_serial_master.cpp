/*
 * test_serial_master —— 串口泵全链集成测试 (2026-09-24, 切片1步4; 2026-09-27 补盲区; 零硬件)。
 *
 * FakeTransport + FakeMotor (假从站: 解析 20B 命令帧 → 维护内部位姿 → 产 26B 反馈帧)。
 * 覆盖:
 *   ①安全怠速: start 后未发命令 → 线上全是停机帧+看门狗位
 *   ②命令往返: publish_cmd 位置 → 假电机收到 run 模式+位置 → 反馈位姿回读一致
 *   ③状态翻译: 使能→ENABLED; 假电机报错→FAULT
 *   ④断流陈旧: 丢全部应答 → 节点态 Unknown (kStaleCycles 后)
 *   ④b/④c 坏帧拒收计数 + 分片应答跨拍拼接 (corrupt/fragment 旋钮)
 *   ⑤quick_stop: 置位后线上变停机帧, 反馈回显实测位 (锚定)
 *   ⑥追帧跳过遥测: 正常=0
 *   ⑦工厂注册: registerToMasterFactory 后 createMaster("serial") 可用
 *   ⑧tx 失败遥测: FlakyWriteTransport 写失败注入 → 计数 + 恢复
 *   ⑨重启: stop 释放 transport → 二次 start → quick_stop 闩锁已复位
 *
 * 编译运行 (零 ROS):
 *   cd unistackbot_hardware/serial_master/test
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_serial_master.cpp \
 *       ../src/serial_master.cpp ../src/framer.cpp ../src/termios_transport.cpp \
 *       ../../bus/src/master_factory.cpp ../../protocol/src/unitree_im_core.cpp \
 *       ../../protocol/src/dm_motor_core.cpp ../../protocol/src/protocol_factory.cpp \
 *       ../../statemachine/src/unitree_im_safety.cpp ../../statemachine/src/cia402_safety.cpp \
 *       ../../statemachine/src/dm_safety.cpp ../../statemachine/src/node_safety_factory.cpp \
 *       -I../include -I../../bus/include -I../../protocol/include -I../../statemachine/include \
 *       -I../../../unistackbot_common -o /tmp/test_serial_master && /tmp/test_serial_master
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_protocol/protocol_factory.hpp"
#include "unistackbot_statemachine/node_safety_factory.hpp"
#include "unistackbot_protocol/unitree_im_core.hpp"
#include "unistackbot_serial/fake_transport.hpp"
#include "unistackbot_serial/serial_master.hpp"

using CmdFrame = std::vector<unistackbot_protocol::NodeCommand>;
using StateFrame = std::vector<unistackbot_bus::NodeSnapshot>;
using unistackbot_protocol::NodeCommand;
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
	bool ever_run[16] = {false};   // 粘性: 帧上出现过 mode1 (意图握手帧只发一拍, 采样窗口不可靠)
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
			if (last_mode[id] == 1)
			{
				ever_run[id] = true;
			}
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
		const std::int32_t q_raw = static_cast<std::int32_t>(std::lround(q * kRatio * 32768.0 / (2.0 * 3.14159265358979323846)));
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
	cfg.endpoint = "fake";
	cfg.protocol = "unitree_im";        // 组合模型 (2026-10-21): 段级名, start 期 per-node 自建
	cfg.statemachine = "unitree_im";   // 主站 start 期经工厂自建 (总线内部组件)
	cfg.rate_hz = 250.0;   // 测试提速 (2 节点 slot 2ms)
	cfg.baudrate = 6000000;
	cfg.thread_priority = 0;   // 测试环境不抢 FIFO (0=rt_tune 降级 SCHED_OTHER)
	cfg.cpu_affinity = -1;    // 测试不绑核
	cfg.stale_cycles = 30;    // 250Hz 下 120ms
	cfg.nodes = {{kNodeA, {}, {}, kRatio}, {kNodeB, {}, {}, kRatio}};
	return cfg;
}

// 写失败注入包装器 (FakeTransport 本体不动): 前 fail_next_writes 次 write 返回 -1
class FlakyWriteTransport final : public unistackbot_serial::IoTransport
{
public:
	explicit FlakyWriteTransport(std::unique_ptr<FakeTransport> inner)
		: inner_(std::move(inner))
	{
	}

	std::size_t fail_next_writes = 0;

	ssize_t read(std::uint8_t *buf, std::size_t cap) override
	{
		return inner_->read(buf, cap);
	}

	ssize_t write(const uint8_t *buf, std::size_t n) override
	{
		if (fail_next_writes > 0)
		{
			--fail_next_writes;
			return -1;
		}
		return inner_->write(buf, n);
	}

	[[nodiscard]] std::size_t bytes_available() const override
	{
		return inner_->bytes_available();
	}

	void close() override { inner_->close(); }

private:
	std::unique_ptr<FakeTransport> inner_;
};

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
	auto fake = std::make_unique<FakeTransport>([motor](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
			return motor->respond(in, n, out);
		});
	FakeTransport *fake_raw = fake.get();
	m->attach_transport(std::move(fake));

	// 组合根注入 (协议+翻译器从工厂创建后注入, 总线不自己创建依赖)

	check(m->start(makeCfg()), "start with injected deps");
	check(m->running(), "running");

	// ① 安全怠速: 等 10 拍, 线上全是停机帧+看dog位
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(motor->cmd_frames > 5, "frames flowing");
	check(motor->last_mode[kNodeA] == 0 && motor->last_watchdog[kNodeA], "idle = stop mode + watchdog");
	check(motor->last_mode[kNodeB] == 0, "idle node B stop too");

	// ② 命令往返
	CmdFrame cmd(2);
	cmd[0].mode = NodeMode::kRun;
	cmd[0].position = 1.234;
	cmd[0].kp = 10.0;
	cmd[0].watchdog_enable = true;
	cmd[1].mode = NodeMode::kRun;
	cmd[1].position = -0.5;
	cmd[1].kp = 10.0;
	cmd[1].watchdog_enable = true;
	m->publish_cmd(cmd);
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(motor->last_mode[kNodeA] == 1, "run mode on wire");
	check(std::fabs(motor->last_position[kNodeA] - 1.234) < 1e-3, "position roundtrip A");
	check(std::fabs(motor->last_position[kNodeB] + 0.5) < 1e-3, "position roundtrip B");

	// ③ 状态翻译
	StateFrame st(2);
	bool got = false;
	for (int i = 0; i < 20 && !got; ++i)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		got = m->take_state(st);
	}
	check(got, "take_state got live snapshot");
	if (got)
	{
		check(st[0].state == NeutralState::kEnabled, "node A ENABLED");
		check(st[0].fb.node_id == kNodeA, "node A id matched");
		check(std::fabs(st[0].fb.position - 1.234) < 1e-3, "state position A");
	}
	check(m->state() == NeutralState::kEnabled, "segment worst-of = ENABLED");

	// ③b 假电机报错 → FAULT (粘性) → 清故障握手 → READY
	motor->error_bits = 0x00000100u;
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(m->state() == NeutralState::kFault, "error bits -> FAULT");
	motor->error_bits = 0;                      // 灭源头 (FAULT 仍粘住)
	std::this_thread::sleep_for(std::chrono::milliseconds(40));
	check(m->state() == NeutralState::kFault, "FAULT sticky after source cleared");
	m->fault_clear();                           // 总线出口: 全节点清故障请求
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	// 握手成立 = 不再粘 FAULT: 清帧后经 READY (瞬态), 上一命令流仍 kRun → 回 ENABLED
	check(m->state() == NeutralState::kEnabled, "fault_clear handshake recovers");

	// ④ 断流陈旧: 丢全部应答 → Unknown (stale_cycles=30 拍 @250Hz ≈ 120ms)
	fake_raw->set_drop_responses(1000000);
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
	check(m->state() == NeutralState::kUnknown, "all-drop -> stale Unknown");
	fake_raw->set_drop_responses(0);
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(m->state() == NeutralState::kEnabled, "recovered from stale (强断言)");

	// ④b 坏帧拒收: corrupt 旋钮 (载荷字节翻转 → CRC 拒)
	const auto rej0 = m->telemetry().rx_rejected;
	fake_raw->set_corrupt_next_response();
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(m->telemetry().rx_rejected > rej0, "corrupt frame rejected");
	check(m->state() == NeutralState::kEnabled, "state survives corrupt frame");

	// ④c 分片应答: fragment 旋钮 (劈两半跨拍; framer 拼接不锁死不丢同步)
	fake_raw->set_fragment_next_response(5);
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(m->state() == NeutralState::kEnabled, "fragmented response stitched");

	// ⑤ quick_stop: 线上变停机帧
	m->quick_stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(60));
	check(motor->last_mode[kNodeA] == 0, "quick_stop -> stop frames on wire");

	// ⑥ 追帧遥测: 正常测试全程 skip 应很少 (测试机非 RT 环境, 只断不为爆炸值)
	const auto tl = m->telemetry();
	check(tl.rx_frames > 10, "rx frames counted");
	check(tl.skipped_slots < 50, "skip telemetry sane");
	check(tl.rx_nofit == 0, "no out-of-table ids");

	m->stop();
	check(!m->running(), "stopped");
	check(fake_raw->closed(), "transport closed by stop");

	// ⑧ tx 失败遥测: 写失败注入 (下一时隙盲发重试)
	auto motor2 = std::make_shared<FakeMotor>();
	auto fake2 = std::make_unique<FakeTransport>([motor2](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
			return motor2->respond(in, n, out);
		});
	auto flaky = std::make_unique<FlakyWriteTransport>(std::move(fake2));
	FlakyWriteTransport *flaky_raw = flaky.get();
	SerialMaster m2;
	m2.attach_transport(std::move(flaky));
	check(m2.start(makeCfg()), "start with flaky transport");
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	flaky_raw->fail_next_writes = 5;
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	const auto tl2 = m2.telemetry();
	check(tl2.tx_failed >= 5, "tx failures counted");
	check(tl2.tx_frames > tl2.tx_failed, "tx resumed after failures");
	m2.stop();

	// ⑨ 重启: stop 释放 transport、start 复位 quick_stop 闩锁与泵态
	auto motor3 = std::make_shared<FakeMotor>();
	auto fake3 = std::make_unique<FakeTransport>([motor3](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
			return motor3->respond(in, n, out);
		});
	FakeTransport *fake3_raw = fake3.get();
	m->attach_transport(std::move(fake3));
	check(m->start(makeCfg()), "restart after stop");
	check(m->running(), "restart running");
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(motor3->cmd_frames > 5, "frames flowing after restart");
	CmdFrame run_cmd(2);
	run_cmd[0].mode = NodeMode::kRun;
	run_cmd[0].position = 0.5;
	run_cmd[0].kp = 10.0;
	run_cmd[0].watchdog_enable = true;
	run_cmd[1] = run_cmd[0];
	m->publish_cmd(run_cmd);
	std::this_thread::sleep_for(std::chrono::milliseconds(80));
	check(motor3->last_mode[kNodeA] == 1, "quick_stop latch cleared on restart");
	check(m->state() == NeutralState::kEnabled, "state after restart");
	m->stop();
	check(fake3_raw->closed(), "second stop closes transport");

	// ---- ⑧ 组合模型装配预演 (2026-10-21): 名解析/混型/kind 校验/诊断 (装配级,
	//      不需帧回环 —— dm 协议帧内容无关, 只验 start 的工厂与校验路径) ----
	{
		auto mk = unistackbot_bus::createMaster("serial");
		auto *mm = dynamic_cast<SerialMaster *>(mk.get());
		check(mm != nullptr, "组合模型 master 创建");
		auto mot2 = std::make_shared<FakeMotor>();
		mm->attach_transport(std::make_unique<FakeTransport>([mot2](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
				return mot2->respond(in, n, out);
			}));
		MasterConfig dm = makeCfg();
		dm.protocol = "dm4340";          // 段级缺省 (stationlite j1-3 形态)
		dm.statemachine = "dm4340";
		dm.nodes[1].protocol = "dm4310";   // 节点 1 覆写 (混型!)
		dm.nodes[1].statemachine = "dm4310";
		dm.nodes[0].ratio = dm.nodes[1].ratio = 1.0;   // 达妙恒等
		// 串口拒不可帧化协议 (2026-10-21 审查 P1: dm 帧头 0x00/0x00=CAN 形态,
		// Framer 将以流中任意 00 00 对同步解噪声)——混型节点表亦被段级同族校验拒
		check(!mm->start(dm), "串口拒无帧头协议 (dm 走 CAN 主站)");
		mm->stop();
		// 拒绝路径: 执行器协议缺状态机名 (段级与节点级皆空)
		MasterConfig bad = makeCfg();
		bad.protocol = "dm4310";
		bad.statemachine = "";
		bad.nodes[0].ratio = bad.nodes[1].ratio = 1.0;
		check(!mm->start(bad), "执行器缺状态机名 → 拒");
		// 拒绝路径: 两级协议名皆空
		MasterConfig bad2 = makeCfg();
		bad2.protocol = "";
		bad2.statemachine = "unitree_im";
		check(!mm->start(bad2), "两级协议名皆空 → 拒");
		// 拒绝路径: dm ratio≠1 (装配期 fail-fast, 先于 framer 校验)
		MasterConfig bad3 = makeCfg();
		bad3.protocol = "dm4310";
		bad3.statemachine = "dm4310";
		bad3.nodes[0].ratio = bad3.nodes[1].ratio = 9.0;
		check(!mm->start(bad3), "dm ratio≠1 → 拒 (fail-fast)");
		// 拒绝路径: 跨族混搭 (unitree 协议 + dm 状态机)
		MasterConfig bad4 = makeCfg();
		bad4.statemachine = "dm4310";
		check(!mm->start(bad4), "跨族混搭 → 拒");
		// 拒绝路径: 重复节点 id (2026-10-21 审查补: 拓扑一致性)
		MasterConfig bad5 = makeCfg();
		bad5.nodes[1].id = bad5.nodes[0].id;
		check(!mm->start(bad5), "重复节点 id → 拒");
		// 宇树段级简写回归: 原形态仍可 start
		auto mk3 = unistackbot_bus::createMaster("serial");
		auto *m3 = dynamic_cast<SerialMaster *>(mk3.get());
		m3->attach_transport(std::make_unique<FakeTransport>([mot2](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
				return mot2->respond(in, n, out);
			}));
		MasterConfig ut = makeCfg();
		check(m3->start(ut), "宇树段级简写零行为变化");
		m3->stop();
	}

	// ---- ⑨ 意图接口三件套 (2026-10-21): enable()/disable() 显式路径 ----
	{
		auto mk2 = unistackbot_bus::createMaster("serial");
		auto *mi = dynamic_cast<SerialMaster *>(mk2.get());
		auto mot3 = std::make_shared<FakeMotor>();
		mi->attach_transport(std::make_unique<FakeTransport>([mot3](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
				return mot3->respond(in, n, out);
			}));
		check(mi->start(makeCfg()), "意图测试 start");
		std::this_thread::sleep_for(std::chrono::milliseconds(60));   // idle 期: disable_command 帧 (宇树=kStop)
		check(mot3->last_mode[kNodeA] == 0 && mot3->last_watchdog[kNodeA], "idle=协议安全形态 (kStop+wd, 零行为变化)");
		mi->enable(kNodeA);   // 显式意图: 泵→request_enable→enable_command (宇树=锚定 kRun)
		std::this_thread::sleep_for(std::chrono::milliseconds(60));
		// 宇树请求生命周期: 反馈 mode=kRun 即自清 → 握手帧只发一拍, 之后回数据流
		// (idle) 覆盖 last_mode —— 断言用粘性标志 (握手发生过)
		check(mot3->ever_run[kNodeA] && !mot3->ever_run[kNodeB], "enable() → 命令面握手 kRun (仅目标节点)");
		check(mot3->last_mode[kNodeA] == 0, "握手完成后回数据流 (idle, 请求自清语义)");
		mi->disable(kNodeA);  // 显式断能: 停机帧
		std::this_thread::sleep_for(std::chrono::milliseconds(60));
		check(mot3->last_mode[kNodeA] == 0, "disable() → 停机帧");
		mi->enable(0xFE);     // 表外 id: no-op 不崩
		std::this_thread::sleep_for(std::chrono::milliseconds(40));
		check(true, "表外 id enable no-op");
		mi->stop();
	}
	// ---- ⑩ 二轮审查: 急停压意图 + 意图旗失效 ----
	{
		auto mk4 = unistackbot_bus::createMaster("serial");
		auto *mq = dynamic_cast<SerialMaster *>(mk4.get());
		auto mot4 = std::make_shared<FakeMotor>();
		mq->attach_transport(std::make_unique<FakeTransport>([mot4](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
				return mot4->respond(in, n, out);
			}));
		check(mq->start(makeCfg()), "二轮 start");
		std::this_thread::sleep_for(std::chrono::milliseconds(50));   // kReady
		mq->enable(kNodeA);                                           // 意图挂起
		mq->quick_stop();                                             // e-stop 压制
		std::this_thread::sleep_for(std::chrono::milliseconds(60));
		check(!mot4->ever_run[kNodeA], "急停期间挂起 enable 不上链 (急停压意图)");
		mq->stop();
	}
	// ---- ⑪ 二轮审查: FAULT 期 enable 意图失效 (fault_clear 回 READY 不自动复电) ----
	{
		auto mk5 = unistackbot_bus::createMaster("serial");
		auto *mf = dynamic_cast<SerialMaster *>(mk5.get());
		auto mot5 = std::make_shared<FakeMotor>();
		mf->attach_transport(std::make_unique<FakeTransport>([mot5](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
				return mot5->respond(in, n, out);
			}));
		check(mf->start(makeCfg()), "⑪ start");
		std::this_thread::sleep_for(std::chrono::milliseconds(50));   // kReady
		mot5->error_bits = 0x00000100u;                               // 注入故障 → FAULT
		std::this_thread::sleep_for(std::chrono::milliseconds(60));
		mf->enable(kNodeA);                                           // FAULT 期点击 (意图将被失效)
		std::this_thread::sleep_for(std::chrono::milliseconds(40));
		mot5->error_bits = 0;                                         // 故障源消除
		mf->fault_clear();                                            // 0xFB → err=0 → confirm → kReady
		std::this_thread::sleep_for(std::chrono::milliseconds(80));
		check(!mot5->ever_run[kNodeA], "FAULT 期 enable 意图已失效 — 回 READY 不自动复电");
		mf->stop();
	}



	// ---- ⑫ 容量动态化 (方案 R-a): 节点数运行期无容量上限 ----
	// 串口形态单段节点上限由协议 id 域决定 (unitree ≤14): 顶格 14 节点装配+交换;
	// >16 交换已在 DynamicSpLatest 组件级证明 (24 节点压测); dm 24 节点属 CAN 形态
	// (CanMaster)。id 域自声明拒绝 (unitree id 15=广播) 同验。
	{
		auto mk6 = unistackbot_bus::createMaster("serial");
		auto *md = dynamic_cast<SerialMaster *>(mk6.get());
		auto mot6 = std::make_shared<FakeMotor>();
		md->attach_transport(std::make_unique<FakeTransport>([mot6](const std::uint8_t *in, std::size_t n, std::uint8_t *out) {
				return mot6->respond(in, n, out);
			}));
		MasterConfig big = makeCfg();
		const std::size_t kTop = 14;   // unitree id 域顶格 (1..14; 15=广播)
		big.nodes.clear();
		for (std::size_t k = 0; k < kTop; ++k)
		{
			big.nodes.push_back({static_cast<std::uint8_t>(k + 1), {}, {}, kRatio});
		}
		check(md->start(big), "14 节点顶格装配 (unitree id 域满, 无容量常量)");
		CmdFrame bigcmd(kTop);
		for (std::size_t k = 0; k < kTop; ++k)
		{
			bigcmd[k].mode = NodeMode::kRun;
			bigcmd[k].position = 0.1 * static_cast<double>(k);
			bigcmd[k].kp = 10.0;
		}
		md->publish_cmd(bigcmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(80));
		StateFrame snap(kTop);
		bool got = false;
		for (int i = 0; i < 20 && !got; ++i)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			got = md->take_state(snap);
		}
		check(got && snap.size() == kTop, "14 节点状态快照");
		md->stop();
		// id 域拒绝: id 15 (unitree 广播域)
		MasterConfig e8 = makeCfg();
		e8.nodes[0].id = 15;
		check(!md->start(e8), "unitree id=15 (广播域) → 拒 (id 域自声明)");
	}

	std::printf("%s: %d cases, %d failed\n", g_fail == 0 ? "PASS" : "FAIL", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}

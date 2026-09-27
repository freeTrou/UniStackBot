#include "unistackbot_serial/serial_master.hpp"

#include <atomic>
#include <cstdio>
#include <system_error>
#include <thread>

#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_protocol/protocol_factory.hpp"
#include "unistackbot_protocol/unitree_im_core.hpp"
#include "unistackbot_serial/framer.hpp"
#include "unistackbot_serial/termios_transport.hpp"
#include "unistackbot_statemachine/state_translator_factory.hpp"

#include "rt_tune/rt_tune.hpp"
#include "sp_latest/sp_latest.hpp"

namespace unistackbot_serial
{
namespace
{

using unistackbot_bus::BusCommand;
using unistackbot_bus::BusState;
using unistackbot_bus::MasterConfig;
using unistackbot_bus::kMaxNodes;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_statemachine::NeutralState;

constexpr int kPumpPrio = 85;    // 串口档 (调度阶梯 EC95>CAN90>串口85)
constexpr int kPumpCpu = 2;      // 总线核
constexpr std::uint64_t kStaleCycles = 50;   // 无反馈判 Unknown 的拍数 (~100ms@500Hz)
constexpr std::int64_t kNsPerSec = 1'000'000'000LL;
constexpr std::size_t kRxBurst = 8;          // 单次排干最多处理帧数
constexpr int kDefaultBaud = 6000000;        // unitree IM 系默认 (MasterConfig 无波特位, 协议侧约定)

std::int64_t clockNowNs()
{
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

void sleepUntilNs(std::int64_t target_ns)
{
	timespec ts{};
	ts.tv_sec = static_cast<time_t>(target_ns / kNsPerSec);
	ts.tv_nsec = static_cast<long>(target_ns % kNsPerSec);
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

}  // namespace

struct SerialMaster::Impl
{
	// ── 装配期 (start() 前注入, 之后只读; stop() 后 io 释放待重装配) ──
	MasterConfig cfg;
	std::unique_ptr<IoTransport> io;
	std::unique_ptr<unistackbot_protocol::ProtocolCore> proto;
	std::unique_ptr<unistackbot_statemachine::StateTranslator> translator;

	// ── 交换通道 (POD 定长; publish/取最新, 零锁) ──
	unistackbot_common::SpLatest<BusCommand> cmd_ch;
	unistackbot_common::SpLatest<BusState> state_ch;

	// ── 线程控制 ──
	std::thread pump;
	std::atomic<bool> stop_flag{false};
	std::atomic<bool> quick_stop_flag{false};
	std::atomic<bool> started{false};

	// ── 泵态 (泵线程唯一写者; start() 复位) ──
	struct PumpState
	{
		Framer framer{unistackbot_protocol::FrameSpec{}};
		NodeFeedback fb[kMaxNodes] = {};
		std::uint64_t last_seen_cycle[kMaxNodes] = {};
		std::uint64_t cycle = 0;
	} pump_;

	// ── 工作缓冲 (随 Impl 一次分配, 泵内只读写; RT 循环零构造零分配) ──
	struct WorkBuf
	{
		BusCommand cmd;         // SpLatest 读出目标
		NodeCommand slot_cmd;   // 当前时隙命令 (quick_stop 改写)
		std::uint8_t tx_frame[64];
		std::uint8_t rx_chunk[256];
		ExtractedFrame rx_frames[kRxBurst];
		NodeFeedback rx_fb;
		BusState snap;          // 快照发布缓冲
	} work_;
};

SerialMaster::SerialMaster() : impl_(new Impl)
{
	BusCommand idle{};   // 安全怠速: 全体停机+看门狗位
	for (std::size_t i = 0; i < kMaxNodes; ++i)
	{
		idle.node[i].mode = unistackbot_protocol::NodeMode::kStop;
		idle.node[i].watchdog_enable = true;
	}
	impl_->cmd_ch.init(idle);
	BusState st{};
	impl_->state_ch.init(st);
}

SerialMaster::~SerialMaster()
{
	stop();
}

void SerialMaster::attach_transport(std::unique_ptr<IoTransport> io)
{
	if (!impl_->started.load() && impl_->pump.joinable() == false)
	{
		impl_->io = std::move(io);
	}
}

bool SerialMaster::start(const MasterConfig &cfg)
{
	if (impl_->started.load())
	{
		return false;
	}
	// 配置校验 (无默认纪律: 非法即拒)
	if (cfg.rate_hz <= 0.0 || cfg.node_count == 0 || cfg.node_count > kMaxNodes)
	{
		return false;
	}
	for (std::size_t i = 0; i < cfg.node_count; ++i)
	{
		if (cfg.node_id[i] > 14 || !(cfg.ratio[i] > 0.0))
		{
			return false;
		}
	}
	impl_->proto = unistackbot_protocol::createProtocolCore(cfg.protocol);
	impl_->translator = unistackbot_statemachine::createStateTranslator(cfg.translator);
	if (impl_->proto == nullptr || impl_->translator == nullptr)
	{
		return false;
	}
	if (impl_->io == nullptr)
	{
		std::string err;
		impl_->io = TermiosTransport::open(cfg.endpoint, kDefaultBaud, err);
		if (impl_->io == nullptr)
		{
			std::fprintf(stderr, "SerialMaster: 打开 %s 失败: %s\n",
				cfg.endpoint.c_str(), err.c_str());
			return false;
		}
	}
	impl_->cfg = cfg;
	resetPumpState();
	impl_->pump_.framer = Framer{impl_->proto->framespec()};
	impl_->stop_flag.store(false);
	impl_->quick_stop_flag.store(false);   // 闩锁不跨生命周期
	try
	{
		impl_->pump = std::thread([this] { pumpMain(); });
	}
	catch (const std::system_error &e)
	{
		std::fprintf(stderr, "SerialMaster: 泵线程创建失败: %s\n", e.what());
		return false;
	}
	impl_->started.store(true);
	running_.store(true);
	return true;
}

void SerialMaster::pumpMain()
{
	unistackbot_common::rt_tune::apply(kPumpCpu, kPumpPrio, 0, "serial_pump");

	const std::int64_t period_ns = static_cast<std::int64_t>(kNsPerSec / impl_->cfg.rate_hz);
	const std::int64_t slot_ns = period_ns / static_cast<std::int64_t>(impl_->cfg.node_count);
	const std::size_t n = impl_->cfg.node_count;
	std::int64_t base = clockNowNs() + period_ns;

	while (!impl_->stop_flag.load())
	{
		for (std::size_t k = 0; k < n && !impl_->stop_flag.load(); ++k)
		{
			// 时隙 k 标称时刻: base + k*period/n (整数分摊, 无舍入累积)
			std::int64_t target = base +
				static_cast<std::int64_t>(k) * period_ns / static_cast<std::int64_t>(n);
			// ── 落后追帧纪律: 逾期超一个时隙即跳未来, 绝不补发 ──
			const std::int64_t behind = clockNowNs() - target;
			if (behind > slot_ns)
			{
				const std::uint64_t skip = static_cast<std::uint64_t>(behind / slot_ns) + 1;
				target += static_cast<std::int64_t>(skip) * slot_ns;
				telemetry_.skipped_slots += skip;   // 单读者近似, 健康签名=0
			}
			sleepUntilNs(target);
			runSlot(k);
		}
		base += period_ns;
		++impl_->pump_.cycle;
		publishSnapshot();
	}
}

void SerialMaster::runSlot(std::size_t k)
{
	resolveSlotCmd(k);

	// 单帧单 write (绝不合并: USB 拼帧 → 线上背靠背碰撞)
	const std::size_t fn = impl_->proto->encode(impl_->work_.tx_frame,
		sizeof(impl_->work_.tx_frame), impl_->work_.slot_cmd,
		impl_->cfg.node_id[k], impl_->cfg.ratio[k]);
	if (fn == 0)
	{
		++telemetry_.tx_failed;
		return;
	}
	const ssize_t written = impl_->io->write(impl_->work_.tx_frame, fn);
	if (written == static_cast<ssize_t>(fn))
	{
		++telemetry_.tx_frames;
	}
	else
	{
		// 写失败/部分写不补写: 下一时隙发最新命令 (盲发语义即重试)
		++telemetry_.tx_failed;
	}

	drainAndDecode();
}

void SerialMaster::resolveSlotCmd(std::size_t k)
{
	uint64_t seq = 0;
	impl_->cmd_ch.read(impl_->work_.cmd, seq);
	impl_->work_.slot_cmd = impl_->work_.cmd.node[k];
	if (impl_->quick_stop_flag.load())
	{
		NodeCommand &stop = impl_->work_.slot_cmd;
		stop.mode = unistackbot_protocol::NodeMode::kStop;
		stop.watchdog_enable = true;
		stop.tau = 0;
		stop.speed = 0;
		stop.kp = 0;
		stop.kd = 0;
		stop.position = impl_->pump_.fb[k].position;   // 锚定实测位
	}
}

void SerialMaster::drainAndDecode()
{
	while (true)
	{
		if (impl_->io->bytes_available() == 0)
		{
			break;
		}
		const ssize_t r = impl_->io->read(impl_->work_.rx_chunk, sizeof(impl_->work_.rx_chunk));
		if (r <= 0)
		{
			break;
		}
		const std::size_t produced = impl_->pump_.framer.push(
			impl_->work_.rx_chunk, static_cast<std::size_t>(r),
			impl_->work_.rx_frames, kRxBurst);
		for (std::size_t f = 0; f < produced; ++f)
		{
			classifyFrame(impl_->work_.rx_frames[f]);
		}
		if (static_cast<std::size_t>(r) < sizeof(impl_->work_.rx_chunk))
		{
			break;   // 排干
		}
	}
}

void SerialMaster::classifyFrame(const ExtractedFrame &frame)
{
	// CRC 与 ratio 无关: 任一节点 ratio 解码一次即完成验帧并取出 node_id
	if (!impl_->proto->decode(frame.bytes, frame.len, impl_->work_.rx_fb,
			impl_->cfg.ratio[0]))
	{
		++telemetry_.rx_rejected;
		return;
	}
	const std::size_t i = findNode(impl_->work_.rx_fb.node_id);
	if (i == impl_->cfg.node_count)
	{
		++telemetry_.rx_nofit;
		return;
	}
	(void)impl_->proto->decode(frame.bytes, frame.len, impl_->work_.rx_fb,
		impl_->cfg.ratio[i]);   // 以本节点 ratio 重解, 物理量按正确减速比换算
	impl_->pump_.fb[i] = impl_->work_.rx_fb;
	impl_->pump_.last_seen_cycle[i] = impl_->pump_.cycle;
	++telemetry_.rx_frames;
}

std::size_t SerialMaster::findNode(std::uint8_t node_id) const
{
	for (std::size_t i = 0; i < impl_->cfg.node_count; ++i)
	{
		if (impl_->cfg.node_id[i] == node_id)
		{
			return i;
		}
	}
	return impl_->cfg.node_count;
}

void SerialMaster::publishSnapshot()
{
	const std::size_t n = impl_->cfg.node_count;
	for (std::size_t i = 0; i < n; ++i)
	{
		impl_->work_.snap.node[i] = impl_->pump_.fb[i];
		impl_->work_.snap.node_state[i] =
			(impl_->pump_.cycle - impl_->pump_.last_seen_cycle[i] > kStaleCycles)
			? NeutralState::kUnknown
			: impl_->translator->map_state(impl_->pump_.fb[i]);
	}
	impl_->work_.snap.seq = impl_->pump_.cycle;
	impl_->state_ch.publish(impl_->work_.snap);
}

void SerialMaster::resetPumpState()
{
	impl_->pump_ = Impl::PumpState{};   // fb/last_seen/cycle 归零; framer 随后由 start() 重设
}

void SerialMaster::stop()
{
	if (!impl_->started.exchange(false))
	{
		return;
	}
	impl_->stop_flag.store(true);
	if (impl_->pump.joinable())
	{
		impl_->pump.join();
	}
	running_.store(false);
	if (impl_->io)
	{
		impl_->io->close();
		impl_->io.reset();   // 重启语义: 释放 transport, 二次 start 重开/重挂
	}
}

void SerialMaster::publish_cmd(const BusCommand &cmd)
{
	impl_->cmd_ch.publish(cmd);
}

bool SerialMaster::take_state(BusState &out)
{
	uint64_t seq = 0;
	return impl_->state_ch.readLastFrame(out, seq) ==
		unistackbot_common::SpLatest<BusState>::FrameKind::kLive;
}

void SerialMaster::quick_stop()
{
	impl_->quick_stop_flag.store(true);
}

NeutralState SerialMaster::state() const
{
	// worst-of 聚合: FAULT > QUICK_STOP > UNKNOWN > ENABLED > READY
	BusState st;
	uint64_t seq = 0;
	impl_->state_ch.read(st, seq);
	NeutralState agg = NeutralState::kReady;
	bool any = false;
	for (std::size_t i = 0; i < impl_->cfg.node_count; ++i)
	{
		const NeutralState s = st.node_state[i];
		if (!any)
		{
			agg = s;
			any = true;
			continue;
		}
		if (s == NeutralState::kFault ||
			(s == NeutralState::kQuickStop && agg != NeutralState::kFault) ||
			(s == NeutralState::kUnknown && agg != NeutralState::kFault &&
				agg != NeutralState::kQuickStop))
		{
			agg = s;
		}
	}
	return any ? agg : NeutralState::kUnknown;
}

bool registerToMasterFactory()
{
	return unistackbot_bus::registerMaster("serial", []() ->
		std::unique_ptr<unistackbot_bus::MasterBase> {
		return std::make_unique<SerialMaster>();
	});
}

}  // namespace unistackbot_serial

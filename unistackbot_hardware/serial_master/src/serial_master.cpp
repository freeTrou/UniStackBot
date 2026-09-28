#include "unistackbot_serial/serial_master.hpp"

#include <atomic>
#include <cstdio>
#include <system_error>
#include <thread>

#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_serial/termios_transport.hpp"

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
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::NodeSafety;
using unistackbot_statemachine::NeutralState;

constexpr std::int64_t kNsPerSec = 1'000'000'000LL;
constexpr std::size_t kRxBurst = 8;   // 单次排干帧数界 (实现常量, 非部署调优)
constexpr double kEnableGripKp = 20.0;   // 使能锁定刚度 (安全默认, 参数化延后)
constexpr double kEnableGripKd = 1.5;    // 使能锁定阻尼

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
	std::thread cycle_thread;
	std::atomic<bool> stop_flag{false};
	std::atomic<bool> quick_stop_flag{false};
	std::atomic<bool> started{false};

	// ── 循环态 (总线线程唯一写者; start() 复位) ──
	struct CycleState
	{
		Framer framer{unistackbot_protocol::FrameSpec{}};
		NodeFeedback fb[kMaxNodes] = {};
		NodeSafety safety[kMaxNodes] = {};   // ← 状态机住总线线程 (零锁)
		std::uint64_t last_seen_cycle[kMaxNodes] = {};
		std::uint64_t cycle = 0;
	} cycle_;

	// ── 工作缓冲 (随 Impl 一次分配, 循环内只读写; RT 零构造零分配) ──
	struct WorkBuf
	{
		BusCommand cmd;
		NodeCommand slot_cmd;
		std::uint8_t tx_frame[64];
		std::uint8_t rx_chunk[256];
		ExtractedFrame rx_frames[kRxBurst];
		NodeFeedback rx_fb;
		BusState snap;
	} work_;
};

SerialMaster::SerialMaster() : impl_(new Impl)
{
	BusCommand idle{};
	for (std::size_t i = 0; i < kMaxNodes; ++i)
	{
		idle.node[i].mode = NodeMode::kStop;
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
	if (!impl_->started.load() && impl_->cycle_thread.joinable() == false)
	{
		impl_->io = std::move(io);
	}
}

void SerialMaster::set_protocol(std::unique_ptr<unistackbot_protocol::ProtocolCore> proto)
{
	if (!impl_->started.load())
	{
		impl_->proto = std::move(proto);
	}
}

void SerialMaster::set_translator(
	std::unique_ptr<unistackbot_statemachine::StateTranslator> translator)
{
	if (!impl_->started.load())
	{
		impl_->translator = std::move(translator);
	}
}

const NodeSafety &SerialMaster::node_safety(std::size_t index) const
{
	return impl_->cycle_.safety[index];
}

bool SerialMaster::start(const MasterConfig &cfg)
{
	if (impl_->started.load())
	{
		return false;
	}
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
	if (cfg.baudrate <= 0 || cfg.thread_priority < 0 || cfg.thread_priority > 99 ||
		cfg.cpu_affinity < -1 || cfg.stale_cycles == 0)
	{
		return false;
	}
	if (impl_->proto == nullptr || impl_->translator == nullptr)
	{
		std::fprintf(stderr, "SerialMaster: start 前须注入 protocol/translator (组合根职责)\n");
		return false;
	}
	if (impl_->io == nullptr)
	{
		std::string err;
		impl_->io = TermiosTransport::open(cfg.endpoint, cfg.baudrate, err);
		if (impl_->io == nullptr)
		{
			std::fprintf(stderr, "SerialMaster: 打开 %s 失败: %s\n",
				cfg.endpoint.c_str(), err.c_str());
			return false;
		}
	}
	impl_->cfg = cfg;
	resetCycleState();
	impl_->cycle_.framer = Framer{impl_->proto->framespec()};
	impl_->stop_flag.store(false);
	impl_->quick_stop_flag.store(false);
	try
	{
		impl_->cycle_thread = std::thread([this] { busLoopMain(); });
	}
	catch (const std::system_error &e)
	{
		std::fprintf(stderr, "SerialMaster: 总线线程创建失败: %s\n", e.what());
		return false;
	}
	impl_->started.store(true);
	running_.store(true);
	return true;
}

void SerialMaster::busLoopMain()
{
	unistackbot_common::rt_tune::apply(impl_->cfg.cpu_affinity,
		impl_->cfg.thread_priority, 0, "serial_bus");

	const std::int64_t period_ns =
		static_cast<std::int64_t>(kNsPerSec / impl_->cfg.rate_hz);
	const std::int64_t slot_ns =
		period_ns / static_cast<std::int64_t>(impl_->cfg.node_count);
	const std::size_t n = impl_->cfg.node_count;
	std::int64_t base = clockNowNs() + period_ns;

	while (!impl_->stop_flag.load())
	{
		for (std::size_t k = 0; k < n && !impl_->stop_flag.load(); ++k)
		{
			std::int64_t target = base +
				static_cast<std::int64_t>(k) * period_ns / static_cast<std::int64_t>(n);
			// ── 追帧跳过纪律: 逾期超一个时隙即跳未来, 绝不补发 ──
			const std::int64_t behind = clockNowNs() - target;
			if (behind > slot_ns)
			{
				const std::uint64_t skip = static_cast<std::uint64_t>(behind / slot_ns) + 1;
				target += static_cast<std::int64_t>(skip) * slot_ns;
				telemetry_.skipped_slots += skip;
			}
			sleepUntilNs(target);
			runSlot(k);
		}
		base += period_ns;
		++impl_->cycle_.cycle;
		publishSnapshot();
	}
}

void SerialMaster::runSlot(std::size_t k)
{
	resolveSlotCmd(k);

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
		++telemetry_.tx_failed;
	}

	drainAndDecode();

	// 状态机更新: 用刚收到的反馈推进 NodeSafety (零延迟——同线程零交接)
	impl_->cycle_.safety[k].update(impl_->cycle_.fb[k]);
}

void SerialMaster::resolveSlotCmd(std::size_t k)
{
	NodeSafety &ns = impl_->cycle_.safety[k];

	// ① 安全状态机优先: 使能/断使能请求 → 生成安全命令
	if (ns.enable_requested())
	{
		impl_->work_.slot_cmd = ns.enable_command(kEnableGripKp, kEnableGripKd);
		ns.clear_requests();
		return;
	}
	if (ns.disable_requested())
	{
		impl_->work_.slot_cmd = ns.disable_command();
		ns.clear_requests();
		return;
	}

	// ② quick_stop 闩锁: 全体停机帧 + 锚定实测位
	if (impl_->quick_stop_flag.load())
	{
		impl_->work_.slot_cmd.mode = NodeMode::kStop;
		impl_->work_.slot_cmd.watchdog_enable = true;
		impl_->work_.slot_cmd.tau = 0;
		impl_->work_.slot_cmd.speed = 0;
		impl_->work_.slot_cmd.kp = 0;
		impl_->work_.slot_cmd.kd = 0;
		impl_->work_.slot_cmd.position = impl_->cycle_.fb[k].position;
		return;
	}

	// ③ 正常命令流: 取最新
	uint64_t seq = 0;
	impl_->cmd_ch.read(impl_->work_.cmd, seq);
	impl_->work_.slot_cmd = impl_->work_.cmd.node[k];
}

void SerialMaster::drainAndDecode()
{
	while (true)
	{
		if (impl_->io->bytes_available() == 0)
		{
			break;
		}
		const ssize_t r = impl_->io->read(impl_->work_.rx_chunk,
			sizeof(impl_->work_.rx_chunk));
		if (r <= 0)
		{
			break;
		}
		const std::size_t produced = impl_->cycle_.framer.push(
			impl_->work_.rx_chunk, static_cast<std::size_t>(r),
			impl_->work_.rx_frames, kRxBurst);
		for (std::size_t f = 0; f < produced; ++f)
		{
			classifyFrame(impl_->work_.rx_frames[f]);
		}
		if (static_cast<std::size_t>(r) < sizeof(impl_->work_.rx_chunk))
		{
			break;
		}
	}
}

void SerialMaster::classifyFrame(const ExtractedFrame &frame)
{
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
		impl_->cfg.ratio[i]);
	impl_->cycle_.fb[i] = impl_->work_.rx_fb;
	impl_->cycle_.last_seen_cycle[i] = impl_->cycle_.cycle;
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
		impl_->work_.snap.node[i] = impl_->cycle_.fb[i];
		impl_->work_.snap.node_state[i] =
			(impl_->cycle_.cycle - impl_->cycle_.last_seen_cycle[i] > impl_->cfg.stale_cycles)
			? NeutralState::kUnknown
			: impl_->cycle_.safety[i].state();
	}
	impl_->work_.snap.seq = impl_->cycle_.cycle;
	impl_->state_ch.publish(impl_->work_.snap);
}

void SerialMaster::resetCycleState()
{
	impl_->cycle_ = Impl::CycleState{};
}

void SerialMaster::stop()
{
	if (!impl_->started.exchange(false))
	{
		return;
	}
	impl_->stop_flag.store(true);
	if (impl_->cycle_thread.joinable())
	{
		impl_->cycle_thread.join();
	}
	running_.store(false);
	if (impl_->io)
	{
		impl_->io->close();
		impl_->io.reset();
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

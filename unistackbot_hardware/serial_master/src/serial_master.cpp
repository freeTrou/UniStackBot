#include "unistackbot_serial/serial_master.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
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
constexpr int kDefaultBaud = 6000000;        // unitree IM 系默认 (MasterConfig 无波特位, 协议侧约定)

double clockNowNs()
{
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<double>(ts.tv_sec) * 1.0e9 + static_cast<double>(ts.tv_nsec);
}

void sleepUntilNs(double target_ns)
{
	timespec ts{};
	ts.tv_sec = static_cast<time_t>(target_ns / 1.0e9);
	ts.tv_nsec = static_cast<long>(target_ns - static_cast<double>(ts.tv_sec) * 1.0e9);
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

}  // namespace

struct SerialMaster::Impl
{
	// ── 装配期定 (start 前/后不变) ──
	MasterConfig cfg;
	std::unique_ptr<IoTransport> io;
	std::unique_ptr<unistackbot_protocol::ProtocolCore> proto;
	std::unique_ptr<unistackbot_statemachine::StateTranslator> translator;

	// ── 交换通道 (POD 定长; publish/取最新, 零锁) ──
	unistackbot_common::SpLatest<BusCommand> cmd_ch;
	unistackbot_common::SpLatest<BusState> state_ch;

	// ── 泵线程 ──
	std::thread pump;
	std::atomic<bool> stop_flag{false};
	std::atomic<bool> quick_stop_flag{false};
	std::atomic<bool> started{false};
	Framer framer{unistackbot_protocol::FrameSpec{}};

	// ── 泵侧私有 (单写者线程; 全部预分配, RT 循环零构造零分配) ──
	NodeFeedback fb[kMaxNodes] = {};
	NeutralState node_state[kMaxNodes] = {};
	std::uint64_t last_seen_cycle[kMaxNodes] = {};
	std::uint64_t cycle = 0;

	// 循环体工作缓冲 (start 前分配一次, pumpMain 内只读写)
	BusCommand cmd_buf;                          // ~800B: SpLatest 读出目标
	NodeCommand out_buf;                         // 单槽命令 (quick_stop 改写用)
	std::uint8_t tx_frame[64];                   // 单帧编码缓冲
	std::uint8_t rx_chunk[256];                  // 非阻塞读缓冲
	ExtractedFrame rx_frames[8];                 // framer 吐出帧
	NodeFeedback rx_fb;                          // decode 工作结构
	BusState snap_buf;                           // ~1.2KB: 快照发布缓冲
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
		impl_->io = TermiosTransport::open(cfg.endpoint, static_cast<int>(kDefaultBaud), err);
		if (impl_->io == nullptr)
		{
			std::fprintf(stderr, "SerialMaster: 打开 %s 失败: %s\n",
				cfg.endpoint.c_str(), err.c_str());
			return false;
		}
	}
	impl_->cfg = cfg;
	impl_->framer = Framer{impl_->proto->framespec()};
	impl_->stop_flag.store(false);
	impl_->started.store(true);
	running_.store(true);

	impl_->pump = std::thread([this] { pumpMain(); });
	return true;
}

void SerialMaster::pumpMain()
{
	unistackbot_common::rt_tune::apply(kPumpCpu, kPumpPrio, 0, "serial_pump");

	const double period_ns = 1.0e9 / impl_->cfg.rate_hz;
	const double slot_ns = period_ns / static_cast<double>(impl_->cfg.node_count);
	const std::size_t n = impl_->cfg.node_count;
	double base = clockNowNs() + period_ns;

	while (!impl_->stop_flag.load())
	{
		for (std::size_t k = 0; k < n && !impl_->stop_flag.load(); ++k)
		{
			double target = base + static_cast<double>(k) * slot_ns;
			// ── 落后追帧纪律: 逾期超一个时隙即跳未来, 绝不补发 ──
			const double behind = clockNowNs() - target;
			if (behind > slot_ns)
			{
				const std::uint64_t skip = static_cast<std::uint64_t>(behind / slot_ns) + 1;
				target += static_cast<double>(skip) * slot_ns;
				telemetry_.skipped_slots += skip;   // 单读者近似, 健康签名=0
			}
			sleepUntilNs(target);

			// ── 取最新命令 (预分配缓冲; quick_stop 置位 → 改发停机帧, 闩锁) ──
			uint64_t seq = 0;
			impl_->cmd_ch.read(impl_->cmd_buf, seq);
			impl_->out_buf = impl_->cmd_buf.node[k];
			if (impl_->quick_stop_flag.load())
			{
				impl_->out_buf.mode = unistackbot_protocol::NodeMode::kStop;
				impl_->out_buf.watchdog_enable = true;
				impl_->out_buf.tau = 0;
				impl_->out_buf.speed = 0;
				impl_->out_buf.kp = 0;
				impl_->out_buf.kd = 0;
				impl_->out_buf.position = impl_->fb[k].position;   // 锚定实测位
			}

			// ── 单帧单 write (绝不合并: USB 拼帧 → 线上背靠背碰撞) ──
			const std::size_t fn = impl_->proto->encode(impl_->tx_frame,
				sizeof(impl_->tx_frame), impl_->out_buf,
				impl_->cfg.node_id[k], impl_->cfg.ratio[k]);
			if (fn > 0 && impl_->io->write(impl_->tx_frame, fn) == static_cast<ssize_t>(fn))
			{
				++telemetry_.tx_frames;
			}

			// ── 非阻塞排干读 → framer → decode (按反馈 node_id 配对) ──
			drainAndDecode();
		}
		base += period_ns;
		++impl_->cycle;

		// ── 快照发布 (预分配缓冲; 含 stale 判定: 无反馈拍数超界 → Unknown) ──
		for (std::size_t i = 0; i < n; ++i)
		{
			impl_->snap_buf.node[i] = impl_->fb[i];
			if (impl_->cycle - impl_->last_seen_cycle[i] > kStaleCycles)
			{
				impl_->snap_buf.node_state[i] = NeutralState::kUnknown;
			}
			else
			{
				impl_->snap_buf.node_state[i] = impl_->translator->map_state(impl_->fb[i]);
			}
		}
		impl_->snap_buf.seq = impl_->cycle;
		impl_->state_ch.publish(impl_->snap_buf);
	}
}

void SerialMaster::drainAndDecode()
{
	while (true)
	{
		const std::size_t avail = impl_->io->bytes_available();
		if (avail == 0)
		{
			break;
		}
		const ssize_t r = impl_->io->read(impl_->rx_chunk, sizeof(impl_->rx_chunk));
		if (r <= 0)
		{
			break;
		}
		const std::size_t produced = impl_->framer.push(
			impl_->rx_chunk, static_cast<std::size_t>(r),
			impl_->rx_frames, sizeof(impl_->rx_frames) / sizeof(impl_->rx_frames[0]));
		for (std::size_t f = 0; f < produced; ++f)
		{
			const std::size_t n = impl_->cfg.node_count;
			bool decoded = false;
			for (std::size_t i = 0; i < n; ++i)
			{
				if (impl_->proto->decode(impl_->rx_frames[f].bytes, impl_->rx_frames[f].len,
					impl_->rx_fb, impl_->cfg.ratio[i]))
				{
					if (impl_->rx_fb.node_id == impl_->cfg.node_id[i])
					{
						impl_->fb[i] = impl_->rx_fb;
						impl_->last_seen_cycle[i] = impl_->cycle;
						decoded = true;
						++telemetry_.rx_frames;
						break;
					}
				}
			}
			if (!decoded)
			{
				++telemetry_.rx_rejected;
			}
		}
		if (static_cast<std::size_t>(r) < sizeof(impl_->rx_chunk))
		{
			break;   // 排干
		}
	}
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

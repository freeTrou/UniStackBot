#include "unistackbot_serial/serial_master.hpp"

#include "unistackbot_statemachine/node_safety_factory.hpp"

#include <atomic>
#include <cstdio>
#include <system_error>
#include <thread>

#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_protocol/protocol_factory.hpp"
#include "unistackbot_serial/termios_transport.hpp"

#include "rt_tune/rt_tune.hpp"
#include "sp_latest/dynamic_sp_latest.hpp"

namespace unistackbot_serial
{
namespace
{

using unistackbot_bus::MasterConfig;
using unistackbot_bus::NodeSnapshot;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::NodeSafety;
using unistackbot_statemachine::NeutralState;

constexpr std::int64_t kNsPerSec = 1'000'000'000LL;
constexpr std::size_t kRxBurst = 8;   // 单次排干帧数界 (实现常量, 非部署调优)

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
	// ── 装配期 (start() 前注入 io, 之后只读; stop() 后 io 释放待重装配) ──
	MasterConfig cfg;
	std::unique_ptr<IoTransport> io;
	// 协议内芯 per-node (2026-10-21 组合模型: start 期按节点表经工厂自建;
	// vector = 数量运行期, 装配期 resize, 循环期只读——RT 零分配)
	std::vector<std::unique_ptr<unistackbot_protocol::ProtocolCore>> proto;

	// ── 交换通道 (DynamicSpLatest: 装配期定容动态双缓冲 seqlock; publish/取最新零锁零分配) ──
	unistackbot_common::DynamicSpLatest<unistackbot_protocol::NodeCommand> cmd_ch;
	unistackbot_common::DynamicSpLatest<NodeSnapshot> state_ch;

	// ── 线程控制 ──
	std::thread cycle_thread;
	std::atomic<bool> stop_flag{false};
	std::atomic<bool> quick_stop_flag{false};
	std::atomic<bool> fault_clear_flag{false};   // 清故障请求闩锁 (全节点; 全清后自熄)
	// 意图请求闩锁 (per-node, 单原子状态 2026-10-21 审查 P2: 双旗交叉清除存在
	// 丢失窗口且每拍双 RMW; 0=无 1=enable 2=disable, 后写胜出): 跨线程置位,
	// 泵下拍转发给状态机 request_* (同线程, 守卫内建), 转发后清
	std::unique_ptr<std::atomic<std::uint8_t>[]> intent;   // atomic 不可移动, 原生数组 (start 期 new[n])
	std::atomic<bool> started{false};

	// ── 循环态 (总线线程唯一写者; start() 复位) ──
	struct CycleState
	{
		Framer framer{unistackbot_protocol::FrameSpec{}};
		std::vector<NodeFeedback> fb;
		std::vector<std::unique_ptr<NodeSafety>> safety;   // 状态机住总线线程 (零锁;
		                                                   // start 期经工厂自建, 循环期不换)
		std::vector<std::uint64_t> last_seen_cycle;
		std::uint64_t cycle = 0;
		uint64_t last_cmd_seq = 0;      // 读门控: cmd_ch.seq() 变化才全量 read (RT 纪律,
		                                // 全量 seqlock 读从每时隙降为每命令)
	} cycle_;

	// ── 工作缓冲 (随 Impl 一次分配, 循环内只读写; RT 零构造零分配) ──
	struct WorkBuf
	{
		std::vector<unistackbot_protocol::NodeCommand> cmd;   // 装配期 resize (start)
		unistackbot_protocol::NodeCommand slot_cmd;
		std::uint8_t tx_frame[64];
		std::uint8_t rx_chunk[256];
		ExtractedFrame rx_frames[kRxBurst];
		NodeFeedback rx_fb;
		std::vector<NodeSnapshot> snap;                       // 装配期 resize (start)
	} work_;
};

SerialMaster::SerialMaster() : impl_(new Impl)
{
	// 交换通道 init 移至 start() (装配期定容 = 节点数; 组合模型 R-a)。
	// DynamicSpLatest init 占位帧 (NSDMI: kStop) 帧内容不载货——idle 形态走
	// idle_command (resolveSlotCmd ③), init 帧仅提供 seq 基线判 "命令流是否开始"。
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

const NodeSafety &SerialMaster::node_safety(std::size_t index) const
{
	return *impl_->cycle_.safety[index];
}

bool SerialMaster::start(const MasterConfig &cfg)
{
	if (impl_->started.load())
	{
		return false;
	}
	if (cfg.rate_hz <= 0.0 || cfg.nodes.empty())
	{
		return false;
	}

	if (cfg.baudrate <= 0 || cfg.thread_priority < 0 || cfg.thread_priority > 99 || cfg.cpu_affinity < -1 || cfg.stale_cycles == 0)
	{
		return false;
	}
	impl_->cfg = cfg;
	resetCycleState();
	// ── 组合模型装配 (2026-10-21): 名字解析 (节点级覆写→段级回退→皆空拒) +
	// 五重校验 (名存在/ratio 语义/kind 交叉/族配对/状态机存在) + 双工厂 per-node
	// 构建 + 失败清理 + 诊断带节点索引; 校验先于 io open (配置错误不碰硬件)。
	// 逻辑私有于本主站——提炼共享挂起, 触发器 = CanMaster 落地 (最小集纪律:
	// 第二个消费者出现才动, bus 包保持纯契约)。
	const std::size_t n = cfg.nodes.size();
	impl_->proto.clear();               // unique_ptr vector: resize/clear 语义 (fill-assign 不可拷贝)
	impl_->proto.resize(n);
	impl_->cycle_.safety.clear();
	impl_->cycle_.safety.resize(n);
	impl_->cycle_.fb.assign(n, NodeFeedback{});
	impl_->cycle_.last_seen_cycle.assign(n, 0);
	impl_->work_.cmd.assign(n, unistackbot_protocol::NodeCommand{});
	impl_->work_.snap.assign(n, NodeSnapshot{});
	impl_->intent.reset(new std::atomic<std::uint8_t>[n]);   // 默认构造 = 零值
	const auto failCleanup = [this, n]() {
		for (std::size_t k = 0; k < n; ++k)
		{
			impl_->proto[k].reset();
			impl_->cycle_.safety[k].reset();
		}
	};
	// 重复 id 检查 (拓扑一致性, 2026-10-21 审查补: 重复 id 下 findNode 首中,
	// 第二节点静默不可达无诊断——装配期拒绝)
	for (std::size_t i = 1; i < cfg.nodes.size(); ++i)
	{
		for (std::size_t j = 0; j < i; ++j)
		{
			if (cfg.nodes[i].id == cfg.nodes[j].id)
			{
				std::fprintf(stderr, "SerialMaster: 节点 #%zu 与 #%zu id 重复 (0x%02X)\n", i, j, cfg.nodes[i].id);
				return false;
			}
		}
	}
	for (std::size_t i = 0; i < n; ++i)
	{
		const unistackbot_bus::NodeSpec &spec = cfg.nodes[i];
		const std::string pname = !spec.protocol.empty() ? spec.protocol : cfg.protocol;
		const std::string sname = !spec.statemachine.empty() ? spec.statemachine : cfg.statemachine;
		if (pname.empty())
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 缺协议名 (节点级与段级皆空, 无默认纪律)\n", i, spec.id);
			return false;
		}
		impl_->proto[i] = unistackbot_protocol::createProtocolCore(pname);
		if (impl_->proto[i] == nullptr)
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 未知协议 '%s' (可选: ", i, spec.id, pname.c_str());
			for (const auto &n : unistackbot_protocol::availableProtocolCores())
			{
				std::fprintf(stderr, "%s ", n.c_str());
			}
			std::fprintf(stderr, ")\n");
			failCleanup();
			return false;
		}
		if (!impl_->proto[i]->ratio_valid(spec.ratio))
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) ratio=%g 不符协议 '%s' 语义 (dm=恒1.0/unitree=>0)\n", i, spec.id, spec.ratio, pname.c_str());
			failCleanup();
			return false;
		}
		if (!impl_->proto[i]->id_valid(spec.id))
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 超出协议 '%s' id 域\n", i, spec.id, pname.c_str());
			failCleanup();
			return false;
		}
		const bool actuator = impl_->proto[i]->kind() == unistackbot_protocol::ProtocolKind::kActuator;
		if (!actuator)
		{
			// 传感器节点: 泵侧调度未实现 (触发器=第一个真实传感器), 拒绝装配——
			// 诚实 fail-fast 优于带雷运行 (2026-10-21 审查 P0)
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 协议 '%s' 为传感器——传感器调度未实现, 当前拒绝\n", i, spec.id, pname.c_str());
			failCleanup();
			return false;
		}
		if (sname.empty())
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 协议 '%s' 为执行器, 缺状态机名 (两级皆空, 可选: ", i, spec.id, pname.c_str());
			for (const auto &n : unistackbot_statemachine::availableNodeSafetyMachines())
			{
				std::fprintf(stderr, "%s ", n.c_str());
			}
			std::fprintf(stderr, ")\n");
			failCleanup();
			return false;
		}
		// 族校验 (key 去数字尾缀得族名): 拦 "unitree_im 协议 + dm 状态机" 跨族
		// 混搭 (语义杂交机故障检测全错); 族内档位混搭 (dm4340/dm4310) 合法
		const auto family = [](const std::string &key) {
			std::size_t end = key.size();
			while (end > 0 && key[end - 1] >= '0' && key[end - 1] <= '9')
			{
				--end;
			}
			return key.substr(0, end);
		};
		if (family(sname) != family(pname))
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 状态机 '%s' 与协议 '%s' 跨族混搭\n", i, spec.id, sname.c_str(), pname.c_str());
			failCleanup();
			return false;
		}
		impl_->cycle_.safety[i] = unistackbot_statemachine::createNodeSafety(sname);
		if (impl_->cycle_.safety[i] == nullptr)
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X) 未知安全状态机 '%s' (可选: ", i, spec.id, sname.c_str());
			for (const auto &n : unistackbot_statemachine::availableNodeSafetyMachines())
			{
				std::fprintf(stderr, "%s ", n.c_str());
			}
			std::fprintf(stderr, ")\n");
			failCleanup();
			return false;
		}
	}
	// 段级同族校验 (2026-10-21 二轮审查: 族校验原为 per-node, unitree 段加 dm 节点
	// 全 gate 通过但 Framer/proto[0] 结构性服务不了——节点静默 kUnknown 无诊断)。
	// 串口段 = 单协议族 (混族属 CAN 帧头路由的形态, 触发器=CanMaster)
	const auto familyOf = [](const std::string &key) {
		std::size_t end = key.size();
		while (end > 0 && key[end - 1] >= '0' && key[end - 1] <= '9')
		{
			--end;
		}
		return key.substr(0, end);
	};
	for (std::size_t i = 1; i < cfg.nodes.size(); ++i)
	{
		const std::string pi = !cfg.nodes[i].protocol.empty() ? cfg.nodes[i].protocol : cfg.protocol;
		if (familyOf(pi) != familyOf(!cfg.nodes[0].protocol.empty() ? cfg.nodes[0].protocol : cfg.protocol))
		{
			std::fprintf(stderr, "SerialMaster: 节点 #%zu (id 0x%02X, 协议 '%s') 与节点 #0 跨族——串口段须单协议族\n", i, cfg.nodes[i].id, pi.c_str());
			failCleanup();
			return false;
		}
	}
	// 交换通道装配期定容 (方案 R-a: 节点数=容量; init 占位帧供 frameKind 判流起点)
	impl_->cmd_ch.init(n, unistackbot_protocol::NodeCommand{});
	impl_->state_ch.init(n, NodeSnapshot{});
	// 串口可帧化校验 (2026-10-21 审查 P1): 帧头 0x00/0x00 = 协议无帧头 (CAN 形态,
	// 如 dm_motor)——Framer 将以字节流中任意 00 00 对同步解噪声, 串口段必须拒绝
	const auto spec = impl_->proto[0]->framespec();
	if (spec.header0 == 0 && spec.header1 == 0)
	{
		std::fprintf(stderr, "SerialMaster: 段级协议帧头为 0x00/0x00 (无帧头, CAN 形态) —— 串口不可帧化, 该协议走 CAN 主站\n");
		for (std::size_t i = 0; i < n; ++i)
		{
			impl_->proto[i].reset();
			impl_->cycle_.safety[i].reset();
		}
		return false;
	}
	impl_->cycle_.framer = Framer{spec};
	if (impl_->io == nullptr)
	{
		std::string err;
		impl_->io = TermiosTransport::open(cfg.endpoint, cfg.baudrate, err);
		if (impl_->io == nullptr)
		{
			std::fprintf(stderr, "SerialMaster: 打开 %s 失败: %s\n", cfg.endpoint.c_str(), err.c_str());
			return false;
		}
	}
	impl_->stop_flag.store(false);
	impl_->quick_stop_flag.store(false);
	impl_->fault_clear_flag.store(false);
	for (std::size_t k = 0; k < n; ++k)
	{
		impl_->intent[k].store(0);
	}
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
	unistackbot_common::rt_tune::apply(impl_->cfg.cpu_affinity, impl_->cfg.thread_priority, 0, "serial_bus");

	const std::int64_t period_ns = static_cast<std::int64_t>(kNsPerSec / impl_->cfg.rate_hz);
	const std::int64_t slot_ns = period_ns / static_cast<std::int64_t>(impl_->cfg.nodes.size());
	const std::size_t n = impl_->cfg.nodes.size();
	std::int64_t base = clockNowNs() + period_ns;

	while (!impl_->stop_flag.load())
	{
		// 清故障闩锁: 对 FAULT 态节点置请求; 全部清完 (无人 FAULT) 自动熄灭
		// —— 新故障须再次 fault_clear() 显式请求, 不自动清 (防复撞)
		if (impl_->fault_clear_flag.load())
		{
			bool any_fault = false;
			for (std::size_t k = 0; k < n; ++k)
			{
				if (impl_->cycle_.safety[k]->state() == NeutralState::kFault)
				{
					impl_->cycle_.safety[k]->request_fault_clear();
					any_fault = true;
				}
			}
			if (!any_fault)
			{
				impl_->fault_clear_flag.store(false);
			}
		}

		// 意图请求转发 (单原子 → 状态机 request, 同线程)。守卫被拒时**保持置位**
		// (kUnknown 窗口的 enable 不消费即失——bringup 时序洞, 2026-10-21 审查 P0;
		// 状态机受理后请求由其生命周期自持, 闩锁清零由泵观察 state 离开 Unknown 实现)。
		// **急停压制**: e-stop 挂起期间不转发不消费 (旗已在 quick_stop() 失效——
		// 期间新到的意图亦挂起, 重启 (旗清) 后按最后意图动作)
		if (!impl_->quick_stop_flag.load())
		{
		for (std::size_t k = 0; k < n; ++k)
		{
			std::uint8_t want = impl_->intent[k].load();
			if (want == 0)
			{
				continue;
			}
			const NeutralState st = impl_->cycle_.safety[k]->state();
			// FAULT 入口失效 enable 意图 (2026-10-21 二轮审查: FAULT 期点击的 enable
			// 若存活到 fault_clear 回 READY 会自动复电——"维护后惊喜上电"危害;
			// 状态机自身故障入口已撤 enable_req_, 泵侧闩锁同语义对齐); 重新使能须
			// 故障清除后显式再点
			if (want == 1 && st == NeutralState::kFault)
			{
				impl_->intent[k].store(0);
				continue;
			}
			want = impl_->intent[k].load();   // 上行可能已清, 重读防 TOCTOU
			if (want == 0)
			{
				continue;
			}
			if (want == 1)
			{
				if (st == NeutralState::kReady)
				{
					impl_->cycle_.safety[k]->request_enable();
					impl_->intent[k].store(0);
				}
			}
			else if (st != NeutralState::kUnknown)
			{
				impl_->cycle_.safety[k]->request_disable();
				impl_->intent[k].store(0);
			}
		}
		}

		for (std::size_t k = 0; k < n && !impl_->stop_flag.load(); ++k)
		{
			std::int64_t target = base + static_cast<std::int64_t>(k) * period_ns / static_cast<std::int64_t>(n);
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

	const std::size_t fn = impl_->proto[k]->encode(impl_->work_.tx_frame, sizeof(impl_->work_.tx_frame), impl_->work_.slot_cmd, impl_->cfg.nodes[k].id, impl_->cfg.nodes[k].ratio);
	if (fn == 0)
	{
		// encode 拒收 (非有限输入等): 计 tx_failed 但**不 return**——反馈排空与状态机
		// 更新必须继续 (2026-10-21 审查 P0: 早退曾饿死安全机, 总线"看着活着")
		++telemetry_.tx_failed;
	}
	else
	{
		const ssize_t written = impl_->io->write(impl_->work_.tx_frame, fn);
		if (written == static_cast<ssize_t>(fn))
		{
			++telemetry_.tx_frames;
		}
		else
		{
			++telemetry_.tx_failed;
		}
	}

	drainAndDecode();

	// 状态机更新: 用刚收到的反馈推进 NodeSafety (零延迟——同线程零交接)
	NodeSafety &ns = *impl_->cycle_.safety[k];
	ns.update(impl_->cycle_.fb[k]);
	// 清故障握手确认: 请求在挂 + 反馈已干净 → confirm (FAULT → READY, 守卫内建)
	if (ns.fault_clear_requested() && ns.last_feedback().error_raw == 0u)
	{
		ns.confirm_fault_cleared();
	}
}

void SerialMaster::resolveSlotCmd(std::size_t k)
{
	NodeSafety &ns = *impl_->cycle_.safety[k];

	// ⓪-top 急停压一切 (2026-10-21 二轮审查: 原 ② 位序使 pending 意图/清故障帧
	// 在 e-stop 期间照发——使能帧在急停中重新上电, 违 "急停=原子旗全段立即" 契约)
	if (impl_->quick_stop_flag.load())
	{
		impl_->work_.slot_cmd = ns.safety_stop_command();
		return;
	}

	// ⓪ 清故障请求挂起 → 发协议清帧 (unitree: mode 6 + tor 魔数, codec 强制)
	if (ns.fault_clear_requested())
	{
		impl_->work_.slot_cmd = unistackbot_protocol::NodeCommand{};
		impl_->work_.slot_cmd.mode = NodeMode::kClear;
		impl_->work_.slot_cmd.watchdog_enable = true;
		return;
	}

	// ① 安全状态机优先: 使能/断使能请求 → 生成安全命令。请求生命周期机器自持
	//    (确认即清) —— CiA402 的使能是多步链, 请求须活到链走完, 泵不代清
	//    (单帧协议如 unitree 的机器也在 update 确认时自清, 行为不变)。
	if (ns.enable_requested())
	{
		impl_->work_.slot_cmd = ns.enable_command();
		return;
	}
	if (ns.disable_requested())
	{
		impl_->work_.slot_cmd = ns.disable_command();
		return;
	}

	// ② 正常命令流: 读门控 (seq 轮询零拷贝, 变化才全量 read——全量 seqlock 读从
	// 每时隙降为每命令, RT 纪律)。**命令流未开始** (frameKind==kInit, SpLatest
	// 原生判据——seq 基线竞态与重启丢保持两坑根治: 首命令无论早于/晚于线程启动
	// 一律判 live; stop/start 重启后通道保留 seq 仍 live = 最后命令保持语义回归)
	// = 安全怠速, 帧形态取 idle_command (协议驻留形态; 达妙=锚定阻尼 MIT 维持
	// 使能与保持力, 堵 enable→idle 失能→命令全忽略链)
	if (impl_->cmd_ch.seq() != impl_->cycle_.last_cmd_seq)
	{
		uint64_t rseq = 0;
		impl_->cmd_ch.read(impl_->work_.cmd.data(), impl_->work_.cmd.size(), rseq);
		impl_->cycle_.last_cmd_seq = rseq;
	}
	if (impl_->cmd_ch.frameKind(impl_->cycle_.last_cmd_seq) != impl_->cmd_ch.FrameKind::kLive)
	{
		impl_->work_.slot_cmd = ns.idle_command();
		return;
	}
	impl_->work_.slot_cmd = impl_->work_.cmd[k];
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
		const std::size_t produced = impl_->cycle_.framer.push(impl_->work_.rx_chunk, static_cast<std::size_t>(r), impl_->work_.rx_frames, kRxBurst);
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
	if (!impl_->proto[0]->decode(frame.bytes, frame.len, impl_->work_.rx_fb, impl_->cfg.nodes[0].ratio))   // 探路取 node_id 定槽 (契约: id 解码不依赖 ratio)
	{
		++telemetry_.rx_rejected;
		return;
	}
	const std::size_t i = findNode(impl_->work_.rx_fb.node_id);
	if (i == impl_->cfg.nodes.size())
	{
		++telemetry_.rx_nofit;
		return;
	}
	(void)impl_->proto[i]->decode(frame.bytes, frame.len, impl_->work_.rx_fb, impl_->cfg.nodes[i].ratio);   // 槽位协议+ratio 正式解码覆写
	impl_->cycle_.fb[i] = impl_->work_.rx_fb;
	impl_->cycle_.last_seen_cycle[i] = impl_->cycle_.cycle;
	++telemetry_.rx_frames;
}

std::size_t SerialMaster::findNode(std::uint8_t node_id) const
{
	for (std::size_t i = 0; i < impl_->cfg.nodes.size(); ++i)
	{
		if (impl_->cfg.nodes[i].id == node_id)
		{
			return i;
		}
	}
	return impl_->cfg.nodes.size();
}

void SerialMaster::publishSnapshot()
{
	const std::size_t n = impl_->cfg.nodes.size();
	for (std::size_t i = 0; i < n; ++i)
	{
		impl_->work_.snap[i].fb = impl_->cycle_.fb[i];
		impl_->work_.snap[i].state =
			(impl_->cycle_.cycle - impl_->cycle_.last_seen_cycle[i] > impl_->cfg.stale_cycles)
			? NeutralState::kUnknown
			: impl_->cycle_.safety[i]->state();
	}
	impl_->state_ch.publish(impl_->work_.snap);
}

void SerialMaster::resetCycleState()
{
	// 成员级复位 (CycleState 含 vector<unique_ptr>, 整体值赋已 deleted) ——
	// POD 部分置零, 资源 vector 由 start() 装配段显式 assign/清理
	impl_->cycle_.framer = Framer{unistackbot_protocol::FrameSpec{}};
	impl_->cycle_.fb.clear();
	impl_->cycle_.safety.clear();
	impl_->cycle_.last_seen_cycle.clear();
	impl_->cycle_.cycle = 0;
	impl_->cycle_.last_cmd_seq = 0;
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

void SerialMaster::publish_cmd(const unistackbot_protocol::NodeCommand *cmds, std::size_t n)
{
	impl_->cmd_ch.publish(cmds, n);
}

bool SerialMaster::take_state(std::vector<NodeSnapshot> &out)
{
	uint64_t seq = 0;
	return impl_->state_ch.readLastFrame(out.data(), out.size(), seq) ==
	       unistackbot_common::DynamicSpLatest<NodeSnapshot>::FrameKind::kLive;
}

void SerialMaster::quick_stop()
{
	impl_->quick_stop_flag.store(true);
	// 意图旗失效 (2026-10-21 二轮审查: 挂起的 enable 意图若存活, e-stop 期间
	// 状态机回 kReady 时会被转发——急停被静默撤销): 急停后重新使能须显式重发
	for (std::size_t k = 0; k < impl_->cfg.nodes.size(); ++k)
	{
		impl_->intent[k].store(0);
	}
}

void SerialMaster::enable(std::uint8_t node_id)
{
	for (std::size_t k = 0; k < impl_->cfg.nodes.size(); ++k)
	{
		if (impl_->cfg.nodes[k].id == node_id)
		{
			impl_->intent[k].store(1);   // 后写胜出 (单原子, 无交叉清除丢失窗口)
			return;
		}
	}
}

void SerialMaster::disable(std::uint8_t node_id)
{
	for (std::size_t k = 0; k < impl_->cfg.nodes.size(); ++k)
	{
		if (impl_->cfg.nodes[k].id == node_id)
		{
			impl_->intent[k].store(2);
			return;
		}
	}
}

void SerialMaster::fault_clear()
{
	impl_->fault_clear_flag.store(true);
}

NeutralState SerialMaster::state() const
{
	std::vector<NodeSnapshot> st(impl_->cfg.nodes.size());
	uint64_t seq = 0;
	impl_->state_ch.read(st.data(), st.size(), seq);
	NeutralState agg = NeutralState::kReady;
	bool any = false;
	for (std::size_t i = 0; i < impl_->cfg.nodes.size(); ++i)
	{
		const NeutralState s = st[i].state;
		if (!any)
		{
			agg = s;
			any = true;
			continue;
		}
		if (s == NeutralState::kFault || (s == NeutralState::kQuickStop && agg != NeutralState::kFault) || (s == NeutralState::kUnknown && agg != NeutralState::kFault &&
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

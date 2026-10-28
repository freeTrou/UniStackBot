#include "unistackbot_canfd/canfd_master.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <system_error>
#include <cstring>
#include <ctime>
#include <string>
#include <thread>
#include <vector>

#include "rt_tune/rt_tune.hpp"
#include "sp_latest/dynamic_sp_latest.hpp"
#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_protocol/protocol_factory.hpp"
#include "unistackbot_statemachine/node_safety_factory.hpp"

using unistackbot_bus::MasterConfig;
using unistackbot_bus::NodeSnapshot;
using unistackbot_bus::NodeSpec;
using unistackbot_common::DynamicSpLatest;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_protocol::ProtocolCore;
using unistackbot_protocol::createProtocolCore;
using unistackbot_statemachine::NeutralState;
using unistackbot_statemachine::NodeSafety;
using unistackbot_statemachine::createNodeSafety;

namespace
{
constexpr std::int64_t kNsPerSec = 1000000000;
constexpr std::size_t kMaxCanData = 64;   // CAN FD 数据段上限 (frame_len 分型界: ≤8 经典)
/*
 "从未收到过反馈"哨兵 (2026-10-28 台架实测抓的 P0): 默认 fb 的 err=0 是占位不是
 真帧——喂给状态机会把 kUnknown 提前推到 kReady, 随后 idle 以默认零位锚定 MIT
 发给仍处使能态的电机 = 向零位猛拉。未见过真帧前不推进状态机、idle 走
 safety_stop 的 kUnknown 分支 (回退失能帧, DmSafety 设计本意)
*/
constexpr std::uint64_t kNeverSeen = UINT64_MAX;

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

/*
 socket 装配 (设计 §2 五选项; 独立函数 = 无 goto 的显式错误流)。
 返回 fd (≥0) 或 -1 (err 已填)。链路参数 (bitrate/restart-ms/txqueuelen)
 由外部脚本装配 (P7)——本函数只开 socket 不碰链路。
*/
[[nodiscard]] int openCanSocket(const MasterConfig &cfg, std::string &err)
{
	const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (fd < 0)
	{
		err = std::string("socket(PF_CAN): ") + std::strerror(errno);
		return -1;
	}
	int on = 1;
	// ① FD 帧使能 (双模: 经典帧不受影响; canfd_frame 收发需此开关)
	if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0)
	{
		err = std::string("CAN_RAW_FD_FRAMES: ") + std::strerror(errno);
		::close(fd);
		return -1;
	}
	/*
 ② 内核层过滤: 每节点 recv_id 精确匹配 (排除 EFF/RTR 帧, 杂帧不进用户态) +
    错误帧 (CAN_ERR_FLAG 须同时置 can_id 与 can_mask, 设计 §4 订阅注意①)
	*/
	{
		std::vector<can_filter> flt;
		flt.reserve(cfg.nodes.size() + 1);
		for (const NodeSpec &s : cfg.nodes)
		{
			flt.push_back(can_filter{s.can_recv_id, CAN_EFF_FLAG | CAN_RTR_FLAG | 0x7FFu});
		}
		if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, flt.data(), flt.size() * sizeof(can_filter)) < 0)
		{
			err = std::string("CAN_RAW_FILTER: ") + std::strerror(errno);
			::close(fd);
			return -1;
		}
	}
	// ④ 错误帧订阅走 CAN_RAW_ERR_FILTER (2026-10-28 审核修: 原经 CAN_RAW_FILTER
	// 的 CAN_ERR_FLAG 条目不投递——内核只按 err_mask 生成错误帧, BUSOFF/RESTARTED
	// 全链死。设计 §2④ 本来就是这么写的)
	{
		can_err_mask_t em = CAN_ERR_MASK;   // 全九类 (第一版策略: 计数记账)
		if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &em, sizeof(em)) < 0)
		{
			err = std::string("CAN_RAW_ERR_FILTER: ") + std::strerror(errno);
			::close(fd);
			return -1;
		}
	}
	// ③ 收帧内核时间戳 (SO_TIMESTAMPNS → recvmsg cmsg; P5: 必须 recvmsg, read 拿不到)
	if (setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) < 0)
	{
		err = std::string("SO_TIMESTAMPNS: ") + std::strerror(errno);
		::close(fd);
		return -1;
	}
	// ⑤ 非阻塞 (RX 无条件非阻塞——设计 §2: 阻塞等帧 = 从站死你卡死)
	{
		int nb = 1;
		if (ioctl(fd, FIONBIO, &nb) < 0)
		{
			err = std::string("FIONBIO: ") + std::strerror(errno);
			::close(fd);
			return -1;
		}
	}
	// bind (接口不存在 = ifindex 0, fail-fast)
	sockaddr_can addr{};
	addr.can_family = AF_CAN;
	addr.can_ifindex = static_cast<int>(if_nametoindex(cfg.endpoint.c_str()));
	if (addr.can_ifindex == 0)
	{
		err = "接口 '" + cfg.endpoint + "' 不存在 (ip link show; 链路装配见台架脚本)";
		::close(fd);
		return -1;
	}
	if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
	{
		err = std::string("bind(") + cfg.endpoint + "): " + std::strerror(errno);
		::close(fd);
		return -1;
	}
	return fd;
}
}  // namespace

namespace unistackbot_canfd
{

struct CanFdMaster::Impl
{
	MasterConfig cfg;

	// ── socket (单线程独占: 泵线程 read/write 同 fd, 零跨线程交接) ──
	int sock = -1;

	// ── 组合模型 per-node (start 期经工厂自建; 循环期只读, RT 零分配) ──
	std::vector<std::unique_ptr<ProtocolCore>> proto;
	std::vector<std::unique_ptr<NodeSafety>> safety;
	// CAN ID 分发表: recv_id → 节点槽 (装配期校验唯一; 线性扫 n≤~16)
	[[nodiscard]] std::size_t findNodeByRecvId(std::uint32_t can_id) const
	{
		for (std::size_t i = 0; i < cfg.nodes.size(); ++i)
		{
			if (cfg.nodes[i].can_recv_id == can_id)
			{
				return i;
			}
		}
		return cfg.nodes.size();
	}

	// ── 交换通道 (DynamicSpLatest: 装配期定容, 零锁零等待) ──
	DynamicSpLatest<NodeCommand> cmd_ch;
	DynamicSpLatest<NodeSnapshot> state_ch;

	// ── 线程控制 ──
	std::thread cycle_thread;
	std::atomic<bool> stop_flag{false};
	std::atomic<bool> quick_stop_flag{false};
	std::atomic<bool> fault_clear_flag{false};
	std::unique_ptr<std::atomic<std::uint8_t>[]> intent;   // 0=无 1=enable 2=disable
	std::atomic<bool> started{false};
	bool bus_off = false;   // BUSOFF 急停旗/RESTARTED 恢复 (总线线程唯一写者)

	// ── 循环态 (总线线程唯一写者; start() 复位) ──
	std::vector<NodeFeedback> fb;
	std::vector<std::uint64_t> last_seen_cycle;
	std::uint64_t cycle = 0;
	std::uint64_t last_cmd_seq = 0;   // 读门控: seq 变化才全量 read (RT 纪律)

	// ── 工作缓冲 (随 Impl 一次分配; 循环内零构造零分配) ──
	std::vector<NodeCommand> cmd;      // 装配期 resize
	NodeCommand slot_cmd;
	std::uint8_t tx_payload[kMaxCanData];
	NodeFeedback rx_fb;
	std::vector<NodeSnapshot> snap;    // 装配期 resize
	// recvmsg 工作件 (栈复用成员化: 循环内零构造)
	std::uint8_t rx_buf[72];           // canfd_frame 全尺寸 (16B 经典为其前缀, 按返回长度分型)
	char rx_control[64];               // CMSG_SPACE(timespec) 上界
};

CanFdMaster::CanFdMaster() : impl_(new Impl)
{
}

CanFdMaster::~CanFdMaster()
{
	stop();
}

const NodeSafety &CanFdMaster::node_safety(std::size_t index) const
{
	return *impl_->safety[index];
}

// ───────────────────────── start: 装配 + 校验 ─────────────────────────

bool CanFdMaster::start(const MasterConfig &cfg)
{
	if (impl_->started.load())
	{
		return false;
	}
	if (cfg.rate_hz <= 0.0 || cfg.nodes.empty() || cfg.endpoint.empty())
	{
		return false;
	}
	if (cfg.thread_priority < 0 || cfg.thread_priority > 99 || cfg.cpu_affinity < -1 || cfg.stale_cycles == 0)
	{
		return false;
	}
	const std::size_t n = cfg.nodes.size();
	impl_->cfg = cfg;
	impl_->proto.clear();
	impl_->proto.resize(n);
	impl_->safety.clear();
	impl_->safety.resize(n);
	impl_->fb.assign(n, NodeFeedback{});
	impl_->last_seen_cycle.assign(n, kNeverSeen);
	impl_->cmd.assign(n, NodeCommand{});
	impl_->snap.assign(n, NodeSnapshot{});
	impl_->intent.reset(new std::atomic<std::uint8_t>[n]);
	const auto failCleanup = [this, n]() {
		for (std::size_t k = 0; k < n; ++k)
		{
			impl_->proto[k].reset();
			impl_->safety[k].reset();
		}
	};

	/*
 ── CAN ID 路由校验 (P6): 唯一性 + 11 位域。recv_id 是 rx 分发键, send_id 是
 tx 帧头——任一撞车 = 静默互吃帧/命令错送, 装配期拒绝。
	*/
	for (std::size_t i = 0; i < n; ++i)
	{
		const NodeSpec &s = cfg.nodes[i];
		if (s.can_send_id == 0 || s.can_send_id > 0x7FFu || s.can_recv_id == 0 || s.can_recv_id > 0x7FFu)
		{
			std::fprintf(stderr, "CanFdMaster: 节点 #%zu (id 0x%02X) CAN ID 非法 (send=0x%03X recv=0x%03X; 须 1..0x7FF)\n",
				i, s.id, s.can_send_id, s.can_recv_id);
			return false;
		}
		for (std::size_t j = 0; j < i; ++j)
		{
			if (s.can_send_id == cfg.nodes[j].can_send_id || s.can_recv_id == cfg.nodes[j].can_recv_id)
			{
				std::fprintf(stderr, "CanFdMaster: 节点 #%zu 与 #%zu CAN ID 重复 (send %03X/%03X recv %03X/%03X)\n",
					j, i, cfg.nodes[j].can_send_id, s.can_send_id, cfg.nodes[j].can_recv_id, s.can_recv_id);
				return false;
			}
			// 跨方向撞车 (2026-10-28 审核修): A.send == B.recv → A 命令帧与 B 反馈帧
			// 同 ID 仲裁互吃 + 主站分发错投——注释宣称拒绝的"静默互吃帧"正是此形态
			if (s.can_send_id == cfg.nodes[j].can_recv_id || s.can_recv_id == cfg.nodes[j].can_send_id)
			{
				std::fprintf(stderr, "CanFdMaster: 节点 #%zu 与 #%zu CAN ID 跨方向撞车 (send %03X/recv %03X vs send %03X/recv %03X)\n",
					j, i, cfg.nodes[j].can_send_id, cfg.nodes[j].can_recv_id, s.can_send_id, s.can_recv_id);
				return false;
			}
		}
	}
	// 节点 id 重复检查 (协议域寻址, serial 同款)
	for (std::size_t i = 1; i < n; ++i)
	{
		for (std::size_t j = 0; j < i; ++j)
		{
			if (cfg.nodes[i].id == cfg.nodes[j].id)
			{
				std::fprintf(stderr, "CanFdMaster: 节点 #%zu 与 #%zu id 重复 (0x%02X)\n", i, j, cfg.nodes[i].id);
				return false;
			}
		}
	}

	// ── 组合模型装配 (serial 同款): 名字解析 + 五重校验 + 双工厂 per-node ──
	for (std::size_t i = 0; i < n; ++i)
	{
		const NodeSpec &spec = cfg.nodes[i];
		const std::string pname = !spec.protocol.empty() ? spec.protocol : cfg.protocol;
		const std::string sname = !spec.statemachine.empty() ? spec.statemachine : cfg.statemachine;
		if (pname.empty())
		{
			std::fprintf(stderr, "CanFdMaster: 节点 #%zu (id 0x%02X) 缺协议名 (节点级与段级皆空)\n", i, spec.id);
			failCleanup();
			return false;
		}
		impl_->proto[i] = createProtocolCore(pname);
		if (impl_->proto[i] == nullptr)
		{
			std::fprintf(stderr, "CanFdMaster: 节点 #%zu 协议 '%s' 未注册 (可用: 见 protocol 工厂)\n", i, pname.c_str());
			failCleanup();
			return false;
		}
		if (!impl_->proto[i]->ratio_valid(spec.ratio))
		{
			std::fprintf(stderr, "CanFdMaster: 节点 #%zu ratio=%.3f 不符协议 '%s' 语义\n", i, spec.ratio, pname.c_str());
			failCleanup();
			return false;
		}
		if (!impl_->proto[i]->id_valid(spec.id))
		{
			std::fprintf(stderr, "CanFdMaster: 节点 #%zu id=0x%02X 不符协议 '%s' id 域\n", i, spec.id, pname.c_str());
			failCleanup();
			return false;
		}
		const auto kind = impl_->proto[i]->kind();
		if (kind == unistackbot_protocol::ProtocolKind::kActuator)
		{
			if (sname.empty())
			{
				std::fprintf(stderr, "CanFdMaster: 执行器节点 #%zu (协议 '%s') 缺状态机名\n", i, pname.c_str());
				failCleanup();
				return false;
			}
			impl_->safety[i] = createNodeSafety(sname);
			if (impl_->safety[i] == nullptr)
			{
				std::fprintf(stderr, "CanFdMaster: 节点 #%zu 状态机 '%s' 未注册\n", i, sname.c_str());
				failCleanup();
				return false;
			}
		}
		else
		{
			if (!sname.empty())
			{
				std::fprintf(stderr, "CanFdMaster: 传感器节点 #%zu 不允许状态机 ('%s')\n", i, sname.c_str());
				failCleanup();
				return false;
			}
			// 泵侧传感器调度未实现 (2026-10-28 审核修: safety 留空但 txOne/意图/快照
			// 无条件解引用 → 首拍 SIGSEGV; serial 同款诚实 fail-fast 优于带雷运行)
			std::fprintf(stderr, "CanFdMaster: 传感器节点 #%zu (协议 '%s') —— CAN 主站泵侧传感器调度未实现, 拒绝\n", i, pname.c_str());
			failCleanup();
			return false;
		}
		// 帧长域校验: CAN 数据段 ≤64B (协议 frame_len 超 = 协议非 CAN 形态)
		if (impl_->proto[i]->framespec().frame_len == 0 || impl_->proto[i]->framespec().frame_len > kMaxCanData)
		{
			std::fprintf(stderr, "CanFdMaster: 节点 #%zu 协议 '%s' frame_len=%zu 非 CAN 形态 (1..64)\n",
				i, pname.c_str(), impl_->proto[i]->framespec().frame_len);
			failCleanup();
			return false;
		}
	}
	// 交换通道装配期定容 (R-a)
	impl_->cmd_ch.init(n, NodeCommand{});
	impl_->state_ch.init(n, NodeSnapshot{});

	// ── socket 装配 (设计 §2 五选项; 校验先于此 = 配置错误不碰硬件) ──
	{
		std::string sock_err;
		impl_->sock = openCanSocket(cfg, sock_err);
		if (impl_->sock < 0)
		{
			std::fprintf(stderr, "CanFdMaster: %s\n", sock_err.c_str());
			failCleanup();
			return false;
		}
	}

	impl_->stop_flag.store(false);
	impl_->quick_stop_flag.store(false);
	impl_->fault_clear_flag.store(false);
	impl_->bus_off = false;
	impl_->cycle = 0;
	impl_->last_cmd_seq = 0;
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
		std::fprintf(stderr, "CanFdMaster: 总线线程创建失败: %s\n", e.what());
		::close(impl_->sock);
		impl_->sock = -1;
		failCleanup();
		return false;
	}
	impl_->started.store(true);
	running_.store(true);
	return true;
}

// ───────────────────────── 泵循环 ─────────────────────────

void CanFdMaster::busLoopMain()
{
	unistackbot_common::rt_tune::apply(impl_->cfg.cpu_affinity, impl_->cfg.thread_priority, 0, "canfd_bus");

	const std::size_t n = impl_->cfg.nodes.size();
	const std::int64_t period_ns = static_cast<std::int64_t>(kNsPerSec / impl_->cfg.rate_hz);
	/*
 排空帧数硬上限 (P4): 2×节点数 + 余量。期望每拍 n 帧回显; 上限防坏节点
 刷总线把整拍预算吃光 (超限帧留在内核队列由下拍继续, 计 rx_capped)
	*/
	const std::size_t drain_cap = 2 * n + 8;
	std::int64_t next = clockNowNs() + period_ns;

	while (!impl_->stop_flag.load())
	{
		// ── 清故障闩锁 (serial 同款: FAULT 节点置请求; 无人 FAULT 自熄) ──
		if (impl_->fault_clear_flag.load())
		{
			bool any_fault = false;
			for (std::size_t k = 0; k < n; ++k)
			{
				if (impl_->safety[k]->state() == NeutralState::kFault)
				{
					impl_->safety[k]->request_fault_clear();
					any_fault = true;
				}
			}
			if (!any_fault)
			{
				impl_->fault_clear_flag.store(false);
			}
		}

		/*
 ── 意图请求转发 (serial 同款: FAULT 入口失效 enable / 守卫拒绝保持 /
    急停压制) ──
		*/
		if (!impl_->quick_stop_flag.load())
		{
			for (std::size_t k = 0; k < n; ++k)
			{
				std::uint8_t want = impl_->intent[k].load();
				if (want == 0)
				{
					continue;
				}
				const NeutralState st = impl_->safety[k]->state();
				if (want == 1 && st == NeutralState::kFault)
				{
					impl_->intent[k].store(0);
					continue;
				}
				want = impl_->intent[k].load();
				if (want == 0)
				{
					continue;
				}
				if (want == 1)
				{
					if (st == NeutralState::kReady)
					{
						impl_->safety[k]->request_enable();
						impl_->intent[k].store(0);
					}
				}
				else if (st != NeutralState::kUnknown)
				{
					impl_->safety[k]->request_disable();
					impl_->intent[k].store(0);
				}
			}
		}

		// ── rxDrain: 非阻塞排空 (上一拍命令的回显 + 错误帧) ──
		rxDrain(drain_cap);

		/*
 ── 状态机推进: 反馈喂 NodeSafety + 清故障握手确认 (同线程零交接)。
 未见过真帧的节点不推进 (kNeverSeen 门)——默认 fb 的 err=0 是占位, 见常量注释
		*/
		for (std::size_t k = 0; k < n; ++k)
		{
			// 陈旧节点同跳 (2026-10-28 审核修: 原来只挡 never-seen——见过反馈后静默的
			// 节点仍被旧帧喂状态机/意图锚定秒级旧 setpoint; 陈旧 = 信念撤销语义一致)
			if (impl_->last_seen_cycle[k] == kNeverSeen ||
				impl_->cycle - impl_->last_seen_cycle[k] > impl_->cfg.stale_cycles)
			{
				continue;
			}
			NodeSafety &ns = *impl_->safety[k];
			ns.update(impl_->fb[k]);
			if (ns.fault_clear_requested() && ns.last_feedback().error_raw == 0u)
			{
				ns.confirm_fault_cleared();
			}
		}

		/*
 ── txCommands: 逐节点意图阶梯 → encode → write ──
 BUSOFF 挂起期停 TX (设计 §4: BUSOFF → 本地急停旗 + 停 TX; RESTARTED 恢复)
		*/
		if (!impl_->bus_off)
		{
			for (std::size_t k = 0; k < n; ++k)
			{
				txOne(k);
			}
		}

		publishSnapshot();
		++impl_->cycle;

		// ── 拍级追帧守卫 (P3): 逾期超一拍重锚绝不补发 (serial 为时隙级, CAN 无时隙) ──
		next += period_ns;
		if (clockNowNs() - next > period_ns)
		{
			next = clockNowNs();
			++telemetry_.skipped_cycles;
		}
		sleepUntilNs(next);
	}
}

void CanFdMaster::rxDrain(std::size_t cap)
{
	const std::size_t n = impl_->cfg.nodes.size();
	bool capped = false;
	for (std::size_t got = 0; got < cap; ++got)
	{
		iovec iov{};
		iov.iov_base = impl_->rx_buf;
		iov.iov_len = sizeof(impl_->rx_buf);
		msghdr msg{};
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;
		msg.msg_control = impl_->rx_control;
		msg.msg_controllen = sizeof(impl_->rx_control);
		const ssize_t r = recvmsg(impl_->sock, &msg, MSG_DONTWAIT);
		if (r < 0)
		{
			break;   // EAGAIN (排空) 或瞬时错——下拍再试, 无帧不报警 (心跳看门狗负责发现)
		}
		if (got + 1 == cap)
		{
			capped = true;   // 触顶: 队列仍有帧 (P4 防御可见化)
		}
		// 内核收帧时间戳 (SO_TIMESTAMPNS → SCM_TIMESTAMPNS)
		for (cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg))
		{
			if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS)
			{
				const timespec * ts = reinterpret_cast<const timespec *>(CMSG_DATA(cmsg));
				telemetry_.last_rx_ts_ns =
					static_cast<std::uint64_t>(ts->tv_sec) * 1000000000u + static_cast<std::uint64_t>(ts->tv_nsec);
			}
		}
		// 帧分型: 16B=can_frame (经典+错误帧) / 72B=canfd_frame (错误帧恒 16B, 设计 §4 细节②)
		if (r != 16 && r != 72)
		{
			continue;   // 垃圾长度 (驱动异常), 丢弃计数走 err 路径之下不掩盖
		}
		const can_frame * cf = reinterpret_cast<const can_frame *>(impl_->rx_buf);
		const std::uint32_t can_id = (r == 16) ? cf->can_id :
			reinterpret_cast<const canfd_frame *>(impl_->rx_buf)->can_id;
		// 错误帧分流 (CAN_ERR_FLAG 位): BUSOFF/RESTARTED 两根线 + 计数记账
		if ((can_id & CAN_ERR_FLAG) != 0u)
		{
			++telemetry_.err_frames;
			if ((can_id & CAN_ERR_BUSOFF) != 0u)
			{
				++telemetry_.busoff_events;
				impl_->bus_off = true;
			}
			if ((can_id & CAN_ERR_RESTARTED) != 0u)
			{
				impl_->bus_off = false;
			}
			continue;
		}
		// 数据帧: CAN ID 分发 (recv_id → 节点槽) + 槽位协议解码
		const std::size_t i = impl_->findNodeByRecvId(can_id & 0x7FFu);
		if (i == n)
		{
			++telemetry_.rx_nofit;
			continue;
		}
		const std::uint8_t * data = (r == 16) ? cf->data :
			reinterpret_cast<const canfd_frame *>(impl_->rx_buf)->data;
		const std::size_t dlen = (r == 16) ? cf->can_dlc :
			reinterpret_cast<const canfd_frame *>(impl_->rx_buf)->len;
		if (!impl_->proto[i]->decode(data, dlen, impl_->rx_fb, impl_->cfg.nodes[i].ratio))
		{
			++telemetry_.rx_rejected;
			continue;
		}
		impl_->fb[i] = impl_->rx_fb;
		impl_->last_seen_cycle[i] = impl_->cycle;
		++telemetry_.rx_frames;
	}
	if (capped)
	{
		++telemetry_.rx_capped;   // 排空触顶 (P4: 刷帧总线的诊断线索)
	}
}

void CanFdMaster::txOne(std::size_t k)
{
	/*
 意图阶梯 (serial resolveSlotCmd 同款; P1 红线: 每节点每拍必产出一帧——
 命令-回显协议下"不发 = 看门狗饿死", 阶梯六形态必落其一)
	*/
	NodeSafety &ns = *impl_->safety[k];
	if (impl_->quick_stop_flag.load())
	{
		impl_->slot_cmd = ns.safety_stop_command();
	}
	else if (ns.fault_clear_requested())
	{
		impl_->slot_cmd = NodeCommand{};
		impl_->slot_cmd.mode = NodeMode::kClear;
		impl_->slot_cmd.watchdog_enable = true;
	}
	else if (ns.enable_requested())
	{
		impl_->slot_cmd = ns.enable_command();
	}
	else if (ns.disable_requested())
	{
		impl_->slot_cmd = ns.disable_command();
	}
	else
	{
		if (impl_->cmd_ch.seq() != impl_->last_cmd_seq)
		{
			uint64_t rseq = 0;
			impl_->cmd_ch.read(impl_->cmd.data(), impl_->cmd.size(), rseq);
			impl_->last_cmd_seq = rseq;
		}
		if (impl_->cmd_ch.frameKind(impl_->last_cmd_seq) != DynamicSpLatest<NodeCommand>::FrameKind::kLive)
		{
			impl_->slot_cmd = ns.idle_command();   // 安全怠速 (达妙=锚定阻尼 MIT)
		}
		else
		{
			impl_->slot_cmd = impl_->cmd[k];
		}
	}

	const NodeSpec &spec = impl_->cfg.nodes[k];
	std::size_t fn = impl_->proto[k]->encode(impl_->tx_payload, sizeof(impl_->tx_payload),
		impl_->slot_cmd, spec.id, spec.ratio);
	if (fn == 0)
	{
		/*
 encode 拒收 (非有限/kd=0 等): 计 tx_failed 不 return——P1 红线下仍须发帧,
 回退安全怠速 (该帧协议必收: idle 形态由状态机产出, 无非法值)
		*/
		++telemetry_.tx_failed;
		impl_->slot_cmd = ns.idle_command();
		// fn 覆写为回退帧长度 (2026-10-28 审核修: 原 fn=0 带进帧组装 →
		// can_dlc=0 空帧替代 idle, P1 红线静默失效)
		fn = impl_->proto[k]->encode(impl_->tx_payload, sizeof(impl_->tx_payload),
			impl_->slot_cmd, spec.id, spec.ratio);
		if (fn == 0)
		{
			return;   // idle 也拒 (协议彻底不符)——放弃本节点本拍, 计数已立
		}
	}

	/*
 帧组装: frame_len ≤8 → can_frame (经典); >8 → canfd_frame (双模裁决,
 v1 消费者达妙恒经典)。write 返回 ≠ 帧上总线 = 已交内核 qdisc (仲裁硬件做)。
	*/
	if (fn <= 8)
	{
		can_frame cf{};
		cf.can_id = spec.can_send_id;
		cf.can_dlc = static_cast<std::uint8_t>(fn);
		std::memcpy(cf.data, impl_->tx_payload, fn);
		if (::write(impl_->sock, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf)))
		{
			++telemetry_.tx_frames;
		}
		else
		{
			++telemetry_.tx_failed;   // ENOBUFS = qdisc 溢出 = 循环落后于总线, 与 overrun 同级
		}
	}
	else
	{
		canfd_frame fdf{};
		fdf.can_id = spec.can_send_id;
		fdf.len = static_cast<std::uint8_t>(fn);
		fdf.flags = CANFD_BRS;   // FD 帧数据段速率切换 (BRS 不置 = 全程仲裁速率, 慢 4~10 倍)
		std::memcpy(fdf.data, impl_->tx_payload, fn);
		if (::write(impl_->sock, &fdf, sizeof(fdf)) == static_cast<ssize_t>(sizeof(fdf)))
		{
			++telemetry_.tx_frames;
		}
		else
		{
			++telemetry_.tx_failed;
		}
	}
}

void CanFdMaster::publishSnapshot()
{
	const std::size_t n = impl_->cfg.nodes.size();
	for (std::size_t i = 0; i < n; ++i)
	{
		impl_->snap[i].fb = impl_->fb[i];
		impl_->snap[i].state =
			(impl_->last_seen_cycle[i] == kNeverSeen ||
				impl_->cycle - impl_->last_seen_cycle[i] > impl_->cfg.stale_cycles)
			? NeutralState::kUnknown   // 信念撤销 (陈旧/未见过覆盖): 不改状态机内部
			: impl_->safety[i]->state();
	}
	impl_->state_ch.publish(impl_->snap);
}

// ───────────────────────── 生命周期 / 交换 / 意图 ─────────────────────────

void CanFdMaster::stop()
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
	if (impl_->sock >= 0)
	{
		::close(impl_->sock);
		impl_->sock = -1;
	}
}

void CanFdMaster::publish_cmd(const NodeCommand *cmds, std::size_t n)
{
	impl_->cmd_ch.publish(cmds, n);
}

bool CanFdMaster::take_state(std::vector<NodeSnapshot> &out)
{
	uint64_t seq = 0;
	return impl_->state_ch.readLastFrame(out.data(), out.size(), seq) ==
	       DynamicSpLatest<NodeSnapshot>::FrameKind::kLive;
}

void CanFdMaster::quick_stop()
{
	impl_->quick_stop_flag.store(true);
	for (std::size_t k = 0; k < impl_->cfg.nodes.size(); ++k)
	{
		impl_->intent[k].store(0);   // 意图旗失效 (急停不被挂起 enable 静默撤销)
	}
}

void CanFdMaster::enable(std::uint8_t node_id)
{
	for (std::size_t k = 0; k < impl_->cfg.nodes.size(); ++k)
	{
		if (impl_->cfg.nodes[k].id == node_id)
		{
			impl_->intent[k].store(1);
			return;
		}
	}
}

void CanFdMaster::disable(std::uint8_t node_id)
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

void CanFdMaster::fault_clear()
{
	impl_->fault_clear_flag.store(true);
}

NeutralState CanFdMaster::state() const
{
	std::vector<NodeSnapshot> st(impl_->cfg.nodes.size());
	uint64_t seq = 0;
	impl_->state_ch.read(st.data(), st.size(), seq);
	NeutralState agg = NeutralState::kReady;
	bool any = false;
	for (std::size_t i = 0; i < st.size(); ++i)
	{
		const NeutralState s = st[i].state;
		if (!any)
		{
			agg = s;
			any = true;
			continue;
		}
		if (s == NeutralState::kFault || (s == NeutralState::kQuickStop && agg != NeutralState::kFault) ||
			(s == NeutralState::kUnknown && agg != NeutralState::kFault && agg != NeutralState::kQuickStop))
		{
			agg = s;
		}
	}
	return any ? agg : NeutralState::kUnknown;
}

bool registerToMasterFactory()
{
	return unistackbot_bus::registerMaster("canfd", []() ->
		std::unique_ptr<unistackbot_bus::MasterBase> {
		return std::make_unique<CanFdMaster>();
	});
}

}  // namespace unistackbot_canfd

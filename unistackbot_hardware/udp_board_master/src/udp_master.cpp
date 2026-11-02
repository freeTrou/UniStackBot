#include "unistackbot_udp_board/udp_master.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>

#include "rt_tune/rt_tune.hpp"
#include "unistackbot_bus/master_factory.hpp"
#include "unistackbot_protocol/board_protocol.hpp"

namespace unistackbot_udp_board
{
using unistackbot_bus::MasterConfig;
using unistackbot_bus::NodeSnapshot;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_statemachine::NeutralState;

namespace
{
inline std::uint64_t nowNs()
{
	return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
}
}   // namespace

UdpMaster::~UdpMaster()
{
	stop();
}

bool UdpMaster::openSocket(const std::string &endpoint)
{
	// endpoint = "dest_ip:dest_port[:bind_port]"
	const auto c1 = endpoint.find(':');
	if (c1 == std::string::npos || c1 == 0 || c1 + 1 >= endpoint.size())
	{
		return false;
	}
	const std::string ip = endpoint.substr(0, c1);
	std::string rest = endpoint.substr(c1 + 1);
	const auto c2 = rest.find(':');
	const int dest_port = std::atoi(rest.substr(0, c2 == std::string::npos ? rest.size() : c2).c_str());
	bind_port_ = static_cast<std::uint16_t>(c2 == std::string::npos ? dest_port
		: std::atoi(rest.substr(c2 + 1).c_str()));
	if (dest_port <= 0 || bind_port_ == 0)
	{
		return false;
	}
	sock_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock_ < 0)
	{
		return false;
	}
	sockaddr_in bind_addr{};
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_addr.s_addr = INADDR_ANY;
	bind_addr.sin_port = htons(bind_port_);
	if (bind(sock_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) < 0)
	{
		close(sock_);
		sock_ = -1;
		return false;
	}
	const int flags = fcntl(sock_, F_GETFL, 0);
	fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
	std::memset(&board_addr_, 0, sizeof(board_addr_));
	board_addr_.sin_family = AF_INET;
	board_addr_.sin_addr.s_addr = inet_addr(ip.c_str());
	board_addr_.sin_port = htons(static_cast<std::uint16_t>(dest_port));
	return true;
}

bool UdpMaster::start(const MasterConfig &cfg)
{
	if (running_.load())
	{
		return false;
	}
	if (!(cfg.rate_hz > 0.0) || cfg.endpoint.empty() || cfg.nodes.empty())
	{
		return false;
	}
	// 槽上限 = 协议帧容量 (单一事实源住 board_protocol.hpp, 不在此复写数字)
	constexpr std::size_t kMaxSlots =
		unistackbot_protocol::kArmBusCount * unistackbot_protocol::kArmMotorsPerBus +
		unistackbot_protocol::kChassisMotorCount;
	if (cfg.nodes.size() > kMaxSlots)
	{
		return false;
	}
	// 名字解析 (MasterConfig 四规则的本实现版): 协议名统一 "udp_board" (聚合帧族
	// 不入 per-node 协议工厂——节点表 protocol 字段在本段只作一致性校验)
	for (std::size_t i = 0; i < cfg.nodes.size(); ++i)
	{
		const std::string &proto = cfg.nodes[i].protocol.empty() ? cfg.protocol : cfg.nodes[i].protocol;
		if (proto != "udp_board")
		{
			return false;   // 诊断: 节点 [i] 协议名非 udp_board
		}
		const std::string &sm = cfg.nodes[i].statemachine.empty() ? cfg.statemachine : cfg.nodes[i].statemachine;
		if (sm.empty())
		{
			return false;
		}
	}
	if (!openSocket(cfg.endpoint))
	{
		return false;
	}
	n_ = cfg.nodes.size();
	period_ = std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / cfg.rate_hz));
	pump_prio_ = cfg.thread_priority;
	pump_cpu_ = cfg.cpu_affinity;
	safety_.clear();
	for (std::size_t i = 0; i < n_; ++i)
	{
		const std::string &sm = cfg.nodes[i].statemachine.empty() ? cfg.statemachine : cfg.nodes[i].statemachine;
		safety_.push_back(unistackbot_statemachine::createNodeSafety(sm));
		if (!safety_.back())
		{
			safety_.clear();
			close(sock_);
			sock_ = -1;
			return false;
		}
	}
	cmd_ch_.init(n_, NodeCommand{});
	state_ch_.init(n_, unistackbot_bus::NodeSnapshot{});
	cmd_rx_.assign(n_, NodeCommand{});
	fb_buf_.assign(n_, NodeFeedback{});
	snap_.assign(n_, unistackbot_bus::NodeSnapshot{});
	idle_.assign(n_, NodeCommand{});
	for (std::size_t i = 0; i < n_; ++i)
	{
		idle_[i] = safety_[i]->idle_command();   // 安全怠速 (start 后未发命令的驻留帧)
	}
	rx_count_.store(0, std::memory_order_relaxed);
	quick_stop_.store(false, std::memory_order_relaxed);
	reset_beats_.store(0, std::memory_order_relaxed);
	running_.store(true, std::memory_order_release);
	pump_ = std::thread([this]() { pumpMain(); });
	return true;
}

void UdpMaster::stop()
{
	if (!running_.exchange(false, std::memory_order_acq_rel))
	{
		return;
	}
	if (pump_.joinable())
	{
		pump_.join();
	}
	if (sock_ >= 0)
	{
		close(sock_);
		sock_ = -1;
	}
}

void UdpMaster::pumpMain()
{
	unistackbot_common::rt_tune::apply(pump_cpu_, pump_prio_, 0, "udp_bus");
	auto next = std::chrono::steady_clock::now();
	std::uint64_t seq = 0;
	while (running_.load(std::memory_order_acquire))
	{
		next += period_;
		std::this_thread::sleep_until(next);

		// ── 命令选源 (优先级: 清故障拍 > quick_stop > 命令流 > 安全怠速) ──
		const bool stopping = quick_stop_.load(std::memory_order_acquire);
		const int resets = reset_beats_.load(std::memory_order_acquire);
		const NodeCommand *src = idle_.data();
		if (resets > 0)
		{
			reset_beats_.fetch_sub(1);
			// 复位拍载荷 = 全零化 (golden: 零化帧语义)
		}
		else if (stopping)
		{
			for (std::size_t i = 0; i < n_; ++i)
			{
				idle_[i] = safety_[i]->safety_stop_command();   // 全节点 kStop, enable 全 0
			}
		}
		else if (cmd_ch_.read(cmd_rx_.data(), cmd_rx_.size(), seq))
		{
			src = cmd_rx_.data();
		}
		const std::uint8_t ctrl_word = resets > 0
			? unistackbot_protocol::kCtrlWordReset
			: unistackbot_protocol::kCtrlWordPdMit;
		if (unistackbot_protocol::encodeBoardControl(src, n_, tx_, sizeof(tx_), ctrl_word))
		{
			tx_[0] = static_cast<std::uint8_t>(nowNs() & 0xFF);   // ts 滚动章
			sendto(sock_, tx_, sizeof(tx_), 0,
				reinterpret_cast<const sockaddr *>(&board_addr_), sizeof(board_addr_));
		}

		// ── RX: 非阻塞收, 取最新一帧 (旧帧丢弃; ≤8 拍积压上限) ──
		sockaddr_in from{};
		socklen_t flen = sizeof(from);
		bool fresh = false;
		for (int drain = 0; drain < 8; ++drain)
		{
			const ssize_t r = recvfrom(sock_, rx_, sizeof(rx_), MSG_DONTWAIT,
				reinterpret_cast<sockaddr *>(&from), &flen);
			if (r != static_cast<ssize_t>(sizeof(rx_)))
			{
				break;
			}
			fresh = true;
		}
		if (fresh && unistackbot_protocol::decodeBoardResponse(rx_, sizeof(rx_), fb_buf_.data(), n_))
		{
			rx_count_.fetch_add(1, std::memory_order_relaxed);
			for (std::size_t i = 0; i < n_; ++i)
			{
				// kNeverSeen 由交换层初值承载: 首帧前状态机吃不到数据 (无 update 调用)
				safety_[i]->update(fb_buf_[i]);
				if (safety_[i]->fault_clear_requested() && fb_buf_[i].error_raw == 0u)
				{
					safety_[i]->acknowledge_fault_clear();
					safety_[i]->confirm_fault_cleared();   // error 持续 0 → FAULT 解除
				}
				snap_[i].fb = fb_buf_[i];
				snap_[i].state = safety_[i]->state();
			}
			state_ch_.publish(snap_);
		}
	}
}

void UdpMaster::publish_cmd(const NodeCommand *cmds, std::size_t n)
{
	if (cmds == nullptr || n != n_)
	{
		return;   // 交换层尺寸契约 (调用方 bug 静默忽略)
	}
	quick_stop_.store(false, std::memory_order_release);   // 命令流恢复 = 急停旗解除
	cmd_ch_.publish(cmds, n);
}

bool UdpMaster::take_state(std::vector<NodeSnapshot> &out)
{
	if (out.size() < n_)
	{
		out.resize(n_);
	}
	std::uint64_t seq = 0;
	return state_ch_.read(out.data(), out.size(), seq);
}

void UdpMaster::enable(std::uint8_t node_id)
{
	if (node_id < n_)
	{
		safety_[node_id]->request_enable();
	}
}

void UdpMaster::disable(std::uint8_t node_id)
{
	if (node_id < n_)
	{
		safety_[node_id]->request_disable();
	}
}

void UdpMaster::fault_clear()
{
	for (auto & s : safety_)
	{
		s->request_fault_clear();
	}
	reset_beats_.store(2, std::memory_order_release);   // 复位拍 ×2 (单拍可能被板内时序吞)
}

NeutralState UdpMaster::state() const
{
	NeutralState worst = NeutralState::kUnknown;
	bool any = false;
	for (const auto & s : safety_)
	{
		const NeutralState st = s->state();
		if (!any || st == NeutralState::kFault)
		{
			worst = st;
			any = true;
		}
		else if (st == NeutralState::kQuickStop && worst != NeutralState::kFault)
		{
			worst = st;
		}
	}
	return any ? worst : NeutralState::kUnknown;
}

bool UdpMaster::registerToMasterFactory()
{
	return unistackbot_bus::registerMaster("udp",
		[]() -> std::unique_ptr<MasterBase> { return std::make_unique<UdpMaster>(); });
}

}  // namespace unistackbot_udp_board

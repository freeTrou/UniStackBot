#ifndef UNISTACKBOT_UDP_BOARD__UDP_MASTER_HPP_
#define UNISTACKBOT_UDP_BOARD__UDP_MASTER_HPP_
/*
 * UdpMaster —— MasterBase 第三实现 (2026-11-02 总线/协议拆分; serial/canfd 同款组织):
 * 总线骨架只留 IO 工程 (socket/总线循环/交换), 协议内芯住 unistackbot_protocol
 * (board_protocol 聚合帧族), 状态机住 unistackbot_statemachine (BoardSafety)。
 *
 * 总线循环 (UDP 请求-应答, 与 serial 时隙/canfd 排空-发送都不同):
 *   每拍 (rate_hz): 取最新命令快照 → 聚合编码 278B Control → sendto
 *   → 非阻塞收 Response (取最新一帧, 旧帧丢弃) → 聚合解码 → 喂状态机 → 发快照。
 *   主站节拍可高于板固件节拍 (板按收到即回语义工作)。
 *
 * endpoint 载荷格式 (MasterConfig::endpoint 中立载体, UDP 语义):
 *   "dest_ip:dest_port[:bind_port]"   (bind 缺省 = dest_port; P20 部署 192.168.x.x:10086:9090)
 *
 * 安全语义:
 *   quick_stop  = 原子旗 → 下一拍全节点 kStop 帧 (enable 全 0, 板侧保护兜底)
 *   安全怠速    = start 后未发命令 → 全节点 idle_command (kStop)
 *   kNeverSeen  = 未见过真 Response 帧不喂状态机 (canfd P0 同款——默认零值反馈
 *                 锚定会令使能命令向零位猛拉)
 *   清故障      = ctrl_word=Reset 特殊拍 ×2 → error 持续 0 → confirm 握手 (M4 验证)
 */
#include <netinet/in.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "unistackbot_bus/master_base.hpp"
#include "unistackbot_statemachine/board_safety.hpp"
#include "unistackbot_statemachine/node_safety_factory.hpp"
#include "sp_latest/dynamic_sp_latest.hpp"

namespace unistackbot_udp_board
{

class UdpMaster final : public unistackbot_bus::MasterBase
{
public:
	UdpMaster() = default;
	~UdpMaster() override;

	[[nodiscard]] bool start(const unistackbot_bus::MasterConfig &cfg) override;
	void stop() override;

	void publish_cmd(const unistackbot_protocol::NodeCommand *cmds, std::size_t n) override;
	[[nodiscard]] bool take_state(std::vector<unistackbot_bus::NodeSnapshot> &out) override;

	void quick_stop() override { quick_stop_.store(true, std::memory_order_release); }
	void enable(std::uint8_t node_id) override;
	void disable(std::uint8_t node_id) override;
	void fault_clear() override;
	[[nodiscard]] unistackbot_statemachine::NeutralState state() const override;

	[[nodiscard]] unistackbot_bus::ParamStatus read_param(std::uint8_t, std::uint32_t,
		std::uint64_t &, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;   // 板协议无慢通道
	}
	[[nodiscard]] unistackbot_bus::ParamStatus write_param(std::uint8_t, std::uint32_t,
		std::uint64_t, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}

	// 组合根注册 ("udp" 进 createMaster 工厂; serial 同款显式调用)
	[[nodiscard]] static bool registerToMasterFactory();

private:
	void pumpMain();
	[[nodiscard]] bool openSocket(const std::string &endpoint);

	// 装配态
	std::size_t n_{0};
	std::vector<std::unique_ptr<unistackbot_statemachine::NodeSafety>> safety_;
	std::chrono::nanoseconds period_{0};
	int pump_prio_{85};
	int pump_cpu_{2};

	// IO
	int sock_{-1};
	struct sockaddr_in board_addr_
	{
	};
	std::uint16_t bind_port_{0};

	// 总线线程
	std::thread pump_;
	std::atomic<bool> running_{false};
	std::atomic<bool> quick_stop_{false};
	std::atomic<int> reset_beats_{0};        // 清故障特殊拍剩余数
	std::atomic<std::uint64_t> rx_count_{0}; // kNeverSeen 判据 (真帧计数)

	// 交换 (装配期定容, 元素=单节点, 容量=节点数; canfd 同款)
	unistackbot_common::DynamicSpLatest<unistackbot_protocol::NodeCommand> cmd_ch_;
	unistackbot_common::DynamicSpLatest<unistackbot_bus::NodeSnapshot> state_ch_;

	// 总线循环暂存
	std::vector<unistackbot_protocol::NodeCommand> cmd_rx_;
	std::vector<unistackbot_protocol::NodeFeedback> fb_buf_;   // start 期定容 (R-a: 容量常量退役, 与节点表同源)
	std::vector<unistackbot_bus::NodeSnapshot> snap_;
	std::vector<unistackbot_protocol::NodeCommand> idle_;
	std::uint8_t tx_[278];
	std::uint8_t rx_[474];
};

}  // namespace unistackbot_udp_board
#endif  // UNISTACKBOT_UDP_BOARD__UDP_MASTER_HPP_

#ifndef UNISTACKBOT_SERIAL__SERIAL_MASTER_HPP_
#define UNISTACKBOT_SERIAL__SERIAL_MASTER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "unistackbot_bus/master_base.hpp"
#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/node_safety.hpp"
#include "unistackbot_serial/framer.hpp"
#include "unistackbot_serial/io_transport.hpp"

// SerialMaster —— 串口总线骨架的 MasterBase 实现 (2026-09-24 落地; 2026-09-28 更名+NodeSafety 集成)。
//
// 总线线程模型 (设计 rs485_master_design.md §3): FIFO 85/核2 (rt_tune, 失败优雅降级),
// 周期 = cfg.rate_hz, 结构分层:
//   busLoopMain     节拍骨架: 时隙推进 + 追帧跳过纪律
//   runSlot(k)      单时隙: 安全决策 → encode → 单帧单 write → 排干解码 → 状态更新
//   publishSnapshot 落拍: 陈旧判定 + 状态翻译 + SpLatest 发布
//
// 单线程合并架构 (2026-09-28 用户全局决策): 总线 IO + 状态机 (NodeSafety) + 安全守卫
// 全部在总线线程内——反馈帧一到手就更新 NodeSafety, 零跨线程交接, 零锁。
//
// **落后追帧纪律**: 逾期超一个时隙即跳到未来期限, 绝不补发 (FIFO 同优先级不轮转教训)。
// **命令年龄 1-2 拍**: 不等本拍响应 (盲发), SpLatest 取最新。
// 安全默认: start 后未 publish 过命令 = 持续发停机帧+看门狗位 (安全怠速)。
// 生命周期: stop() 释放 transport、start() 复位循环态与 quick_stop 闩锁 —— 可重启。
//
// 交换: NodeCommand/NodeSnapshot 载荷经 DynamicSpLatest (装配期定容动态双缓冲, 零锁零等待零分配)。
// 慢通道: read_param/write_param 返回 kUnsupported (unitree 纯周期帧协议无此能力)。

namespace unistackbot_serial
{

class SerialMaster final : public unistackbot_bus::MasterBase
{
public:
	// 遥测 (测试/诊断; copy 读, 总线线程侧为唯一写者时近似一致)
	struct Telemetry
	{
		std::uint64_t tx_frames = 0;      // 发出命令帧数
		std::uint64_t tx_failed = 0;      // 写失败/部分写 (下一时隙发最新命令 = 盲发重试)
		std::uint64_t rx_frames = 0;      // decode 成功的反馈帧数
		std::uint64_t rx_rejected = 0;    // decode 拒收 (坏 CRC/坏头)
		std::uint64_t rx_nofit = 0;       // 帧合法但 node_id 不在节点表
		std::uint64_t skipped_slots = 0;  // 追帧跳过的时隙数 (健康签名=0)
	};

	SerialMaster();
	~SerialMaster() override;

	// 装配期注入 (组合根在 start() 之前调用)。安全状态机不经注入——总线内部组件,
	// start 期按 cfg.statemachine 名经工厂自建 per-node 实例。
	void attach_transport(std::unique_ptr<IoTransport> io);

	[[nodiscard]] bool start(const unistackbot_bus::MasterConfig &cfg) override;
	void stop() override;
	void publish_cmd(const unistackbot_protocol::NodeCommand *cmds, std::size_t n) override;
	using MasterBase::publish_cmd;   // vector 便利重载不因 override 遮蔽
	[[nodiscard]] bool take_state(std::vector<unistackbot_bus::NodeSnapshot> &out) override;
	void quick_stop() override;
	void enable(std::uint8_t node_id) override;
	void disable(std::uint8_t node_id) override;
	void fault_clear() override;   // 清故障闩锁 (全节点; 全清后自熄, 见 cpp)
	[[nodiscard]] unistackbot_statemachine::NeutralState state() const override;
	[[nodiscard]] unistackbot_bus::ParamStatus read_param(std::uint8_t, std::uint32_t, std::uint64_t &, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}
	[[nodiscard]] unistackbot_bus::ParamStatus write_param(std::uint8_t, std::uint32_t, std::uint64_t, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}

	// 单节点安全状态机 (外部只读访问; 使能/断使能/清故障经 request_* 方法触发,
	// 总线线程在下一拍消费请求并生成安全命令)
	[[nodiscard]] const unistackbot_statemachine::NodeSafety &node_safety(std::size_t index) const;

	[[nodiscard]] Telemetry telemetry() const { return telemetry_; }
	[[nodiscard]] bool running() const { return running_.load(); }

private:
	// ── 总线循环结构 (全部总线线程侧调用; 详见文件头注释) ──
	void busLoopMain();
	void runSlot(std::size_t k);
	void resolveSlotCmd(std::size_t k);
	void drainAndDecode();
	void classifyFrame(const ExtractedFrame &frame);
	[[nodiscard]] std::size_t findNode(std::uint8_t node_id) const;
	void publishSnapshot();
	void resetCycleState();

	struct Impl;
	std::unique_ptr<Impl> impl_;
	std::atomic<bool> running_{false};
	mutable Telemetry telemetry_;
};

// 组合根调用: 向 bus 工厂注册 "serial" (幂等, 重复返回 false)
[[nodiscard]] bool registerToMasterFactory();

}  // namespace unistackbot_serial

#endif

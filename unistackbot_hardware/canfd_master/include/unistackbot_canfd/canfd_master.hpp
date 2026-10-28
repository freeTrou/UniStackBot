#ifndef UNISTACKBOT_CANFD__CANFD_MASTER_HPP_
#define UNISTACKBOT_CANFD__CANFD_MASTER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "unistackbot_bus/master_base.hpp"
#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/node_safety.hpp"

/*
 CanFdMaster —— SocketCAN 总线骨架的 MasterBase 实现 (2026-10-28, 设计
 docs/bus/socketcan_master_design.md + 达妙修订 §8; serial_master 骨架第二实例)。
 单类 1+1 (用户裁决 2026-10-27): CAN 无 Framer (内核帧对象) / 无传输接口抽象
 (socket 住 Impl) / vcan 即测试缝——serial 五文件里的三件是串口特有包袱。
 总线线程模型 (设计 §1): FIFO (cfg.thread_priority, 建议 90)/绑核, 周期 =
 cfg.rate_hz, 与 serial 相反的**排空-发送**结构 (设计 §2 "RX 必须非阻塞无条件"):
   rxDrain        非阻塞排空上一拍命令的回显 (recvmsg+内核时间戳, 帧 cap 硬上限)
   状态机推进     反馈喂 NodeSafety + 清故障握手确认 (同线程零交接)
   txCommands     逐节点意图阶梯生成命令 → encode → write (can_send_id 帧头)
   publishSnapshot 陈旧判定 → 粗态覆盖 → DynamicSpLatest 发布
 ── 与 serial 的结构差异 (CAN 本质, 勿回退) ──
   · 无锁步时隙: CAN 命令-回显模型 (发 N 帧收 N 帧, 总线仲裁排序), 泵循环内
     零等待——**拍级**追帧守卫 (逾期超一拍重锚, 绝不补发; serial 是时隙级)
   · CAN ID 分发: 帧头 can_id = 寻址 (数据段不含 ID, dm_motor_core 域约定),
     recv_id → 节点槽 (装配期唯一性校验——分发键撞车 = 静默互吃帧)
   · 错误帧通道: CAN_ERR_BUSOFF → 本地急停旗 (停 TX) / CAN_ERR_RESTARTED → 恢复;
     其余九类计数记账 (设计 §4 第一版策略: 记录为主)
   · **每节点每拍必发一帧** (P1 红线): 命令-回显协议下不发命令 = 无反馈 = 心跳
     看门狗饿死——意图阶梯 (急停/清故障/使能/断使能/idle/命令流) 保证每拍每节点
     恰有一帧。**禁止"命令无变化跳发"优化** (设计 §3 该策略仅适用 CANopen PDO 类)
 双模 (2026-10-21 用户裁决): socket 恒开 CAN_RAW_FD_FRAMES (经典帧不受影响);
 TX 按协议 FrameSpec.frame_len 分型 (≤8 → can_frame 16B / >8 → canfd_frame 72B)。
 v1 消费者达妙 = 纯经典。
 链路装配 (P7): 主站不配 ip link (bitrate/restart-ms/txqueuelen 由外部脚本);
 bind 失败 = 接口不存在, fail-fast。
 安全默认: start 后未 publish 过命令 = 安全怠速 (idle_command, DmSafety=锚定
 阻尼 MIT 维持保持力); quick_stop 原子旗压一切 (含意图)。慢通道 kUnsupported。
 生命周期: stop() 关 socket、start() 复位闩锁/循环态——可重启。
*/

namespace unistackbot_canfd
{

class CanFdMaster final : public unistackbot_bus::MasterBase
{
public:
	// 遥测 (测试/诊断; copy 读, 总线线程侧为唯一写者时近似一致)
	struct Telemetry
	{
		std::uint64_t tx_frames = 0;        // 发出命令帧数
		std::uint64_t tx_failed = 0;        // write 失败 (含 ENOBUFS=qdisc 溢出)
		std::uint64_t rx_frames = 0;        // decode 成功的反馈帧数
		std::uint64_t rx_rejected = 0;      // decode 拒收
		std::uint64_t rx_nofit = 0;         // CAN ID 不在节点表
		std::uint64_t rx_capped = 0;        // 排空触顶被截断的帧数 (P4 防御)
		std::uint64_t err_frames = 0;       // 错误帧 (九类, 计数记账)
		std::uint64_t busoff_events = 0;    // BUSOFF 事件 (急停旗 + 停 TX)
		std::uint64_t skipped_cycles = 0;   // 拍级追帧跳过 (健康签名=0)
		std::uint64_t last_rx_ts_ns = 0;    // 最近收帧内核时间戳 (SO_TIMESTAMPNS)
	};

	CanFdMaster();
	~CanFdMaster() override;

	[[nodiscard]] bool start(const unistackbot_bus::MasterConfig &cfg) override;
	void stop() override;
	void publish_cmd(const unistackbot_protocol::NodeCommand *cmds, std::size_t n) override;
	using MasterBase::publish_cmd;   // vector 便利重载不因 override 遮蔽
	[[nodiscard]] bool take_state(std::vector<unistackbot_bus::NodeSnapshot> &out) override;
	void quick_stop() override;
	void enable(std::uint8_t node_id) override;
	void disable(std::uint8_t node_id) override;
	void fault_clear() override;
	[[nodiscard]] unistackbot_statemachine::NeutralState state() const override;
	[[nodiscard]] unistackbot_bus::ParamStatus read_param(std::uint8_t, std::uint32_t, std::uint64_t &, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}
	[[nodiscard]] unistackbot_bus::ParamStatus write_param(std::uint8_t, std::uint32_t, std::uint64_t, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}

	// 单节点安全状态机只读访问 (测试消费; 使能/断使能/清故障经意图接口)
	[[nodiscard]] const unistackbot_statemachine::NodeSafety &node_safety(std::size_t index) const;

	[[nodiscard]] Telemetry telemetry() const { return telemetry_; }
	[[nodiscard]] bool running() const { return running_.load(); }

private:
	// ── 总线循环结构 (全部总线线程侧调用; 详见文件头注释) ──
	void busLoopMain();
	void rxDrain(std::size_t cap);
	void txOne(std::size_t k);
	void publishSnapshot();

	struct Impl;
	std::unique_ptr<Impl> impl_;
	std::atomic<bool> running_{false};
	mutable Telemetry telemetry_;
};

// 组合根调用: 向 bus 工厂注册 "canfd" (幂等, 重复返回 false)
[[nodiscard]] bool registerToMasterFactory();

}  // namespace unistackbot_canfd

#endif

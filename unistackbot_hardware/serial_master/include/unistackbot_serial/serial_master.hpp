#ifndef UNISTACKBOT_SERIAL__SERIAL_MASTER_HPP_
#define UNISTACKBOT_SERIAL__SERIAL_MASTER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>

#include "unistackbot_bus/master_base.hpp"
#include "unistackbot_serial/framer.hpp"
#include "unistackbot_serial/io_transport.hpp"

// SerialMaster —— 串口总线骨架的 MasterBase 实现 (切片1步4, 2026-09-24)。
//
// 泵线程模型 (设计 rs485_master_design.md §3): FIFO 85/核2 (rt_tune, 失败优雅降级),
// 周期 = cfg.rate_hz, 每拍逐节点时隙调度:
//   [等时隙期限(绝对节拍)] → encode (quick_stop 置位时全体改发停机帧) → 单帧单 write
//   → 非阻塞排干读 → framer 切帧 → decode (按反馈自带 node_id 配对) → 更新节点表
// **落后追帧纪律**: 逾期超一个时隙即跳到未来期限, 绝不补发 (FIFO 同优先级不轮转教训)。
// **命令年龄 1-2 拍**: 不等本拍响应 (盲发), SpLatest 取最新。
// 安全默认: start 后未 publish 过命令 = 持续发停机帧+看门狗位 (安全怠速)。
// 停机: stop_flag → 线程退出 (帧停 → 驱动器固件看门狗兜底)。
//
// 交换: BusCommand/BusState 经 SpLatest (POD 定长, 零锁零等待)。
// 慢通道: read_param/write_param 返回 kUnsupported (unitree 纯周期帧协议无此能力;
// 有慢通道的协议经子类/扩展按加法纪律补)。

namespace unistackbot_serial
{

class SerialMaster final : public unistackbot_bus::MasterBase
{
public:
	// 遥测 (测试/诊断; copy 读, 泵线程侧为唯一写者时近似一致)
	struct Telemetry
	{
		std::uint64_t tx_frames = 0;      // 发出命令帧数
		std::uint64_t rx_frames = 0;      // decode 成功的反馈帧数
		std::uint64_t rx_rejected = 0;    // decode 拒收 (坏 CRC/坏头)
		std::uint64_t rx_nofit = 0;       // 反馈 node_id 不在节点表
		std::uint64_t skipped_slots = 0;  // 追帧跳过的时隙数 (健康签名=0)
	};

	SerialMaster();
	~SerialMaster() override;

	// 装配期注入 transport (测试/假从站/智能 dongle); 不注入则 start() 按
	// cfg.endpoint 打开 TermiosTransport。必须在 start() 之前调用。
	void attach_transport(std::unique_ptr<IoTransport> io);

	[[nodiscard]] bool start(const unistackbot_bus::MasterConfig &cfg) override;
	void stop() override;
	void publish_cmd(const unistackbot_bus::BusCommand &cmd) override;
	[[nodiscard]] bool take_state(unistackbot_bus::BusState &out) override;
	void quick_stop() override;
	[[nodiscard]] unistackbot_statemachine::NeutralState state() const override;
	[[nodiscard]] unistackbot_bus::ParamStatus read_param(std::uint8_t, std::uint32_t,
		std::uint64_t &, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}
	[[nodiscard]] unistackbot_bus::ParamStatus write_param(std::uint8_t, std::uint32_t,
		std::uint64_t, int) override
	{
		return unistackbot_bus::ParamStatus::kUnsupported;
	}

	[[nodiscard]] Telemetry telemetry() const { return telemetry_; }
	[[nodiscard]] bool running() const { return running_.load(); }

private:
	void pumpMain();
	void drainAndDecode();

	struct Impl;
	std::unique_ptr<Impl> impl_;
	std::atomic<bool> running_{false};
	mutable Telemetry telemetry_;   // 单读者近似 (测试轮询), 生产诊断走 BusState 扩展
};

// 组合根调用: 向 bus 工厂注册 "serial" (幂等, 重复返回 false)
[[nodiscard]] bool registerToMasterFactory();

}  // namespace unistackbot_serial

#endif

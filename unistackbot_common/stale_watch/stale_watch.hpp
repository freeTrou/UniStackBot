#ifndef UNISTACKBOT_COMMON__STALE_WATCH_HPP_
#define UNISTACKBOT_COMMON__STALE_WATCH_HPP_

#include <cstdint>

namespace unistackbot_common
{

/*
 * 陈旧看门狗 (0c, 2026-09-21) —— 值通道消费者的"上游还活着吗"判定。
 *
 * 配合 SpLatest::seq() 零拷贝轮询: 调用方每控制周期喂一次通道当前 seq,
 * 本类只数"seq 多少拍没变"—— 无 syscall、无时钟、无分配, RT 热路径零负担。
 *
 * 状态机 (三态, 边沿即事件):
 *   IDLE  从未见过 seq 变化 —— 无流 = 待命, 不是故障 (控制器激活后等首条命令)
 *   LIVE  流活着 (seq 在动)
 *   STALE 流曾活, 已连续 stale_cycles 拍无变化 —— 上游断流
 *
 * 判据语义: seq 变化 = 收到**消息** (不是值变化) —— 上层重复发同一目标仍算活着,
 * "还在线"与"还在变"是两个问题, 本类只答前者。
 *
 * 阈值 stale_cycles = 0 → 永不 STALE (功能关闭); 调用方拿 ms 参数换算:
 *   cycles = static_cast<uint32_t>(timeout_ms * update_rate_hz / 1000.0)
 * 边沿消费推荐姿势 (转换即事件, 无重复触发):
 *   const auto st = watch.tick(ch.seq());
 *   if (st == State::STALE && prev != State::STALE) { 进入断流: 起减速 }
 *   prev = st;
 */
class StaleWatch
{
public:
	enum class State : uint8_t
	{
		IDLE = 0,
		LIVE = 1,
		STALE = 2,
	};

	// stale_cycles: 断流判定阈值 (控制周期数); 0 = 关闭 (永不离开 LIVE)
	explicit StaleWatch(uint32_t stale_cycles = 0)
		: stale_cycles_(stale_cycles)
	{
	}

	// 每控制周期喂通道当前 seq (如 SpLatest::seq()); 返回本周期状态。
	// 重入/多线程: 无 —— 本类只在 RT 消费线程使用 (单线程假设, 与 SpLatest 读者契约一致)
	State tick(uint64_t seq)
	{
		if (seq != last_seq_)
		{
			last_seq_ = seq;
			since_ = 0;
			state_ = State::LIVE;
		}
		else
		{
			++since_;   // 静默拍持续计数 (STALE 后也数: 诊断"死了多久")
			if (state_ == State::LIVE && stale_cycles_ != 0u && since_ > stale_cycles_)
			{
				state_ = State::STALE;
			}
		}
		// STALE 下 seq 恢复变化 → 上面首分支转 LIVE; IDLE 下 seq 不动 → 保持 IDLE
		return state_;
	}

	[[nodiscard]] State state() const {return state_;}

	// 距上次 seq 变化已过周期数 (观测/诊断用)
	[[nodiscard]] uint32_t cyclesSinceUpdate() const {return since_;}

	// 生命周期复位 (on_activate 调用): 回 IDLE, 忘掉旧流 —— 重激活不继承断流态,
	// 首个新 seq 到来时重新 LIVE
	void reset()
	{
		state_ = State::IDLE;
		since_ = 0;
		last_seq_ = 0;
	}

private:
	uint32_t stale_cycles_{0};
	State state_{State::IDLE};
	uint32_t since_{0};        // seq 未变的连续周期数
	uint64_t last_seq_{0};     // 上次见到的 seq (0 = 从未; 首个非零 seq 必然触发 LIVE)
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__STALE_WATCH_HPP_

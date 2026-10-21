#ifndef UNISTACKBOT_COMMON__DYNAMIC_SP_LATEST_HPP_
#define UNISTACKBOT_COMMON__DYNAMIC_SP_LATEST_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace unistackbot_common
{

/*
 * DynamicSpLatest —— 装配期定容的动态尺寸双缓冲覆盖写/取最新原语 (SPSC)。
 *
 * 与 SpLatest 的差别只有一个: 缓冲尺寸**运行期装配时定** (init(n)), 不再是类型
 * 布局的编译期常量 —— 消除 "节点数上限" 这类容量常量 (2026-10-21 组合模型裁决:
 * 机器人 = 单/多总线 × 多协议 × 节点表, 节点数是配置轴)。
 *
 * RT 纪律 (与 SpLatest 同款完整保持):
 *   - 分配只发生在 init() (生命周期边界, 非 RT) —— init 后两缓冲尺寸永不变,
 *     publish/read 全路径零分配零 new;
 *   - 写 = std::copy 元素到后备缓冲 (元素 trivially copyable → trivial 拷贝);
 *   - 读 = 拷到**调用方预分配**的 dst (零分配);
 *   - 再次 init() = 重装配, 只允许在无读写者时 (stop 后) 调用。
 *
 * 算法: 与 SpLatest 同款宣告式 seqlock (奇=写入中/偶=稳定可读, 先宣告后写槽,
 * 读者奇偶检查先于任何数据字节 —— SpLatest 踩坑记录直接继承)。
 *
 * T 约束: trivially copyable 元素 (NodeCommand/NodeFeedback 即此类)。
 * 单写者前提由使用架构保证 (同 SpLatest, 本类不做运行期检查)。
 */
template <typename T>
class DynamicSpLatest
{
	static_assert(std::is_trivially_copyable_v<T>, "DynamicSpLatest requires trivially copyable element type");

	static constexpr bool kLockFree64 = std::atomic<uint64_t>::is_always_lock_free;
	using seq_t = std::conditional_t<kLockFree64, uint64_t, uint32_t>;
	static_assert(std::atomic<seq_t>::is_always_lock_free, "seq atomic must be lock-free on target platform");

public:
	// init 占位判据 / 帧类别 —— 语义与 SpLatest 逐字相同 (init 后 seq=2; 首次真实
	// publish 后 ≥4; 0=从未发布/奇=写入中/偶=稳定)
	static constexpr uint64_t kInitFrameSeq = 2;

	enum class FrameKind : int32_t
	{
		kNone = 0,   // 重试预算耗尽 — dst 保持既有值 = 上一帧 (持久接收缓冲下即语义)
		kInit = 1,   // init 占位帧 (装配期初值, 调用方应忽略)
		kLive = 2,   // 真实发布帧
	};

	// 重试预算: 载荷 = n×sizeof(T), 按总量三级自适应 (SpLatest 同款档位, 阈值按
	// 总字节算而非 sizeof(T))
	static constexpr int32_t kReadRetries = 3;

	// seq → 帧类别 (输入契约同 SpLatest: 恒为偶数)
	[[nodiscard]] static FrameKind frameKind(uint64_t seq)
	{
		if (seq == 0)
		{
			return FrameKind::kNone;
		}
		return (seq <= kInitFrameSeq) ? FrameKind::kInit : FrameKind::kLive;
	}

	DynamicSpLatest() = default;

	// 装配期初始化: n=元素数 (节点数), initial=逐元素初值。两缓冲 resize+填充 ——
	// **此调用后尺寸永不变** (publish/read 不分配的根基); 重 init 须无读写者。
	void init(std::size_t n, const T &initial)
	{
		slots_[0].assign(n, initial);
		slots_[1].assign(n, initial);
		n_ = n;
		current_.store(0, std::memory_order_relaxed);
		published_.store(2, std::memory_order_relaxed);
	}

	[[nodiscard]] std::size_t size() const { return n_; }

	// 写者(唯一): 覆盖写, wait-free。n 必须等于 size() (契约: 尺寸装配期定,
	// 不符返回 false —— 这是调用方 bug, 非运行态)。
	bool publish(const T *data, std::size_t n)
	{
		if (data == nullptr || n != n_)
		{
			return false;
		}
		const uint32_t target = static_cast<uint32_t>(1 - current_.load(std::memory_order_relaxed));
		published_.fetch_add(1, std::memory_order_acq_rel);   // →奇数: 宣告写入开始
		// 元素拷贝: T trivially copyable, 尺寸 init 后固定 → 零分配
		T *dst = slots_[target].data();
		for (std::size_t i = 0; i < n; ++i)
		{
			dst[i] = data[i];
		}
		std::atomic_thread_fence(std::memory_order_release);
		current_.store(target, std::memory_order_release);
		if constexpr (sizeof(seq_t) == 4)
		{
			const seq_t prev = published_.fetch_add(1, std::memory_order_release);
			if (static_cast<seq_t>(prev + 1) == 0)
			{
				published_.fetch_add(2, std::memory_order_release);
			}
		}
		else
		{
			published_.fetch_add(1, std::memory_order_release);
		}
		return true;
	}

	bool publish(const std::vector<T> &data)
	{
		return publish(data.data(), data.size());
	}

	// 读者(唯一): 取最新完整快照到 dst (调用方预分配, cap ≥ size())。
	// true = dst 为 seq_out 对应完整快照; false = 尚未发布/写入中/撕裂 —— dst 保持。
	bool read(T *dst, std::size_t cap, uint64_t &seq_out) const
	{
		if (dst == nullptr || cap < n_)
		{
			return false;
		}
		const seq_t s0 = published_.load(std::memory_order_acquire);
		if (s0 == 0 || (s0 & 1u) != 0u)
		{
			return false;
		}
		const uint32_t idx = current_.load(std::memory_order_acquire);
		const T *src = slots_[idx].data();
		for (std::size_t i = 0; i < n_; ++i)
		{
			dst[i] = src[i];   // 逐元素拷到调用方 dst (失败不污染: 校验后覆写)
		}
		std::atomic_thread_fence(std::memory_order_seq_cst);
		const seq_t s1 = published_.load(std::memory_order_acquire);
		if (s0 != s1)
		{
			tears_.fetch_add(1, std::memory_order_relaxed);
			return false;   // 拷贝窗口内有新发布 → dst 弃用, 调用方沿用旧值
		}
		seq_out = s1;
		return true;
	}

	// 宽松取值: 要么最新帧要么上一帧 (SpLatest::readLastFrame 同款: 有界重试+pause
	// 节流; 档位按总载荷字节)。dst = 持久预分配缓冲 (kNone 时保持既有值)。
	FrameKind readLastFrame(T *dst, std::size_t cap, uint64_t &seq_out) const
	{
		const int32_t pause_n = (n_ * static_cast<std::size_t>(sizeof(T)) > 4096u) ? 128 : (n_ * sizeof(T) > 1024u) ? 64 : 16;
		for (int32_t attempt = 0; attempt < kReadRetries; ++attempt)
		{
			if (read(dst, cap, seq_out))
			{
				return frameKind(seq_out);
			}
			for (int32_t p = 0; p < pause_n; ++p)
			{
#if defined(__x86_64__) || defined(__i386__)
				asm volatile("pause" ::: "memory");
#elif defined(__aarch64__) || defined(__arm__)
				asm volatile("yield" ::: "memory");
#else
				std::atomic_thread_fence(std::memory_order_relaxed);
#endif
			}
		}
		return FrameKind::kNone;
	}

	// 零拷贝看门狗轮询
	[[nodiscard]] uint64_t seq() const
	{
		return published_.load(std::memory_order_acquire);
	}

	[[nodiscard]] uint32_t tears() const
	{
		return tears_.load(std::memory_order_relaxed);
	}

private:
	std::vector<T> slots_[2];                        // 双缓冲 (init resize, 之后尺寸只读)
	std::size_t n_ = 0;                              // 元素数 (init 后只读)
	alignas(64) std::atomic<seq_t> published_{0};
	alignas(64) std::atomic<uint32_t> current_{0};
	alignas(64) mutable std::atomic<uint32_t> tears_{0};
};

}  // namespace unistackbot_common

#endif  // UNISTACKBOT_COMMON__DYNAMIC_SP_LATEST_HPP_

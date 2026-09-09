#ifndef UNISTACKBOT_COMMON__MPSC_RING_HPP_
#define UNISTACKBOT_COMMON__MPSC_RING_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace unistackbot_common
{

// 自旋等待的体系结构提示 (x86 pause / ARM yield; 评审 3.2: 明确语义, 不留模糊)
inline void cpu_relax() noexcept
{
#if defined(__x86_64__) || defined(__i386__)
	__builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
	asm volatile("yield" ::: "memory");
#else
	std::atomic_signal_fence(std::memory_order_seq_cst);
#endif
}

/*
 * MPSC 覆盖式无锁环: 多写单读, 满则丢旧保新 (最新 N 条恒完整可达), push 恒成功。
 * 家族分工: SpLatest=最新 1 个; SpscRing=单写单读逐条必达; 本组件=多写单读丢旧保新。
 * 典型场景: 实时日志缓冲 / 传感器滑动窗口 / 音视频环形缓冲。
 *
 * 契约:
 *   多写者(任意数量) / 单读者; 全接口无阻塞语义 (push 最坏 = 逐出自援, 确定性 ≤N 次 CAS)
 *   元素可平凡拷贝、不含指针; 容量 2 的幂 (static_assert 把关)
 *   一次定容分配, 此后零 malloc; 游标/槽 seq 64 位单调 (回绕 ~亿年, 无需保护)
 *
 * 状态协议 (cell c 在代 g ≡ c (mod N)), seq 编码 2g/2g+1:
 *   seq == 2g     空 (初始 seq=2c; 由上一代 head 通过者标记 2(g)… 即 2*(hh+N))
 *   seq == 2g+1   已发布, 待消费
 *   通过位置 hh (消费或逐出): CAS head hh→hh+1 的赢家标记 seq = 2*(hh+N) (释放给下一代)
 *   乘 2 编码的理由 (压测换来的教训): N=1 时 g+1 == g+N, "已发布"与"空"三态坍缩同值,
 *   状态机错乱 —— 2g/2g+1 恒两态不撞, 回绕周期仍 ~1.4 亿年 @1kHz
 *   seq 是唯一状态权威; head 只是计数 —— 评审修复记录: 初版用 pos-head<N 判槽释放,
 *   消费者 CAS 与标空之间的窗口使生产者提前写入被覆盖 (丢数据) + 逐出可越过在途占位
 *   (下溢死循环) —— 两缺陷同根, 一并消除: 生产者等 seq==2*pos, 逐出者只过已发布记录
 *
 * 消费读契约 (UB 已消除, 裁定记录):
 *   pop 的载荷读取可能与"逐出后的槽重写"交叠 —— 载荷存储为原子字节数组
 *   (std::atomic<uint8_t>[sizeof(T)]), 逐字节 relaxed 存取: 交叠从"未定义行为"变为
 *   "定义良好的竞争" (单字节原子永不撕裂), 混合的新旧内容由 seq 双验证照旧兜底丢弃。
 *   代价: push 逐字节 store 替代 memcpy (128B 场景 +~50ns, 相对周期预算 0.005%, 无感);
 *   x86 上 relaxed 字节原子 = 普通 mov, ARM 上 = strb/ldrb, 零额外指令。
 *   演进史: v1 为 memcpy + 双层屏障 (内核 seqlock 惯例, 显式接受 UB) —— 评审重新算账
 *   (倍数评估失真, 绝对代价无感) 后升级, UB 归零、TSAN 免疫、未来编译器免疫
 *
 * 无弃单论证: 逐出只过已发布 → 未发布占位永不被越过 → 等待者恒有 pos ≥ head
 *   (无 2.2 下溢路径); head 的暂时阻塞只来自"在途 memcpy" (其写者不等待任何人) → 有界
 */
template <typename T, size_t Capacity>
class MpscRing
{
	static_assert(Capacity != 0 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");
	static_assert(std::is_trivially_copyable_v<T>, "MpscRing requires a trivially copyable POD type");

public:
	MpscRing() noexcept
	{
		for (size_t i = 0; i < Capacity; ++i)
		{
			cells_[i].seq.store(2 * i, std::memory_order_relaxed);   // 初始: 槽 i 空待代 i (编码 2g)
		}
	}

	// 生产者(任意线程): 恒成功 —— 满时逐出自援, 最坏 ≤N 次 CAS (消费者死亡也不挂死业务线程)
	void push(const T & value) noexcept
	{
		// 占位: FAA 天然多生产者互斥 (每人独得一个 pos)
		const uint64_t pos = tail_.fetch_add(1, std::memory_order_relaxed);
		Cell & cell = cells_[pos & kModMask];

		// 等本槽空 (seq==2*pos): 由上一代 head 通过者的标记产生
		while (cell.seq.load(std::memory_order_acquire) != 2 * pos)
		{
			uint64_t hh = head_.load(std::memory_order_acquire);
			if (pos - hh < Capacity)
			{
				// 环未满: 槽只是还没轮到释放 (在途写入/排队中) → 无需逐出, 等待即可
				// (评审优化: 避免未满时的无谓 head CAS 竞争)
				cpu_relax();
				continue;
			}
			// 真满 → 逐出: 只逐"已发布"记录 (seq==2*hh+1) —— 永不越过在途写入
			Cell & oldest = cells_[hh & kModMask];
			if (oldest.seq.load(std::memory_order_acquire) == 2 * hh + 1)
			{
				if (head_.compare_exchange_strong(hh, hh + 1, std::memory_order_acq_rel, std::memory_order_acquire))
				{
					// 赢家释放槽: 标记下一代空; 输家重载 head 重试
					oldest.seq.store(2 * (hh + Capacity), std::memory_order_release);
					evicted_.fetch_add(1, std::memory_order_relaxed);
				}
			}
			else
			{
				cpu_relax();   // head 上是在途写入 → 有界稍候 (写者仅 memcpy, 无等待依赖)
			}
		}
		// 本代唯一写者 (占位互斥 + seq 门双重保证); 逐字节原子写: 消除读写交叠的 UB
		// (x86 relaxed 字节原子 = 普通 mov; memcpy 会在"消费者可能正在读"的槽上留下竞争)
		const uint8_t * src = reinterpret_cast<const uint8_t *>(&value);
		for (size_t b = 0; b < sizeof(T); ++b)
		{
			cell.bytes[b].store(src[b], std::memory_order_relaxed);
		}
		// 发布: acquire 侧见到 2*pos+1 时, 载荷必完整可见 (release 与逐字节 store 的顺序)
		cell.seq.store(2 * pos + 1, std::memory_order_release);
	}

	// 消费者(唯一): 取最旧未消费条目; 返回 false = 已消费完当前可达前缀
	// (false ≠ 永久空: 在途写入发布后即可再 pop —— 前缀语义, JCTools MPSC 同款)
	bool pop(T & out) noexcept
	{
		for (;;)
		{
			uint64_t h = head_.load(std::memory_order_acquire);
			Cell & cell = cells_[h & kModMask];
			const uint64_t s1 = cell.seq.load(std::memory_order_acquire);
			if (s1 != 2 * h + 1)
			{
				return false;   // 空 / head 上为在途写入
			}
			// 逐字节原子读到局部缓冲: 交叠为定义良好的竞争, 混合内容由校验兜底;
			// 局部缓冲同时保证契约 "false 时 out 保持原值" (旧版 continue 路径会污染 out, 顺手修正)
			uint8_t buffer[sizeof(T)];
			for (size_t b = 0; b < sizeof(T); ++b)
			{
				buffer[b] = cell.bytes[b].load(std::memory_order_relaxed);
			}
			// seq_cst 全屏障 (第六轮评审裁定): 需要**双向**约束 ——
			//   ① 字节 load 不得下移到二次 seq 检查之后 (acquire fence 不保证此方向)
			//   ② 二次 seq 检查不得上移到字节 load 之前 (release fence 不保证此方向 —— 评审方案二的对称洞)
			// x86 = mfence / ARM = dmb ish, pop 在后端非 RT 线程, ~20ns 无感。
			// 注: 载荷全原子化后已无普通内存访问, 旧 signal_fence 冗余, 删
			std::atomic_thread_fence(std::memory_order_seq_cst);
			if (cell.seq.load(std::memory_order_acquire) != s1)
			{
				continue;   // 拷贝期间槽已换代 (被逐出重写) → 本条作废, 从最新 head 重试
			}
			if (head_.compare_exchange_strong(h, h + 1, std::memory_order_acq_rel, std::memory_order_acquire))
			{
				std::memcpy(&out, buffer, sizeof(T));   // 校验已过, 缓冲内容完整 → 落地
				cell.seq.store(2 * (h + Capacity), std::memory_order_release);   // 释放槽 (与逐出标记同值, 两路径收敛)
				return true;
			}
			// CAS 失败 = 逐出者抢先通过 → 从最新 head 重试
		}
	}

	// 观测: 逐出累计 (丢旧计数 —— 日志场景的健康指标)
	[[nodiscard]] uint64_t evicted() const noexcept
	{
		return evicted_.load(std::memory_order_relaxed);
	}

	// 观测: 近似堆积数 (仅诊断); 两次 load 间 head 可能越过旧 t, 下溢保护归零 (评审 3.3)
	[[nodiscard]] size_t size() const noexcept
	{
		const uint64_t t = tail_.load(std::memory_order_acquire);
		const uint64_t h = head_.load(std::memory_order_acquire);
		return (t < h) ? 0 : static_cast<size_t>(t - h);
	}

private:
	static constexpr uint64_t kModMask = Capacity - 1;

	struct Cell
	{
		std::atomic<uint64_t> seq{0};   // 槽状态权威 (空/已发布/释放标记), 见类头协议
		// 载荷: 原子字节存储 —— 读写交叠从 UB 变为定义良好 (混合内容由 seq 双验证兜底)
		std::array<std::atomic<uint8_t>, sizeof(T)> bytes{};
	};

	alignas(64) std::atomic<uint64_t> tail_{0};   // 占位游标 (多写者, FAA)
	alignas(64) std::atomic<uint64_t> head_{0};   // 消费游标 (消费者推进 + 生产者逐出, 全 CAS)
	alignas(64) std::atomic<uint64_t> evicted_{0};
	alignas(64) std::array<Cell, Capacity> cells_;
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__MPSC_RING_HPP_

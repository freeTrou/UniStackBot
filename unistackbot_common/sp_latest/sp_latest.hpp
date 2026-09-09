#ifndef UNISTACKBOT_COMMON__SP_LATEST_HPP_
#define UNISTACKBOT_COMMON__SP_LATEST_HPP_

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace unistackbot_common
{

/*
 * 双缓冲覆盖写/取最新原语 (SPSC)。
 * 写者 publish() wait-free 覆盖写; 读者 read() 单遍取最新完整快照 (撕裂即沿用旧值, 永不阻塞);
 * seq 零拷贝轮询支撑陈旧看门狗。设计契约见同目录 README.md。
 *
 * 单写者前提由使用架构保证 (如"五服务挂同一互斥组"), 本类不做运行期检查。
 */
template <typename T>
class SpLatest
{
	static_assert(std::is_trivially_copyable_v<T>, "SpLatest requires a trivially copyable POD type");

	// seq 位宽自适应 (评审两轮裁决的落点: uint32+补丁 → uint64 → 32 位兼容):
	//   64 位平台 (x86_64/AArch64) → uint64, 无绕回 (1kHz 下 ~2.9 亿年)
	//   32 位平台 → uint32, 编译期启用跳 0 绕回保护 (uint32 原子在一切现实架构上无锁)
	// 两分支均为无锁原子, static_assert 钉死 —— 拿到异构平台编译失败好过静默退化为锁
	static constexpr bool kLockFree64 = std::atomic<uint64_t>::is_always_lock_free;
	using seq_t = std::conditional_t<kLockFree64, uint64_t, uint32_t>;
	static_assert(std::atomic<seq_t>::is_always_lock_free, "seq atomic must be lock-free on target platform");

public:
	// 构造不做任何事 —— 装配由 init() 显式完成 (与全工程"装配期显式初始化"风格一致)。
	// 零态即"从未发布"(seq=0, 成员 NSDMI 已保证), 默认构造后 read 直接 false
	SpLatest() = default;

	// 装配期初始化 (带初值): 读者首拍前即有稳定值; seq 从偶数 2 起算 (0=从未发布/奇=写入中/偶=稳定)。
	// 必须在任何 publish/read 之前调用 (单线程装配期假设, relaxed 足够), 不得与活跃线程并发。
	// 无错误路径, 返回 void 即诚实 (对照 BackendKinematic::init 的 bool: 那里有可失败的校验)。
	void init(const T & initial)
	{
		slots_[0] = initial;
		current_.store(0, std::memory_order_relaxed);
		published_.store(2, std::memory_order_relaxed);
	}

	// 写者(唯一): 覆盖写, wait-free —— 读者慢或不在都不阻塞写者。
	// 踩坑记录: 初版"写槽→翻槽→seq++"在压测中出现脏数据 —— 写槽进行中 seq 未动,
	// 读者 s0==s1 判稳却拷到半新半旧。修正为经典 seqlock 宣告式: 先置奇数宣告"写入中",
	// 读者凭奇偶即可拒绝在写快照, 奇偶检查必须先于任何数据字节。
	void publish(const T & data)
	{
		const uint32_t target = static_cast<uint32_t>(1 - current_.load(std::memory_order_relaxed));
		// acq_rel 宣告 (第七轮评审修复): 原 relaxed + release fence 不约束**后续写槽上移** ——
		// 下一次 publish 的写槽可上移穿过本次宣告序列, 与"读旧 current 槽"的读者同槽交叠
		// (撕裂且 s0==s1 校验通过)。x86 TSO (stores 不重排) 掩盖, ARM 理论风险。
		// acquire 侧挡后续写槽上移过宣告; release 侧兼挡前序 (此处无前序写, 冗余无害)
		published_.fetch_add(1, std::memory_order_acq_rel);   // →奇数: 宣告写入开始
		slots_[target] = data;
		// release: 槽内容与翻槽先于"完成"自增可见 —— 读者 acquire 到偶数时数据必完整
		current_.store(target, std::memory_order_release);
		// →偶数: 写入完成 (与 read 的 acquire 配对)
		if constexpr (sizeof(seq_t) == 4)
		{
			// 32 位 seq 专属绕回保护: 1kHz 下 ~24.9 天绕回, 而 0 有"从未发布"语义, 跳过之
			const seq_t prev = published_.fetch_add(1, std::memory_order_release);
			if (static_cast<seq_t>(prev + 1) == 0)
			{
				published_.fetch_add(2, std::memory_order_release);
			}
		}
		else
		{
			published_.fetch_add(1, std::memory_order_release);   // uint64: ~2.9 亿年才绕回, 无需保护
		}
	}

	// 读者(唯一): 取最新完整快照。单遍直线代码, WCET = 一次拷贝 (无重试循环)。
	// true  = out 为 seq_out 对应的完整快照
	// false = 从未发布(seq==0) / 写入中(奇数) / 撕裂 —— out 保持调用前的值 (调用方沿用旧值即标准降级)
	bool read(T & out, uint64_t & seq_out) const
	{
		// acquire: 与 publish 的完成自增(release) 配对
		const seq_t s0 = published_.load(std::memory_order_acquire);
		if (s0 == 0 || (s0 & 1u) != 0u)
		{
			return false;   // 尚未发布过 / 写入进行中
		}
		const uint32_t idx = current_.load(std::memory_order_acquire);
		T tmp = slots_[idx];   // 拷到临时量: 失败不污染 out (契约: 失败时 out 保持原值)
		// seq_cst 全屏障 (第七轮评审同步升级, 同 MpscRing 第六轮裁定): 双向约束 ——
		// ① tmp 拷贝不得下移过 s1 校验 (acquire fence 不挡先前 load 后移)
		// ② s1 校验不得上移到拷贝之前 (release fence 不挡后续 load 前移)
		std::atomic_thread_fence(std::memory_order_seq_cst);
		const seq_t s1 = published_.load(std::memory_order_acquire);
		if (s0 != s1)
		{
			tears_.fetch_add(1, std::memory_order_relaxed);
			return false;   // 拷贝窗口内有新发布, 快照存疑 → 沿用旧值
		}
		out = tmp;
		seq_out = s1;
		return true;
	}

	// 零拷贝看门狗轮询: N 拍不变 = 上游停发 (seq_t 零扩展为 uint64, 恒无损)
	[[nodiscard]] uint64_t seq() const
	{
		return published_.load(std::memory_order_acquire);
	}

	// 撕裂计数 (读者单写, 观测用; 持续增长是写读频率失衡的信号)
	[[nodiscard]] uint32_t tears() const
	{
		return tears_.load(std::memory_order_relaxed);
	}

private:
	// seq 语义: 0=从未发布, 奇=写入中(瞬态), 偶=稳定可读; 单调递增
	alignas(64) std::atomic<seq_t> published_{0};      // 位宽自适应 (见类顶部说明); 与槽位隔离防伪共享
	alignas(64) std::atomic<uint32_t> current_{0};     // 当前稳定槽 (读者从这里拷)
	alignas(64) T slots_[2]{};                          // 双缓冲槽位 (写者永远写非当前槽)
	mutable std::atomic<uint32_t> tears_{0};            // 撕裂计数 (检测到即 +1 并 miss; read 为 const, 计数可变)
};

}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__SP_LATEST_HPP_

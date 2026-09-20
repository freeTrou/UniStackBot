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
	// init 占位帧判据: init() 后首个可读快照的 seq 恒为 2, 写者首次**真实** publish 后 seq≥4。
	// init 初值不一定是实际反馈 (真机总线上线前=占位值; 仿真后端 init 即真值) ——
	// 真机调用方拿到 seq==kInitFrameSeq 的快照应忽略不进控制 (上线前 CM 不动作)。
	// 已知瑕疵 (仅 32 位平台): seq_t=uint32 时 24.9 天绕回, 序列会再次经过 2 →
	// frameKind 误判 kInit 一拍; 64 位平台 (~2.9 亿年) 不涉及, 不修。
	static constexpr uint64_t kInitFrameSeq = 2;

	// 帧类别 —— 通用的"init 占位 vs 真实反馈"判断, 调用方不碰 seq 魔法数。
	enum class FrameKind : int32_t
	{
		kNone = 0,   // 本拍重试未取到新帧 — **out 保持既有值 = 上一帧** (持久接收变量下
		             //   API 即"要么最新要么上一帧"; 非错误, 勿当异常处理)
		kInit = 1,   // init 占位帧 (装配期初值, 非实际反馈 — 真机调用方应忽略)
		kLive = 2,   // 真实发布帧
	};

	// readLastFrame 有界重试预算 (用户终裁 2026-09-20: 3 次):
	// pause 档位按载荷三级自适应 (≤1KB:16 / ≤4KB:64 / >4KB:128)。本机实测 pause≈31ns
	// → 每轮节流 0.5/2/4µs, 3 轮总覆盖 1.5/6/12µs, 覆盖对应写槽;
	// 档位不再上探: 更大档的单轮节流在 2kHz 周期占比过高, 破坏 WCET 的风险大于收益。
	// **物理约束账 (必撞极限实测, 2026-09-20)**: 大载荷的真实瓶颈不在节流预算, 而在
	// "拷贝时长 vs 写者周期" — read 成功率 ≈ 1 − sizeof(T)拷贝时长/写者间隔。真实总线
	// (1kHz, 间隔 1000µs) 下 4KB 载荷成功率 ≈99.9%; 全速背靠背写者 (~3.5µs 间隔) 下
	// 成功率 ~10%, kNone 高是超设计域的正确降级 (零脏数据, 实测 fat-stress 用例)。
	// **载荷上界纪律**: sizeof(T) 应补齐 64 倍数 (slots_[1] 槽对齐的前提, 调用方以
	// reserved 字段补齐并 static_assert 钉死); >16KB 的大数据勿走值通道 — 用 sp_ring
	// (事件通道) 或拆分通道 (勿用指针+池: 回收安全需 hazard/epoch, 重回多变量交叉推理)。
	static constexpr int32_t kReadRetries = 3;
	static constexpr int32_t kRetryPauseN = (sizeof(T) > 4096u) ? 128 : (sizeof(T) > 1024u) ? 64 : 16;

	// seq → 帧类别。输入契约: 须为 read/readLastFrame 返回的 seq_out (恒为偶数——
	// read 只在稳定偶数时成功); 奇数输入未定义 (正常运行不可达)
	[[nodiscard]] static FrameKind frameKind(uint64_t seq)
	{
		if (seq == 0)
		{
			return FrameKind::kNone;
		}
		return (seq <= kInitFrameSeq) ? FrameKind::kInit : FrameKind::kLive;
	}

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
	// true  = out 为 seq_out 对应的完整快照 (seq_out==kInitFrameSeq 为 init 占位帧, 见常量处说明)
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

	// 宽松取值: **要么最新帧, 要么上一帧** (用户需求终稿 2026-09-20)。
	// 重试成功 → 最新帧 (kLive/kInit); 重试耗尽 → kNone, out 保持既有值 = 上一帧。
	// **推荐姿势 = 持久接收变量 + 一个 last_live 布尔** (防 init 残留):
	//   Feedback fb;  bool live = false;              // 循环外持久
	//   switch (ch.readLastFrame(fb, s)) {
	//     case kLive: live = true; /* 用 fb */ break;
	//     case kInit: live = false; break;            // 总线未上线, 忽略
	//     case kNone: if (live) { /* 用 fb = 上一帧 */ } break;
	//   }
	// 上一帧的载体是 out 参数本身 — 组件零缓存、零旧槽推理, kNone 分支零代码。
	// 首次调用即 kNone (从未成功过) 时 out = 调用方初始化值 (与 init 双保险)。
	//
	// 实现设计裁决 (2026-09-20, 三轮外部评审三轮边界 bug 后的终解): **只依赖已证明原语**。
	// 旧实现"读旧槽自证"需 current_(idx/idx2) 与 published_(s0/s1) 四读点交叉推理 — 两个独立
	// 原子变量的交错空间组合爆炸, 证明不可维护 (≥4 → ≥3 → 奇偶分界, 三轮全在新交错上翻车)。
	// 终解 = 有界重试 read(): read 自身是单变量夹逼 (published_ 两读夹一次拷贝, 闭式安全),
	// 组合零新增推理; 重试间 pause 节流推进时间跨过写者的写槽窗口, 成功拿到的是**新帧**
	// (新鲜度优于旧实现的"上一帧")。init 占位帧可判断: 成功返回 frameKind(seq_out),
	// seq==kInitFrameSeq → kInit, 调用方以 kind==kLive 门禁控制路径即可挡住占位帧。
	// kNone = 重试预算耗尽 (写者高频连发 / 读者异常卡顿): out 保持既有值 = 上一帧
	// (持久接收变量下即需求语义, 非异常; 临时量下 out 为垃圾, 故姿势要求持久变量)。
	// WCET 账: 常态 1 次拷贝; 撞窗典型为早期失败 (s0 奇 → 2 原子读 0 拷贝); 理论最坏
	// 5×(3 原子读 + 1 拷贝, 撕裂型失败) + 节流 (~1-2.5µs 按档)。
	// tears 语义 (外部评审四轮钉死): 计的是**失败的 read 尝试数** — readLastFrame 一次
	// kNone 会 +5, 与 read() 混用时也逐次累计; 监控阈值须按此口径校准。
	FrameKind readLastFrame(T & out, uint64_t & seq_out) const
	{
		for (int32_t attempt = 0; attempt < kReadRetries; ++attempt)
		{
			if (read(out, seq_out))
			{
				return frameKind(seq_out);
			}
			for (int32_t p = 0; p < kRetryPauseN; ++p)
			{
#if defined(__x86_64__) || defined(__i386__)
				asm volatile("pause" ::: "memory");
#elif defined(__aarch64__) || defined(__arm__)
				asm volatile("yield" ::: "memory");   // ARM 交叉编译目标
#else
				std::atomic_thread_fence(std::memory_order_relaxed);   // 未知架构兜底
#endif
			}
		}
		return FrameKind::kNone;
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

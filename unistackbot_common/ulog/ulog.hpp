#ifndef UNISTACKBOT_COMMON__ULOG_HPP_
#define UNISTACKBOT_COMMON__ULOG_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

#include <fcntl.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "mpsc_ring/mpsc_ring.hpp"

// 队列容量 (2 的幂): 默认 8192 条 ≈ 1.2MB; 过载测试可用 -DULOG_QUEUE_CAPACITY=64 缩小
#ifndef ULOG_QUEUE_CAPACITY
#define ULOG_QUEUE_CAPACITY 8192
#endif

// 单例符号可见性硬化 (2026-09-17 评审 P0 实验): 单例静态依赖 inline 符号跨 .so
// 合并 —— 默认可见性下成立; 使用方 -fvisibility=hidden 会把符号藏掉导致各 .so
// 一份实例 (双后端+文件交错, dlopen 实验实测)。对单例入口显式 default 导出,
// hidden 旗标下依然合并; -Bsymbolic 仍不可用 (自绑定, 见 README 注意事项 8)。
#if defined(__GNUC__) || defined(__clang__)
#define ULOG_EXPORT __attribute__((visibility("default")))
#else
#define ULOG_EXPORT
#endif

namespace unistackbot_common
{

enum class ulog_level : uint32_t
{
	debug = 0,
	info = 1,
	warn = 2,
	error = 3,
};

// 组件配置 (init 一次性)
struct ulog_config
{
	bool console = true;             // 终端 sink 开关
	const char * file_path = nullptr;   // 文件 sink 路径; nullptr = 不写文件
	ulog_level level = ulog_level::info; // 默认日志等级 
	size_t max_bytes = 10 * 1024 * 1024;   // 单文件上限 (轮转触发) 10MB
	int backups = 5;                     // 轮转保留份数 (log.1 .. log.N)
};

namespace ulog_detail
{

// 日志记录: 定长 POD (设计 §3; 无全局 seq —— seq 分配与 push 占位是两次独立原子,
// drain 序 ≠ seq 序, 顺序对账靠 evicted + 账目守恒)
struct ULogRecord
{
	uint64_t ts_ns{0};    // 入队时墙钟 (vDSO 取时钟, 无 syscall)
	uint32_t level{0};
	uint32_t tid{0};      // thread_local 缓存的 gettid, 热路径零 syscall
	uint8_t msg_len{0};
	char msg[128]{};      // 预格式化消息 (超长截断于 128)
};
static_assert(sizeof(ULogRecord) == 152, "record layout is part of the contract");
static_assert(std::is_trivially_copyable_v<ULogRecord>, "ULogRecord must stay trivially copyable");

ULOG_EXPORT inline MpscRing<ULogRecord, ULOG_QUEUE_CAPACITY> & ring()
{
	static MpscRing<ULogRecord, ULOG_QUEUE_CAPACITY> r;
	return r;
}

// 前端状态: active=false (未 init/已 shutdown) 时宏直接短路 —— 连格式化都不做, 零成本
ULOG_EXPORT inline std::atomic<bool> active{false};
ULOG_EXPORT inline std::atomic<int> min_level{static_cast<int>(ulog_level::info)};
ULOG_EXPORT inline std::atomic<uint64_t> noop_dropped{0};   // active=false 期间被短路的条数

inline uint64_t now_ns() noexcept
{
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

inline uint32_t cached_tid() noexcept
{
	static thread_local const uint32_t t = static_cast<uint32_t>(::syscall(SYS_gettid));
	return t;
}

// 编码并入队: vsnprintf 到定长缓冲 (调用侧格式化, 无堆分配) → push 恒成功 (满则环内逐旧)
__attribute__((format(printf, 2, 3))) inline void emit(ulog_level level, const char * fmt, ...) noexcept
{
	ULogRecord rec;
	rec.ts_ns = now_ns();
	rec.level = static_cast<uint32_t>(level);
	rec.tid = cached_tid();
	va_list ap;
	va_start(ap, fmt);
	const int n = std::vsnprintf(rec.msg, sizeof(rec.msg), fmt, ap);
	va_end(ap);
	rec.msg_len = (n < 0) ? 0 : static_cast<uint8_t>(n >= static_cast<int>(sizeof(rec.msg)) ? sizeof(rec.msg) - 1 : n);
	ring().push(rec);
}

// 无格式快路径: 纯字面量 memcpy, 连 vsnprintf 都不走 (字面量日志的主流形态)
inline void emit_raw(ulog_level level, const char * msg) noexcept
{
	ULogRecord rec;
	rec.ts_ns = now_ns();
	rec.level = static_cast<uint32_t>(level);
	rec.tid = cached_tid();
	const size_t len = std::strlen(msg);
	rec.msg_len = static_cast<uint8_t>(len > sizeof(rec.msg) ? sizeof(rec.msg) : len);
	for (size_t i = 0; i < rec.msg_len; ++i)
	{
		rec.msg[i] = msg[i];
	}
	ring().push(rec);
}

// ---- 后端 (单线程, 承担全部 I/O; 后端可分配可休眠) ----

class ULog
{
public:
	std::thread thread;
	std::atomic<bool> running{false};
	FILE * file = nullptr;
	std::string path;
	bool console = true;
	size_t max_bytes = 10 * 1024 * 1024;
	int backups = 5;
	uint64_t file_bytes = 0;
	uint64_t evicted_base = 0;   // init 时环内 evicted 的基线 (会话账目独立)
	std::atomic<uint64_t> written{0};   // 已写出行数 (本会话; written + dropped == emitted)

	// flush 确认 (代际计数, 原子): 前端 req++, 后端排空+落盘后 done=req 并 notify。
	// gen 用 atomic + relaxed (锁仅服务于 cv 等待 —— 原子化消除 TSAN 对
	// "锁保护的非原子状态 + cv" 组合的报告, 标准做法)
	std::mutex flush_mtx;
	std::condition_variable flush_cv;
	std::atomic<uint64_t> flush_req_gen{0};
	std::atomic<uint64_t> flush_done_gen{0};

	// 析构兜底 (踩坑记录: 该析构曾在 Backend→LogWriter→ULog 两轮改名中丢失 ——
	// 使用方漏调 ulog_shutdown 时, 静态析构阶段 joinable 线程析构 → std::terminate)。
	// 析构 = 停线程+排空+落盘+关闭, 使"忘记 shutdown"从退出崩溃变为优雅收尾
	~ULog()
	{
		running.store(false, std::memory_order_relaxed);
		if (thread.joinable())
		{
			thread.join();   // 后端循环 ~1ms 一拍, join 有界
		}
		flush_sinks();
		if (file != nullptr)
		{
			std::fclose(file);
			file = nullptr;
		}
	}

private:
	void rotate()
	{
		std::fclose(file);
		char old_path[512];
		char new_path[512];
		for (int i = backups - 1; i >= 1; --i)
		{
			std::snprintf(old_path, sizeof(old_path), "%s.%d", path.c_str(), i);
			std::snprintf(new_path, sizeof(new_path), "%s.%d", path.c_str(), i + 1);
			if (::rename(old_path, new_path) != 0 && errno != ENOENT)
			{
				// ENOENT = 首次轮转时 log.N 尚不存在, 常态; 其余失败 (权限/磁盘)
				// 直写 stderr 诊断 (不得用 ULOG 自身 —— 递归), 轮转尽力继续
				std::fprintf(stderr, "ulog: 轮转 rename 失败 %s -> %s: %s\n",
					old_path, new_path, std::strerror(errno));
			}
		}
		std::snprintf(new_path, sizeof(new_path), "%s.1", path.c_str());
		if (::rename(path.c_str(), new_path) != 0 && errno != ENOENT)
		{
			std::fprintf(stderr, "ulog: 轮转 rename 失败 %s: %s\n",
				path.c_str(), std::strerror(errno));
		}
		file = std::fopen(path.c_str(), "a");
		if (file == nullptr)
		{
			// 直接 stderr (不得用 ULOG 自身 —— 递归)。file 置空后 write_one 跳过
			// 文件写, file_bytes 不再累计, 不会形成"失败循环重触发 rotate"
			std::fprintf(stderr, "ulog: rotate 后重开失败: %s\n", path.c_str());
		}
		file_bytes = 0;
	}

	void write_one(const ULogRecord & rec)
	{
		static const char * kLevelName[] = {"DEBUG", "INFO", "WARN", "ERROR"};
		const char * lvname = (rec.level < 4) ? kLevelName[rec.level] : "UNKNOWN";
		char line[256];
		const time_t sec = static_cast<time_t>(rec.ts_ns / 1000000000ULL);
		const unsigned msec = static_cast<unsigned>((rec.ts_ns % 1000000000ULL) / 1000000ULL);
		// 同秒时间戳缓存 (2026-09-17 评审 P1): localtime_r 内部涉时区数据访问,
		// 每条调用是高 TPS 下的后端瓶颈 —— 秒不变则复用格式化结果
		static time_t last_sec = -1;   // -1 起: 首条必更新缓存 (sec==0 边界)
		static char last_stamp[32];
		static size_t last_sl = 0;
		if (sec != last_sec)
		{
			std::tm tm_buf;
			localtime_r(&sec, &tm_buf);
			last_sl = std::strftime(last_stamp, sizeof(last_stamp), "%Y-%m-%d %H:%M:%S", &tm_buf);
			last_sec = sec;
		}
		const size_t sl = last_sl;
		const int len = std::snprintf(line, sizeof(line), "[%.*s.%03u] [%s] %.*s\n",
			static_cast<int>(sl), last_stamp, msec,
			lvname,   // 越界显式判 "UNKNOWN" (加新级别时同步扩表)
			static_cast<int>(rec.msg_len), rec.msg);
		if (len <= 0)
		{
			return;
		}
		const size_t ulen = static_cast<size_t>(len);
		if (console)
		{
			std::fwrite(line, 1, ulen, stdout);
		}
		if (file != nullptr)
		{
			std::fwrite(line, 1, ulen, file);
			file_bytes += ulen;
			if (rec.level == static_cast<uint32_t>(ulog_level::error))
			{
				std::fflush(file);   // error 级强刷: 崩溃现场尽量保住
			}
			if (file_bytes >= max_bytes)
			{
				rotate();
			}
		}
		written.fetch_add(1, std::memory_order_relaxed);
	}

	void flush_sinks()
	{
		if (console)
		{
			std::fflush(stdout);
		}
		if (file != nullptr)
		{
			std::fflush(file);
		}
	}

public:
	void run()
	{
		prctl(PR_SET_NAME, "ulog", 0, 0, 0);
		// 无害化让路 (职责边界: ulog 保证不干扰采集等 IO 业务 —— 本线程是日志唯一的
		// 落盘执行者, 主动在 CPU/IO 调度上降级; 失败静默, 非特权环境无碍)
		setpriority(PRIO_PROCESS, 0, 10);   // CPU: nice +10, 永远让路
		::syscall(SYS_ioprio_set, 1 /*IOPRIO_WHO_PROCESS*/, 0,
			(2 /*IOPRIO_CLASS_BE*/ << 13) | 7 /*最低档*/);   // IO: best-effort 最低优先
		auto last_flush = std::chrono::steady_clock::now();
		while (running.load(std::memory_order_relaxed))
		{
			bool any = false;
			ULogRecord rec;
			while (ring().pop(rec))
			{
				write_one(rec);
				any = true;
			}
			// flush 请求处理: I/O 在锁外 (持锁做 fflush 是反模式, 也触发 TSAN 报告),
			// 确认与 notify 在锁内 (防丢失唤醒的标准姿势)
			// 代际快照先行 (2026-09-17 评审修复): 声明完成的 must 是快照值 ——
			// 若 store 前又有新 flush 请求到达, 其 waiters 留给下一轮循环
			// (原实现 store(flush_req_gen.load()) 双读竞态, 新请求被过早确认)
			const uint64_t flush_req_snapshot = flush_req_gen.load(std::memory_order_acquire);
			if (flush_req_snapshot != flush_done_gen.load(std::memory_order_acquire))
			{
				// 排空在快照之后补一轮: 满足"排空当前环内全部记录并落盘"的文档语义
				ULogRecord pending;
				while (ring().pop(pending))
				{
					write_one(pending);
				}
				flush_sinks();
				std::lock_guard<std::mutex> lk(flush_mtx);
				flush_done_gen.store(flush_req_snapshot, std::memory_order_relaxed);
				flush_cv.notify_all();
			}
			const auto now = std::chrono::steady_clock::now();
			if (now - last_flush >= std::chrono::milliseconds(100))
			{
				flush_sinks();
				last_flush = now;
			}
			if (!any)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
		// shutdown: 排空后落盘 (被逐出的天然不在待写集合 —— 丢旧语义下的最强 flush)
		ULogRecord rec;
		while (ring().pop(rec))
		{
			write_one(rec);
		}
		flush_sinks();
	}
};

ULOG_EXPORT inline ULog & writer()
{
	static ULog instance;
	return instance;
}

// 组件级引用计数 (2026-09-17): 同进程多个插件/组件共用 ulog 单例 —— 各自
// init/shutdown 不得互踢 (实测场景: ros2_control_node 内 SimControlHardware 与
// CartesianMotionController 并存)。first init 起 writer, 末位 shutdown 才停。
ULOG_EXPORT inline std::mutex & ref_mutex()
{
	static std::mutex m;
	return m;
}

ULOG_EXPORT inline int & ref_count()
{
	static int n = 0;
	return n;
}

}  // namespace ulog_detail

// 初始化 (幂等)。返回 false = 文件打开失败 (此时终端模式仍已启动, 见设计 §7);
// true = 全部就绪。必须在任何日志调用之前完成 (单线程装配期)。
inline bool ulog_init(const ulog_config & cfg)
{
	std::lock_guard<std::mutex> lk(ulog_detail::ref_mutex());
	auto & b = ulog_detail::writer();
	if (b.running.load(std::memory_order_relaxed))
	{
		++ulog_detail::ref_count();   // 幂等 + 计引用 (首配置为准)
		return true;
	}
	ulog_detail::min_level.store(static_cast<int>(cfg.level), std::memory_order_relaxed);
	b.console = cfg.console;
	b.max_bytes = cfg.max_bytes;
	b.backups = cfg.backups;
	b.file = nullptr;
	// 会话账目基线: written/evicted/noop 均为静态累计值, init 清零/记基线使
	// "written + dropped == emitted" 的账目守恒在多次 init/shutdown 会话间独立成立
	b.written.store(0, std::memory_order_relaxed);
	b.evicted_base = ulog_detail::ring().evicted();
	ulog_detail::noop_dropped.store(0, std::memory_order_relaxed);
	if (cfg.file_path != nullptr)
	{
		b.path = cfg.file_path;
		b.file = std::fopen(cfg.file_path, "a");
		if (b.file != nullptr)
		{
			std::fseek(b.file, 0, SEEK_END);
			const long sz = std::ftell(b.file);
			b.file_bytes = sz > 0 ? static_cast<uint64_t>(sz) : 0;
		}
	}
	b.running.store(true, std::memory_order_relaxed);
	b.thread = std::thread([&b]() { b.run(); });
	ulog_detail::active.store(true, std::memory_order_relaxed);
	ulog_detail::ref_count() = 1;
	// 文件开失败: 返回 false 提示, 但终端模式已启动 (设计 §7)
	return !(cfg.file_path != nullptr && b.file == nullptr);
}

// 停机: 停标志 → 后端排空+落盘 → join。此后日志调用静默短路并计入 noop_dropped
inline void ulog_shutdown()
{
	std::lock_guard<std::mutex> lk(ulog_detail::ref_mutex());
	auto & b0 = ulog_detail::writer();
	if (!b0.running.load(std::memory_order_relaxed))
	{
		return;
	}
	if (ulog_detail::ref_count() > 1)
	{
		--ulog_detail::ref_count();   // 还有其他组件在用, 只退引用不停机
		return;
	}
	ulog_detail::ref_count() = 0;
	ulog_detail::active.store(false, std::memory_order_relaxed);
	auto & b = ulog_detail::writer();
	if (!b.running.exchange(false, std::memory_order_relaxed))
	{
		return;
	}
	if (b.thread.joinable())
	{
		b.thread.join();
	}
	if (b.file != nullptr)
	{
		std::fclose(b.file);
		b.file = nullptr;
	}
}

// flush 精确语义 (设计 §6): 排空当前环内全部记录并落盘, cv 确认, 超时保护
inline void ulog_flush()
{
	auto & b = ulog_detail::writer();
	if (!b.running.load(std::memory_order_relaxed))
	{
		return;
	}
	std::unique_lock<std::mutex> lk(b.flush_mtx);
	// release (2026-09-17 评审内存序修复): 本调用之前的 ring push (release 写) 不得
	// 重排到 req 自增之后 —— 后端 acquire 读到新代际时, 待 flush 记录必已入环。
	// relaxed 在 x86 TSO 下侥幸成立, ARM/RISC-V 上 flush 可能空确认。
	// 依赖链: MpscRing::push 的发布 store 是 release (mpsc_ring.hpp, 已核实) ——
	// 该组件若改弱内存序, 此处保证失效, 修改须联动评审
	const uint64_t my_gen = b.flush_req_gen.fetch_add(1, std::memory_order_release) + 1;
	b.flush_cv.wait_for(lk, std::chrono::seconds(5), [&b, my_gen]() { return b.flush_done_gen.load(std::memory_order_relaxed) >= my_gen; });
}

inline void ulog_set_level(ulog_level level)
{
	ulog_detail::min_level.store(static_cast<int>(level), std::memory_order_relaxed);
}

// 丢弃计数 (健康观测, 本会话): 环内逐出 (自 init 基线) + 未激活短路
inline uint64_t ulog_dropped()
{
	const uint64_t ev = ulog_detail::ring().evicted();
	const uint64_t base = ulog_detail::writer().evicted_base;
	return (ev > base ? ev - base : 0) + ulog_detail::noop_dropped.load(std::memory_order_relaxed);
}

inline size_t ulog_pending()
{
	return ulog_detail::ring().size();
}

// 已写出行数 (账目观测: written + evicted == 发出)
inline uint64_t ulog_written()
{
	return ulog_detail::writer().written.load(std::memory_order_relaxed);
}

}  // namespace unistackbot_common

// ---- 调用侧宏: 级别过滤第一句 (级别外零开销), 编译期 printf 格式检查 ----
// 丢弃计数语义 (裁定): 仅"未激活短路"计数 (未 init/已 shutdown);
// 级别过滤是有意为之, 不计入丢弃 —— 混计会让 dropped 失去健康指标意义

#define ULOG_DETAIL_LOG(lv, ...)                                                                            \
	do                                                                                                      \
	{                                                                                                       \
		if (!unistackbot_common::ulog_detail::active.load(std::memory_order_relaxed))                       \
		{                                                                                                   \
			unistackbot_common::ulog_detail::noop_dropped.fetch_add(1, std::memory_order_relaxed);          \
		}                                                                                                   \
		else if (static_cast<int>(lv) >= unistackbot_common::ulog_detail::min_level.load(std::memory_order_relaxed)) \
		{                                                                                                   \
			unistackbot_common::ulog_detail::emit(lv, ##__VA_ARGS__);                                       \
		}                                                                                                   \
	} while (0)

#define ULOG_DEBUG(...) ULOG_DETAIL_LOG(unistackbot_common::ulog_level::debug, __VA_ARGS__)
#define ULOG_INFO(...) ULOG_DETAIL_LOG(unistackbot_common::ulog_level::info, __VA_ARGS__)
#define ULOG_WARN(...) ULOG_DETAIL_LOG(unistackbot_common::ulog_level::warn, __VA_ARGS__)
#define ULOG_ERROR(...) ULOG_DETAIL_LOG(unistackbot_common::ulog_level::error, __VA_ARGS__)

// 无格式快路径: 字面量纯 memcpy, 连 vsnprintf 都不走。
// active 检查与 ULOG_DETAIL_LOG 对齐 (2026-09-17 评审修复): 未激活时不得付出
// now_ns/tid/strlen/拷贝/push 的完整 emit 代价, 更不得触碰未消费的环
#define ULOG_INFO_RAW(msg)                                                                                  \
	do                                                                                                      \
	{                                                                                                       \
		if (unistackbot_common::ulog_detail::active.load(std::memory_order_relaxed))                        \
		{                                                                                                   \
			if (static_cast<int>(unistackbot_common::ulog_level::info) >=                                    \
				unistackbot_common::ulog_detail::min_level.load(std::memory_order_relaxed))                 \
			{                                                                                               \
				unistackbot_common::ulog_detail::emit_raw(unistackbot_common::ulog_level::info, msg);       \
			}                                                                                               \
		}                                                                                                   \
		else                                                                                                \
		{                                                                                                   \
			unistackbot_common::ulog_detail::noop_dropped.fetch_add(1, std::memory_order_relaxed);          \
		}                                                                                                   \
	} while (0)

#endif  // UNISTACKBOT_COMMON__ULOG_HPP_

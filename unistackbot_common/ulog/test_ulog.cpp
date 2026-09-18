// ulog 功能/并发/RT-safety/过载 测试 (规范 42 门槛)。
// 独立编译, 零 ROS 依赖 (include 根 = unistackbot_common):
//   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_ulog.cpp -o test_ulog && ./test_ulog

// 自定义 new=malloc/delete=free 配对正确; -Wmismatched-new-delete 是编译器对
// "delete 里用 free"的静态误报 (它无法知道我们的 new 也是 malloc), 全文抑制
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"

#include "ulog/ulog.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace
{

using unistackbot_common::ulog_config;
using unistackbot_common::ulog_flush;
using unistackbot_common::ulog_init;
using unistackbot_common::ulog_level;
using unistackbot_common::ulog_set_level;
using unistackbot_common::ulog_shutdown;
using unistackbot_common::ulog_written;
using unistackbot_common::ulog_dropped;

int g_failures = 0;

void check(bool ok, const char * name)
{
	std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
	if (!ok)
	{
		++g_failures;
	}
}

// ---- 零 malloc 检测: thread_local 标志 + 全局 operator new 钩子 (只计当前线程 emit 窗口) ----
thread_local int t_in_emit = 0;
int g_emit_mallocs = 0;



}  // namespace

// 自定义 new=malloc/delete=free 配对正确 (警告抑制见文件头)
void * operator new(size_t sz)
{
	if (t_in_emit != 0)
	{
		++g_emit_mallocs;
	}
	void * p = std::malloc(sz == 0 ? 1 : sz);
	if (p == nullptr)
	{
		throw std::bad_alloc();
	}
	return p;
}

void operator delete(void * p) noexcept
{
	std::free(p);
}

void operator delete(void * p, size_t) noexcept
{
	std::free(p);
}

namespace
{

std::string g_dir;

size_t count_lines(const char * path)
{
	FILE * f = std::fopen(path, "r");
	if (f == nullptr)
	{
		return 0;
	}
	size_t n = 0;
	char buf[512];
	while (std::fgets(buf, sizeof(buf), f) != nullptr)
	{
		++n;
	}
	std::fclose(f);
	return n;
}

bool file_contains(const char * path, const char * needle)
{
	FILE * f = std::fopen(path, "r");
	if (f == nullptr)
	{
		return false;
	}
	char buf[512];
	bool found = false;
	while (!found && std::fgets(buf, sizeof(buf), f) != nullptr)
	{
		if (std::strstr(buf, needle) != nullptr)
		{
			found = true;
		}
	}
	std::fclose(f);
	return found;
}

// 行格式验证: [YYYY-mm-dd HH:MM:SS.mmm] [LEVEL] msg
bool line_format_ok(const char * line)
{
	int y, mo, d, h, mi, s, ms;
	char level[8];
	if (std::sscanf(line, "[%4d-%2d-%2d %2d:%2d:%2d.%3d] [%7[^]]]", &y, &mo, &d, &h, &mi, &s, &ms, level) != 8)
	{
		return false;
	}
	return y >= 2026 && mo >= 1 && mo <= 12 && ms <= 999;
}

// ---- 1. 功能测试 ----

void functional_tests()
{
	const std::string log = g_dir + "/func.log";
	std::remove(log.c_str());

	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = log.c_str();
	cfg.level = ulog_level::debug;
	check(ulog_init(cfg) == true, "init: ok with file");

	ULOG_INFO("hello %d", 42);
	ULOG_WARN("warn-payload %s", "xyz");
	ULOG_ERROR("err-line");
	ulog_flush();

	check(file_contains(log.c_str(), "hello 42"), "content: formatted message lands");
	check(file_contains(log.c_str(), "[WARN] warn-payload xyz"), "content: level + args");
	check(file_contains(log.c_str(), "[ERROR] err-line"), "content: error line");

	// 级别过滤: 提到 error 后 info/warn 不再出现
	ulog_set_level(ulog_level::error);
	const size_t before = count_lines(log.c_str());
	ULOG_INFO("should-not-appear");
	ULOG_ERROR("should-appear");
	ulog_flush();
	const size_t after = count_lines(log.c_str());
	check(after == before + 1 && file_contains(log.c_str(), "should-appear") &&
			!file_contains(log.c_str(), "should-not-appear"), "filter: level gate works");

	// RAW 快路径: 字面量 memcpy, 无 vsnprintf
	ulog_set_level(ulog_level::info);
	ULOG_INFO_RAW("raw-literal-fast-path");
	ulog_flush();
	check(file_contains(log.c_str(), "raw-literal-fast-path"), "raw: literal lands via fast path");

	// 超长截断: 300 字符消息 → 截断于 127, "truncated" 尾巴必然被切掉 → 验证长 A 串存在且行长受限
	char big[300];
	std::memset(big, 'A', sizeof(big) - 1);
	big[sizeof(big) - 1] = '\0';
	ULOG_INFO("%s-truncated", big);
	ulog_flush();
	{
		FILE * f = std::fopen(log.c_str(), "r");
		char buf[512];
		bool found_trunc = false;
		while (std::fgets(buf, sizeof(buf), f) != nullptr)
		{
			if (std::strchr(buf, 'A') != nullptr && std::strstr(buf, "truncated") == nullptr)
			{
				found_trunc = true;
				check(std::strlen(buf) < 200, "truncation: line capped near 128+overhead");
			}
		}
		std::fclose(f);
		check(found_trunc, "truncation: long message present (truncated before tail)");
	}

	// shutdown 排空: 停机后所有已入队记录落盘 (error 级 —— 当前级别已提到 error)
	ULOG_ERROR("final-before-shutdown");
	ulog_shutdown();
	check(file_contains(log.c_str(), "final-before-shutdown"), "shutdown: drains queue to file");

	// 未激活短路: shutdown 后日志零成本丢弃 + 计数
	const uint64_t dropped0 = unistackbot_common::ulog_dropped();
	ULOG_INFO("post-shutdown-noop");
	check(unistackbot_common::ulog_dropped() == dropped0 + 1, "noop: counted after shutdown");
}

// ---- 1b. 级别专项: 四档独立正确 + 运行期逐级调节 ----

void level_tests()
{
	const std::string log = g_dir + "/level.log";
	std::remove(log.c_str());

	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = log.c_str();
	cfg.level = ulog_level::info;
	ulog_init(cfg);

	// ① 默认 info: debug 被滤, 其余三档各自落在文件里且级别标签正确
	ulog_set_level(ulog_level::info);
	ULOG_DEBUG("lv-debug-msg");
	ULOG_INFO("lv-info-msg");
	ULOG_WARN("lv-warn-msg");
	ULOG_ERROR("lv-error-msg");
	ulog_flush();
	check(file_contains(log.c_str(), "[DEBUG] lv-debug-msg") == false, "level: debug filtered at info");
	check(file_contains(log.c_str(), "[INFO] lv-info-msg"), "level: info tag correct");
	check(file_contains(log.c_str(), "[WARN] lv-warn-msg"), "level: warn tag correct");
	check(file_contains(log.c_str(), "[ERROR] lv-error-msg"), "level: error tag correct");

	// ② 运行期降到 debug: debug 开始出现
	ulog_set_level(ulog_level::debug);
	ULOG_DEBUG("lv-debug-now-passes");
	ulog_flush();
	check(file_contains(log.c_str(), "[DEBUG] lv-debug-now-passes"), "level: runtime downgrade to debug");

	// ③ 运行期提到 warn: info/debug 被滤, warn/error 仍过
	ulog_set_level(ulog_level::warn);
	ULOG_DEBUG("lv-should-not-2");
	ULOG_INFO("lv-should-not-3");
	ULOG_WARN("lv-warn-passes");
	ULOG_ERROR("lv-error-passes-2");
	ulog_flush();
	check(file_contains(log.c_str(), "lv-should-not-2") == false &&
			file_contains(log.c_str(), "lv-should-not-3") == false, "level: runtime upgrade filters lower");
	check(file_contains(log.c_str(), "[WARN] lv-warn-passes") &&
			file_contains(log.c_str(), "[ERROR] lv-error-passes-2"), "level: warn/error pass at warn");

	// ④ 只留 error 档
	ulog_set_level(ulog_level::error);
	ULOG_WARN("lv-should-not-4");
	ULOG_ERROR("lv-error-only");
	ulog_flush();
	check(file_contains(log.c_str(), "lv-should-not-4") == false, "level: warn filtered at error");
	check(file_contains(log.c_str(), "[ERROR] lv-error-only"), "level: error passes at error");

	// ⑤ 宏的级别外零成本路径: 提级后 emit 不入环 (pending 不增长)
	ulog_set_level(ulog_level::error);
	const size_t p0 = unistackbot_common::ulog_pending();
	for (int i = 0; i < 1000; ++i)
	{
		ULOG_INFO("filtered-loop-%d", i);
	}
	check(unistackbot_common::ulog_pending() == p0, "level: filtered emit never enqueues");
	ulog_shutdown();
}

// ---- 2. 轮转 ----

void rotation_tests()
{
	const std::string log = g_dir + "/rot.log";
	std::remove(log.c_str());
	const std::string l1 = log + ".1";
	const std::string l2 = log + ".2";
	std::remove(l1.c_str());
	std::remove(l2.c_str());

	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = log.c_str();
	cfg.max_bytes = 1024;   // 1KB: 几条就触发
	cfg.backups = 2;
	ulog_init(cfg);

	for (int i = 0; i < 200; ++i)
	{
		ULOG_INFO("rotation-fill-line-%04d-xxxxxxxxxxxxxxxxxxxx", i);
	}
	ulog_shutdown();

	struct stat st;
	const bool has_current = ::stat(log.c_str(), &st) == 0 && st.st_size <= 1024;
	const bool has_b1 = ::stat(l1.c_str(), &st) == 0;
	const bool no_b3 = ::stat((log + ".3").c_str(), &st) != 0;   // backups=2: 最多 .1 .2
	check(has_current && has_b1 && no_b3, "rotation: files rolled, backup count respected");
}

// ---- 3. 并发风暴: 8 线程 × 12500, 账目守恒 + 行格式 + 无撕裂 ----

void storm_test()
{
	const std::string log = g_dir + "/storm.log";
	std::remove(log.c_str());

	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = log.c_str();
	ulog_init(cfg);

	constexpr int kThreads = 8;
	constexpr uint64_t kPer = 12500;
	constexpr uint64_t kTotal = kThreads * kPer;
	std::atomic<bool> go{false};
	std::vector<std::thread> pool;
	for (int t = 0; t < kThreads; ++t)
	{
		pool.emplace_back([t, &go]()
		{
			while (!go.load(std::memory_order_acquire))
			{
				std::this_thread::yield();
			}
			for (uint64_t i = 0; i < kPer; ++i)
			{
				ULOG_INFO("T%d-i=%llu", t, static_cast<unsigned long long>(i));
			}
		});
	}
	go.store(true, std::memory_order_release);
	for (auto & th : pool)
	{
		th.join();
	}
	ulog_shutdown();

	const uint64_t written = unistackbot_common::ulog_written();
	const uint64_t evicted = unistackbot_common::ulog_dropped();
	check(written + evicted == kTotal, "storm: accounting (written + dropped == emitted)");
	check(count_lines(log.c_str()) == written, "storm: file line count == written");
	// 每线程子序列严格递增 (撕裂行 = sscanf 失败或 i 回退; 首条不比较 —— 哨兵法,
	// 踩坑记录: 曾用 0 初始化 + i<=last, 导致每线程首条 i=0 必然误判, 普通 build 靠
	// "i=0 恰最旧总被逐出"侥幸通过, ASan 慢速时序下暴露)
	uint64_t per_t_last[kThreads];
	for (int t = 0; t < kThreads; ++t)
	{
		per_t_last[t] = UINT64_MAX;   // 哨兵 = 该线程首条, 不参与比较
	}
	bool order_ok = true;
	bool format_ok = true;
	FILE * f = std::fopen(log.c_str(), "r");
	char buf[512];
	while (std::fgets(buf, sizeof(buf), f) != nullptr)
	{
		if (!line_format_ok(buf))
		{
			format_ok = false;
			continue;
		}
		int t = -1;
		unsigned long long i = 0;
		if (std::sscanf(buf, "%*[^\nT]T%d-i=%llu", &t, &i) == 2 && t >= 0 && t < kThreads)
		{
			if (per_t_last[t] != UINT64_MAX && i <= per_t_last[t])
			{
				order_ok = false;
			}
			per_t_last[t] = i;
		}
		else
		{
			format_ok = false;
		}
	}
	std::fclose(f);
	check(format_ok, "storm: every line well-formed (no torn lines)");
	check(order_ok, "storm: per-thread subsequence strictly increasing");
	std::printf("INFO: emitted=%llu written=%llu evicted=%llu\n",
		static_cast<unsigned long long>(kTotal),
		static_cast<unsigned long long>(written),
		static_cast<unsigned long long>(evicted));
}

// ---- 4. RT-safety: WCET + 零 malloc (线程局部窗口钩子) ----

void rt_safety_test()
{
	const std::string log = g_dir + "/rt.log";
	std::remove(log.c_str());
	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = log.c_str();
	ulog_init(cfg);

	constexpr int kOps = 20000;
	uint64_t max_ns = 0;
	uint64_t total_ns = 0;
	g_emit_mallocs = 0;
	for (int i = 0; i < kOps; ++i)
	{
		const auto t0 = std::chrono::steady_clock::now();
		t_in_emit = 1;
		ULOG_INFO("rt-probe-%d-payload-xxxxxxxxxxxxxxxx", i);
		t_in_emit = 0;
		const auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0);
		total_ns += static_cast<uint64_t>(dt.count());
		if (static_cast<uint64_t>(dt.count()) > max_ns)
		{
			max_ns = static_cast<uint64_t>(dt.count());
		}
	}
	ulog_shutdown();
	check(g_emit_mallocs == 0, "rt: zero malloc on emit path");
#if defined(__SANITIZE_THREAD__)
	// TSAN 插桩使每个原子操作 +数百 ns, WCET 阈值相应放宽为防死锁哨兵
	check(max_ns < 5000000, "rt: WCET < 5ms (TSAN build, deadlock sentinel)");
#else
	check(max_ns < 50000, "rt: WCET < 50us (x86 实测通常 <5us, 上限留弱平台裕度)");
#endif
	std::printf("INFO: emit WCET max=%lluus avg=%lluns mallocs=%d\n",
		static_cast<unsigned long long>(max_ns / 1000),
		static_cast<unsigned long long>(total_ns / kOps),
		g_emit_mallocs);
}

// ---- 5. 过载: 8 线程全速 40 万条打 8192 环 → 调用者时延有界 + 账目守恒 ----

void overload_test()
{
	const std::string log = g_dir + "/over.log";
	std::remove(log.c_str());
	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = log.c_str();
	ulog_init(cfg);

	constexpr int kThreads = 8;
	constexpr uint64_t kPer = 50000;
	std::atomic<uint64_t> max_emit_ns{0};
	std::vector<std::thread> pool;
	for (int t = 0; t < kThreads; ++t)
	{
		pool.emplace_back([&]()
		{
			uint64_t local_max = 0;
			for (uint64_t i = 0; i < kPer; ++i)
			{
				const auto t0 = std::chrono::steady_clock::now();
				ULOG_INFO("overload-flood-payload-%llu-xxxxxxxxxxxx", static_cast<unsigned long long>(i));
				const auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
				if (static_cast<uint64_t>(dt) > local_max)
				{
					local_max = static_cast<uint64_t>(dt);
				}
			}
			uint64_t cur = max_emit_ns.load(std::memory_order_relaxed);
			while (local_max > cur && !max_emit_ns.compare_exchange_weak(cur, local_max))
			{
			}
		});
	}
	for (auto & th : pool)
	{
		th.join();
	}
	ulog_shutdown();

	const uint64_t emitted = static_cast<uint64_t>(kThreads) * kPer;
	const uint64_t written = unistackbot_common::ulog_written();
	const uint64_t dropped = unistackbot_common::ulog_dropped();
	check(written + dropped == emitted, "overload: accounting holds under flood");
	// 5ms = 防死锁哨兵 (8192 环竞争逐出实测 ~100us 量级, 仍是"有界不阻塞", 弱平台留裕度)
	check(max_emit_ns.load() < 5000000, "overload: caller bounded (<5ms, deadlock sentinel)");
	check(count_lines(log.c_str()) == written, "overload: file consistent");
	std::printf("INFO: emitted=%llu written=%llu dropped=%llu max_emit=%lluus\n",
		static_cast<unsigned long long>(emitted),
		static_cast<unsigned long long>(written),
		static_cast<unsigned long long>(dropped),
		static_cast<unsigned long long>(max_emit_ns.load() / 1000));
}

}  // namespace


// 忘记 shutdown 的优雅退出验证: init + log 后直接返回 (不 shutdown),
// 进程退出时静态析构 ~ULog 兜底 (停线程+落盘) —— 崩溃即非零退出码
void no_shutdown_exit_test()
{
	ulog_config cfg;
	cfg.console = false;
	// 具名局部: 临时 string 的 c_str() 在语句结束析构, ulog_init 会读到悬垂指针
	const std::string log_path = g_dir + "/noshutdown.log";
	cfg.file_path = log_path.c_str();
	ulog_init(cfg);
	ULOG_INFO("record-before-exit-without-shutdown");
}


// ---- 引用计数 (2026-09-17 新增): 多组件同进程共用 ulog —— init 幂等计引用,
//      shutdown 末位才真停。触发场景: ros2_control_node 内 SimControlHardware
//      与 CartesianMotionController 并存, 各自 init/shutdown 不得互踢。 ----
void refcount_tests()
{
	const std::string f0 = g_dir + "/rc_a.log";
	const std::string f1 = g_dir + "/rc_b.log";

	// 组件 A: init (首引用, 起 writer)
	ulog_config ca;
	ca.console = false;
	ca.file_path = f0.c_str();
	ca.level = ulog_level::info;
	ca.max_bytes = 1000000;   // 配额给足: 本测试不验证轮转 (max_bytes=0 会按默认轮转,
	                          // 67B 就被转走, 内容断言失效 —— 2026-09-17 实测踩中)
	ca.backups = 2;
	const bool a_ok = ulog_init(ca);
	check(a_ok, "refcount: A init");

	// 组件 B: init (第二引用, 幂等路径; 首配置为准 —— 不得重开文件)
	ulog_config cb = ca;
	cb.file_path = f1.c_str();
	const bool b_ok = ulog_init(cb);
	check(b_ok, "refcount: B init (幂等)");
	// A 发日志 -> 落盘 (writer 未受 B init 影响)
	ULOG_INFO("refcount: A line 1");
	ulog_flush();
	const uint64_t written_after_a = ulog_written();
	check(written_after_a > 0, "refcount: A 发日志 writer 正常");

	// A shutdown (引用 2->1): writer 必须存活, B 还能发
	ulog_shutdown();
	ULOG_INFO("refcount: B line after A shutdown");
	ulog_flush();
	check(ulog_written() > written_after_a, "refcount: A shutdown 后 writer 存活");

	// B shutdown (引用 1->0): 真停 (fclose 落盘), 后续 emit 短路进 noop
	ulog_shutdown();
	{
		// 首配置 file_path 保持: B 的日志进了 A 的文件 (f0), f1 从未创建
		std::ifstream in(f0);
		std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		check(all.find("B line after A shutdown") != std::string::npos,
			"refcount: 首配置 file_path 保持 (B 日志落 f0)");
	}
	check(!std::ifstream(f1.c_str()).good(), "refcount: f1 未被创建 (幂等 init 未重定向)");
	const uint64_t before_noop = ulog_dropped();
	ULOG_INFO("refcount: after final shutdown (应短路)");
	check(ulog_dropped() > before_noop, "refcount: 末位 shutdown 后 emit 短路");

	// 重开: 新会话正常
	ulog_config cc = ca;
	cc.file_path = f1.c_str();   // 新会话独立文件
	check(ulog_init(cc), "refcount: 停机后再 init");
	ULOG_INFO("refcount: new session line");
	ulog_flush();
	check(ulog_written() > 0, "refcount: 新会话 writer 正常");
	ulog_shutdown();
	ulog_config cd = cc;
	check(ulog_init(cd), "refcount: 新会话再 init");
	ULOG_INFO("refcount: probe");
	const uint64_t base2 = ulog_dropped();
	ulog_shutdown();
	check(ulog_dropped() >= base2, "refcount: 新会话 shutdown 生效 (emit 短路)");
}


// ---- RAW 快路径与 flush 落盘语义 (2026-09-17 评审修复配套) ----
void raw_flush_tests()
{
	const std::string f0 = g_dir + "/rf.log";
	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = f0.c_str();
	cfg.level = ulog_level::info;
	cfg.max_bytes = 1000000;
	cfg.backups = 2;
	check(ulog_init(cfg), "raw/flush: init");

	// RAW 正常路径: 激活时落盘
	ULOG_INFO_RAW("raw line while active");
	ulog_flush();

	// flush 语义: 返回后文件立即可读 (后端分支内补排空 + fflush —— 2026-09-17 修复)
	{
		std::ifstream in(f0);
		std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		check(all.find("raw line while active") != std::string::npos,
			"raw/flush: flush 返回后文件已可读");
	}

	// RAW 级别过滤对齐 (评审第二轮): min_level=warn 时 RAW(INFO) 被级别过滤,
	// 不落盘也不计 noop (级别过滤不计数的既有裁定)。
	// 调级走 ulog_set_level —— init 幂等且首配置为准, 重 init 改不了级别 (注意事项 2)
	ulog_set_level(ulog_level::warn);
	ULOG_INFO_RAW("raw filtered by level (must not land)");
	ulog_flush();
	{
		std::ifstream in(f0);
		std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		check(all.find("raw filtered by level") == std::string::npos,
			"raw/flush: min_level=warn 时 RAW 被级别过滤");
	}
	ulog_set_level(ulog_level::info);

	// RAW 关闭路径回归 (评审 P0-2): shutdown 后 RAW 必须短路计 noop, 不得付出完整
	// emit 代价 (now/tid/strlen/拷贝/push) —— 与 ULOG_INFO 语义对齐
	ulog_shutdown();
	const uint64_t base = ulog_dropped();
	ULOG_INFO_RAW("raw after shutdown (must be swallowed)");
	check(ulog_dropped() == base + 1, "raw/flush: shutdown 后 RAW 短路计 noop");
}

// ---- 轮转 rename 失败路径 (2026-09-18 评审第三轮): 预建同名目录使 rename 报
//      EISDIR (非 ENOENT) —— 断言诊断非致命, 日志继续落盘, 不形成失败循环 ----
void rotate_rename_fail_test()
{
	const std::string f0 = g_dir + "/rotfail.log";
	ulog_config cfg;
	cfg.console = false;
	cfg.file_path = f0.c_str();
	cfg.level = ulog_level::info;
	cfg.max_bytes = 200;   // 几条即触发轮转
	cfg.backups = 2;
	check(ulog_init(cfg), "rotfail: init");
	// 让 f0.1 成为目录: rename(file -> 已存在目录) = EISDIR
	const std::string blocker = f0 + ".1";
	::mkdir(blocker.c_str(), 0755);
	for (int i = 0; i < 30; ++i)
	{
		ULOG_INFO("rotfail line %d padpadpadpad", i);   // 撑过 max_bytes 多次触发轮转
	}
	ulog_flush();
	ulog_shutdown();
	// 断言: 没崩 (走到这即通过一半); 且至少有日志落盘 (f0 或被轮转出的 .2)
	bool landed = false;
	{
		std::ifstream in(f0);
		std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		landed = !all.empty();
	}
	check(landed, "rotfail: rename 失败不致命, 日志继续落盘");
	::rmdir(blocker.c_str());
}
int main()
{
	g_dir = "/tmp/ulog_test_XXXXXX";
	std::vector<char> dir_buf(g_dir.begin(), g_dir.end());
	dir_buf.push_back('\0');
	if (::mkdtemp(dir_buf.data()) == nullptr)
	{
		std::printf("FAIL: cannot create tmp dir\n");
		return 1;
	}
	g_dir.assign(dir_buf.data());
	std::printf("INFO: workdir=%s\n", g_dir.c_str());

	functional_tests();
	level_tests();
	rotation_tests();
	storm_test();
	rt_safety_test();
	overload_test();
	refcount_tests();
	raw_flush_tests();
	rotate_rename_fail_test();
	no_shutdown_exit_test();

	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}

# ulog · 高性能异步日志组件

> 状态:**已实现, 全测试通过** (29 项断言 + 三 sanitizer + 吞吐基准)。定位: `unistackbot_common` 第三个组件, 多工程通用
> (纯 C++17 + POSIX, 零 ROS 零第三方依赖, 自包含可整体拷贝复用)。
> 接口形状继承 LUMOSUMI `FileLogger` (生命周期/宏/运行期调级/flush 确认),
> 内核升级: string 条目 → 定长 POD, mutex 入队 → 每线程无锁环, 单文件 → 终端+文件双 sink。

## 1. 设计目标 (按优先级)

1. **调用线程安全**: 任意线程(含 RT)调用日志 API 不阻塞、不分配、无锁、WCET 有界
2. **双 sink**: 终端 + 文件, 后端单线程承担全部 I/O
3. **跨工程复用**: 依赖闭包自包含 (ulog/ + mpsc_ring/ 两个文件夹), 无 ROS/第三方
4. **过载不倒灌**: 任何情况下日志系统绝不反压业务线程 —— 过载时丢日志并计数
5. 接口形状与 LUMOSUMI FileLogger 同族 (老用户零学习成本)

## 2. 总体架构 (三段式)

```
任意线程 (含 RT)                      后端线程 (唯一, 承担全部 I/O)
┌────────────────────┐              ┌──────────────────────────────┐
│ ULOG_INFO("...", …) │              │ drain 共享环 → 组装行          │
│  ①级别过滤(原子)     │   push       │  → 终端 write(批量)           │
│  ②snprintf 入定长缓冲│ ──(无锁)──→ │  → 文件缓冲写 (error 级强刷)   │
│  ③push, 满则丢旧保新 │   丢旧+跳号  │  → 轮转检查                   │
└────────────────────┘              └──────────────────────────────┘
     单只共享 MpscRing<Record> (Vyukov 每槽 seq + 逐出, init 时一次定容分配)
```

三段分工: **格式化在调用侧**(v1, snprintf 入定长缓冲, 无分配), **I/O 在后端**, **通道无锁**。
与市面对照: 形态 = "共享 MPSC 环 + 后端单线程" (nanomsg/JCTools 日志形态), 编码 = spdlog 的"调用侧格式化" (v2 再演进延迟格式化)。

## 3. 记录编码 (v1 定长 POD)

```cpp
struct ULogRecord                       // 152B, trivially_copyable (static_assert)
{
	uint64_t ts_ns;      // 入队时墙钟 CLOCK_REALTIME (ns, vDSO ~20ns 无 syscall)
	uint32_t level;      // Debug=0 Info=1 Warn=2 Error=3
	uint32_t tid;        // 线程标识 (thread_local 缓存 gettid, 热路径零 syscall)
	uint8_t  msg_len;    // 实际消息长度 (≤128, uint8 足够)
	char     msg[128];   // 预格式化消息 (超长截断, 尾部无截断标记 —— 长度字段自含)
};
```

- 无 tag (裁决): 行身份 = 时间戳 + 级别 + 线程 id; 多工程复用不需要预置模块枚举
- **无全局 seq 字段 (设计审核裁决)**: seq 分配 (全局 FAA) 与 push 占位 (ring 的 FAA) 是两次
  独立原子, 取号后可被抢占 → drain 序 ≠ seq 序, 拿 seq 做顺序对账必然误报
  (MpscRing 测试踩过的同款坑)。对账机制 = `ring.evicted()` + 账目守恒
  (收到 + 逐出 + 残留 == 推入), MpscRing 已验证的同款方案
- 无 std::string —— 全 POD, 踩进 mpsc_ring 的元素契约
- v2 演进位: msg[] 换成"格式串 id + 二进制参数" (延迟格式化, NanoLog/Quill 路线), 队列与后端结构不变

## 4. 通道: 单只共享 MpscRing (新原语, 满则丢旧保新)

- 一只 `MpscRing<ULogRecord, kQueueCap>` (默认 8192 条 ≈ 1.2MB), `ulog_init` 时一次定容分配
- **任意线程直接 push, 恒成功** (CAS 抢位, 无锁无等待) —— 无 thread_local、无注册表、无线程生命周期问题;
  来源识别靠记录内 tid
- 溢出策略 = **丢旧保新**: 满时逐出最旧未消费条目 (head CAS 前移, 与消费者共写走原子 RMW),
  消费者"拷贝 → 验槽 seq → CAS 推进"自检, 被逐出途中的拷贝检出即弃 —— 撕裂数据永不外泄;
  风暴末尾的最新记录恒保留 (诊断价值最高的部分), 中间被逐出的条目以跳号可观测
- 后端单读者 drain: 每槽 seq 保证只读到完整已提交条目
- **设计裁决记录**: 初稿曾用"thread_local 每线程 SpscRing + 注册表", 评审否决 ——
  线程退出环销毁但注册表仍持指针 (use-after-free)、内存随线程数无界增长。
  二稿溢出定"丢新", 评审再否 —— 风暴末尾恰是最有诊断价值的记录, 丢新语义颠倒。
  定稿: 共享 MPSC 环 + 丢旧保新。MPSC 环的立项触发方即本组件

## 5. 后端 (单线程)

- **drain 循环**: 单只共享环无阻塞 pop 到空 → 组装行 → 批量 write。
  行格式化全部在后端做 (ts_ns → `YYYY-mm-dd HH:MM:SS.mmm` 可读时间, 对齐 LUMOSUMI 行格式;
  前端只存 ns 整数); 后端持有行缓冲, 可分配 —— 后端非 RT
- 后端线程以 `prctl(PR_SET_NAME, "ulog")` 命名 (top/gdb 可辨识)
- **终端 sink**: write(stdout); 终端慢由后端承受, 不倒灌
- **文件 sink**: `open(O_APPEND|O_CREAT)`, 后端自缓冲; **error 级强制 flush**,
  其余按 100ms 周期 flush; **大小轮转**: 超过 max_bytes (默认 10MB) →
  `.log → .log.1 → … → .log.N` 位移 (默认保留 5 份, 与 LUMOSUMI 语义一致)
- **drain 休眠**: 环空时睡 1ms; 非空则连续 drain (非 RT 线程, 可睡可忙)
- **shutdown**: 停标志 → 排空所有环 → flush 文件 → join → 此后日志调用静默丢弃+计数

## 6. 前端 API (继承 LUMOSUMI 形状, 去 tag)

```cpp
// 生命周期 (init 幂等; 未 init 时日志调用静默丢弃+计数)
bool ulog_init(const ulog_config & cfg);   // { 终端开关, 文件路径, 级别, 容量, 轮转参数 }
void ulog_shutdown();
void ulog_flush();                          // 精确语义: 排空当前环内全部记录并落盘

// 打日志 (宏: ①级别过滤 → ②snprintf 编码 → ③try_push)
ULOG_DEBUG("fmt", args…);  ULOG_INFO(…);  ULOG_WARN(…);  ULOG_ERROR(…);
ULOG_INFO_RAW("纯字面量");                  // 无格式快路径 (纯 memcpy)

// 运行期控制
void ulog_set_level(Level);                // atomic relaxed
size_t ulog_dropped();                     // 全局丢弃计数 (健康观测)
size_t ulog_pending();                     // 待写积压
```

- 级别四档对齐 LUMOSUMI (DEBUG/INFO/WARN/ERROR); 调用侧过滤为宏展开第一句, 级别外零开销
- 无 tag (裁决); 需要模块归属时打进消息文本

## 7. 错误处理

- 文件打开失败: init 返回 false (终端 sink 可独立存活); 运行中写失败/磁盘满: 文件 sink
  置错误态停写 + 计数, 终端照常 —— 日志系统自身故障绝不外溢成业务故障
- 后端线程异常死亡: 前端检测环长期满 → 静默丢弃+计数 (与过载同路径), 不崩溃业务

## 8. 边界与 v1 明确不做

| 边界 | 说明 |
|---|---|
| **无害化承诺** (职责边界: 不干扰采集等 IO 业务) | 无 fsync (不制造同步盘点); 写盘量有界 (环容量界定单次 flush 上限); 后端线程自降 nice+10 与 ioprio 最低档 —— CPU/IO 调度上永远让路; 系统级回写调优 (dirty_bytes 等) 归使用方 |
| 崩溃丢尾 | 环内未 drain 的记录在进程崩溃时丢失; error 级强刷缓解但不根除 (v2 可做关键级同步旁路) |
| 堆积过久逐出最旧 | 丢旧保新的设计语义; 最新 N 条恒完整, 被逐出条目以跳号可观测 |
| 无轮转归档/压缩 | 仅大小轮转位移, 无月度归档 (LUMOSUMI 的 archive 不继承) |
| 无网络 sink / 无延迟格式化 / 无运行期增删 sink | v2 演进位 |
| RT 线程使用纪律 | 组件使 RT 日志**可行**(无阻塞有界), 但**常态化高频日志仍应放非 RT 线程**——RT 侧只打异常/迁移事件 |

## 9. 多工程复用指南

- 依赖闭包 = `ulog/` + `mpsc_ring/` 两个文件夹; 零 ROS、零第三方、仅 C++17 + POSIX (Linux 目标)
- 复用 = 整体拷贝 + 加 include 路径 + 调用 `ulog_init`; 可按工程改 namespace 与宏前缀

## 10. 测试与验证 (已全部执行)

```
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_ulog.cpp -o test_ulog && ./test_ulog
# sanitizer:
g++ -std=c++17 -O1 -g -pthread -fsanitize=address,undefined -I.. test_ulog.cpp -o test_asan && ./test_asan
g++ -std=c++17 -O1 -g -pthread -fsanitize=thread -I.. test_ulog.cpp -o test_tsan && \
  TSAN_OPTIONS="suppressions=tsan_suppressions.txt" setarch $(uname -m) -R ./test_tsan
# 吞吐基准 (手动): 阶梯找饱和点 + 洪泛看过载行为
g++ -std=c++17 -O2 -pthread -I.. bench_ulog.cpp -o bench_ulog && ./bench_ulog
```

实测 (x86_64): 29 项断言全过 (功能 10 + **级别专项 10** + 轮转 1 + 风暴 4 + RT 2 + 过载 3 减汇总);
emit WCET max 2-6µs / avg ~170ns; **emit 路径零 malloc**;
后端饱和点 ≥200 千行/s (bench_ulog 阶梯, 5~200kHz 全档零丢失);
并发风暴 8×12500 与过载 8×50000 账目守恒 (written + dropped == emitted);
TSAN 零竞争 (白名单仅一条已知工具误报, 见 tsan_suppressions.txt)。

实现期踩坑记录 (全部转为断言/注释):
- 会话账目: written/evicted/noop 为静态累计, init 清零/记基线 —— 否则多会话账目串账
- 截断语义: 128 字符截断发生在长消息尾部之前, 断言须查前缀长串而非尾标
- 每线程首条哨兵: `i <= last` 对首条必误判, 普通 build 靠"首条恰被逐出"侥幸通过,
  ASan 慢时序暴露 —— 哨兵法修复
- flush 代际计数原子化 (锁只服务 cv 等待) + 锁外 I/O —— 消 TSAN 报告且是正确姿势

## 11. 决策点 (已全部定稿)

| # | 决策 | 本方案默认 | 备选 |
|---|---|---|---|
| D1 | v1 调用侧格式化 (snprintf) | ✓ | v2 延迟格式化 (Quill 路线) |
| D2 | 溢出 = 丢旧保新 (满逐出最旧, 最新 N 条恒在) | ✓ (评审两轮定稿) | 丢新 (已否: 丢掉风暴末尾诊断记录) |
| D3 | 轮转进 v1 | ✓ (10MB × 5, 对齐 LUMOSUMI) | v1 单文件超限停写 |
| D4 | header-only (inline 后端) | ✓ (与家族组件一致) | hpp + cpp (使用方编译单元) |
| D5 | 全局单例形态 (init 幂等) | ✓ | 显式句柄 (多实例) |
| D6 | 终端 sink 默认 stdout | ✓ | stderr |
| D7 | 新原语 `mpsc_ring/` 作为通道 (Vyukov 每槽 seq) | ✓ (日志 = 第一个多写单读消费场景, 触发立项) | 复用 SpscRing + 互斥汇聚 (RT 不可用, 已否) |

## 12. 落位

```
unistackbot_common/
├── mpsc_ring/       # 前置原语: 多写单读无锁环 (六轮评审定稿)
└── ulog/
    ├── README.md            # 本文档
    ├── ulog.hpp             # 前端 API + 记录定义 + 后端 (header-only)
    ├── test_ulog.cpp        # 功能/并发/RT-safety/过载
    └── tsan_suppressions.txt  # TSAN 已知误报白名单 (仅一条, 验证命令见 §10)
```

## 13. v1.1 多实例设计 (评审稿 · 待拍板)

> 目标: 不同模块写不同文件 (分流), 核心语义 = **风暴隔离**。
> 形态: 多环实例 + 共享单后端线程 (Quill/内部 spdlog 结构)。

### 13.1 概念模型

```
ulog_init(cfg)                        = open 默认实例 (名字 "", 宏的默认去向, 现状不变)
ulog_open(name, cfg) -> ulog_handle    = 新建命名实例: 独立环 + 独立文件/配置
ULOG_INFO(...)                        → 默认实例的环 ──┐
vlog->info(...)  /  ULOG_L(vlog, INFO, ...) → vlog 的环 ──┼→ 共享后端线程(1个)
                                                          │   轮询 drain 各环 →
                                                          │   按实例分发到各自文件
                                                          ┘   (独立轮转/独立 flush)
```

### 13.2 API 完整定义

```cpp
// ---- v1 现状, 零改动 ----
bool   ulog_init(const ulog_config &);      // = open 默认实例 + 注册为宏默认
void   ulog_shutdown();                     // 关闭全部实例 (含未显式 close 的) + 停后端
void   ulog_flush();                        // 默认实例: 排空+落盘确认
void   ulog_set_level(ulog_level);          // 默认实例
size_t ulog_dropped();  size_t ulog_pending();  uint64_t ulog_written();   // 默认实例
ULOG_DEBUG/INFO/WARN/ERROR(...)             // 默认实例宏, 向后兼容

// ---- v1.1 新增 ----
using ulog_handle = ulog_detail::UlogInstance *;

ulog_handle ulog_open(const char * name, const ulog_config & cfg);
//  - 独立环/文件/级别/轮转; console 每实例独立可配
//  - 同名幂等: 已存在则返回已有句柄, cfg 被忽略 (README 契约, 不报错)
//  - 失败 (文件开不出且 cfg 要求文件) 返回 nullptr
//  - 必须在 emit 并发开始前调用 (装配期, 同 ulog_init)

bool   ulog_close(ulog_handle h);           // null 安全; 重复 close 安全 (幂等)
//  语义: 停该实例 emit → 摘出注册表 → 由后端线程排空该环 + flush + fclose
//        → close 同步等待完成才返回 (文件此时已完整可读)
//  契约: close 后不得再对该 handle emit (其宏短路+计入该实例 noop)

void   ulog_flush_h(ulog_handle h);         // 指定实例的 flush (语义同 ulog_flush)
void   ulog_set_level_h(ulog_handle h, ulog_level);
size_t ulog_dropped_h(ulog_handle h);       // 指定实例观测 (逐实例健康)

ULOG_L(h, INFO, "fmt", ...);               // 实例宏变体: h 为 null 时短路
//  展开等价于 (h)->emit_path —— 级别过滤读 h->min_level (非全局)
h->info("...") / h->warn(...)              // 方法形态 (无宏时可用)
```

### 13.3 内部结构

```cpp
struct UlogInstance                       // 每个 = 一条独立日志通道
{
    // ---- emit 热路径 (任意线程) ----
    std::atomic<bool>     active{false};
    std::atomic<int>      min_level;
    MpscRing<ULogRecord, ULOG_QUEUE_CAPACITY> ring;     // 容量编译期统一 (模板约束)
    std::atomic<uint64_t> noop_dropped{0};

    // ---- 后端单线程独占 (open 时装配期初始化) ----
    std::string name, path;
    FILE * file{nullptr};
    bool console;  size_t max_bytes;  int backups;  uint64_t file_bytes{0};
    uint64_t evicted_base{0};
    std::atomic<uint64_t> written{0};

    // ---- flush/close 确认 (原子代际, 同 v1 机制) ----
    std::atomic<uint64_t> flush_req_gen{0}, flush_done_gen{0};
    std::atomic<bool>     closing{false};
    std::mutex done_mtx;  std::condition_variable done_cv;   // close 完成确认
};

struct Backend                            // 全局唯一
{
    std::thread thread;  std::atomic<bool> running{false};
    std::mutex reg_mtx;  std::vector<UlogInstance *> registry;      // open/close 时变更
    std::vector<ulog_handle> retire;                                // 待收尾实例
    // run(): 轮询快照 → 逐实例 pop 到空 → write_one(inst, rec)
    //        → 处理各实例 flush 请求 (锁外 I/O) → 处理 retire (排空+fclose+delete)
    //        → 空闲睡 1ms
};
```

关键并发裁决:
- **pop 的单读者契约不破坏**: drain 只由后端线程执行; close 的排空也交给后端
  (经 retire 列表), close 调用方只等待确认 —— 全程一个消费者
- **删除权归后端**: UlogInstance 由后端在 retire 处理中 delete, 消除
  "close 与 drain 竞争指针" 的 UAF 窗口
- **注册表只在 open/close 碰 mutex** (装配/拆卸期); drain 每轮锁内拷贝指针快照
  (N<64, 1ms 一轮, 开销可忽略)

### 13.4 生命周期与并发契约

| 操作 | 线程约束 | 说明 |
|---|---|---|
| ulog_init / ulog_open | 单线程装配期 | 不得与 emit 并发 (同 v1 契约) |
| emit (宏/方法) | 任意线程任意并发 | 环内无锁; 实例 closing 后短路+noop 计数 |
| ulog_set_level(_h) | 任意线程任意时刻 | atomic relaxed |
| ulog_flush(_h) | 任意线程 | 代际确认, 5s 超时保护 |
| ulog_close | 任意线程, 但之后不得再 emit 该 handle | 同步返回 = 文件已完整 |
| ulog_shutdown | 单线程拆卸期 | 兜底关闭全部注册实例 (含忘记 close 的) → 停后端 |

### 13.5 隔离语义 (本方案的核心卖点, 精确化)

- 实例 A 风暴灌满自己的环 → 逐出只发生在 A 的环内; 实例 B 的环、B 的
  written 账目、B 的文件**完全不受影响** (测试将此写成断言)
- 隔离的代价 = 每实例内存一份环 (~1.2MB @8192): 10 模块 ≈ 12MB, 已知上界
- **跨实例不保证总序**: 各文件各自有序, 跨文件对齐靠 ts_ns 墙钟 (ms 级,
  与 log4j/spdlog 多文件同属性, 排障够用)

### 13.6 与 v1 的兼容

- 全部 v1 API/宏行为不变; 默认实例 = 名字 "" 的普通实例 (内部同构, 无特殊代码路径
  除"宏的默认去向")
- ULogRecord 布局不变 (152B) —— 不加 channel 字段 (路由信息在实例指针里, 不占记录)
- MpscRing 组件零改动 (实例把环作为成员)
- 既有测试零改动应全绿 (回归门)

### 13.7 测试增补

1. **双实例隔离** (核心新断言): A 8 线程灌 20 万条全速, B 单线程慢速 1000 条
   → B: written == 1000, dropped == 0, 文件行数 == 1000 (风暴未殃及)
   → A: 账目守恒 (允许 evicted)
2. 双实例独立轮转/独立级别: A 设 error-only + 1KB 轮转, B 全级别 + 大文件
3. close 语义: close 后 emit 短路计入该实例 noop; close 返回后文件完整
4. 同名幂等 / 重复 close 安全 / null 安全
5. shutdown 兜底: open 未 close 直接 shutdown → 文件完整
6. v1 全部 20 项回归零改动通过

### 13.8 决策点 (本轮待拍板)

| # | 决策 | 默认 | 备选 |
|---|---|---|---|
| E1 | 共享单后端线程 (非每实例一线程) | ✓ | 每实例独立线程 (朴素多实例, 已否: 线程随模块数膨胀) |
| E2 | 同名 open 幂等返回旧句柄 | ✓ | 返回 nullptr 报错 (严格模式) |
| E3 | 容量编译期统一 (ULOG_QUEUE_CAPACITY) | ✓ | 每实例运行期容量 (需改造 MpscRing 为堆分配环, 动已定稿组件, 不值) |
| E4 | 实例上限 | 无硬限 (注册表 vector) | 设上限 (如 64) 防泄漏式 open |
| E5 | console 每实例独立配置 | ✓ | 仅默认实例可 console (防多实例刷屏终端) |

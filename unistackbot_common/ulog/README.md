# ulog · 高性能异步日志组件

> Header-only 单例异步日志: 前端宏 (级别过滤 → snprintf 定长 POD → 无锁入队) + 后端单线程双 sink (终端 + 文件)。
> emit 零 malloc、WCET ~µs、过载丢旧保新不倒灌。纯 C++17 + POSIX, 零 ROS 零第三方, 可整体拷贝复用。

## 快速上手

```cpp
#include "ulog/ulog.hpp"

unistackbot_common::ulog_config cfg;
cfg.console    = true;                      // 终端 sink
cfg.file_path  = "/tmp/app.ulog";           // 文件 sink (不需要则 nullptr)
cfg.level      = unistackbot_common::ulog_level::info;
cfg.max_bytes  = 10 * 1024 * 1024;          // 必须 > 0 (见注意事项 3)
cfg.backups    = 5;                         // 轮转保留份数
unistackbot_common::ulog_init(cfg);

ULOG_INFO("opened %s", name);               // ULOG_DEBUG/INFO/WARN/ERROR (printf 风格)
ULOG_INFO_RAW("纯字面量快路径");              // 无格式化

unistackbot_common::ulog_shutdown();
```

## API

| API | 语义 |
|---|---|
| `ulog_init(cfg)` | 幂等 + 计引用: 首调用起 writer, 后续只计引用 (首配置为准) |
| `ulog_shutdown()` | 退引用, **末位才真停** (排空+落盘+join); 之后 emit 短路计入 dropped |
| `ulog_flush()` | 排空环 + fflush 确认 (5s 超时保护) |
| `ulog_set_level(lvl)` | 运行期调级, 任意线程 |
| `ulog_dropped() / ulog_written() / ulog_pending()` | 健康观测 (会话级计数, init 清零) |
| `ULOG_DEBUG/INFO/WARN/ERROR / _RAW` | 任意线程含 RT; 未 init 静默丢弃 + 计数; _RAW = INFO 快路径, 同受级别过滤 |

## 使用注意事项 (必读, 每条有实测出处)

1. **同进程多组件共用**: init 幂等计引用, shutdown 末位才真停 —— 各组件独立调用互不干扰
   (实证: ros2_control_node 内 SimControlHardware + CartesianMotionController 并存)。
2. **首配置为准**: writer 运行中再 init, 新 cfg 全部忽略。同进程第二组件要不同配置 = 不支持。
3. **`max_bytes` 必须 > 0**: 判据 `file_bytes >= max_bytes`, 传 0 = 每条记录都轮转。
   "不轮转"无开关, 传极大值。
4. **漏调 shutdown 不崩** (静态析构兜底), 但反复 init/shutdown 的组件必须严格配对,
   少一次 = 引用泄漏 = writer 永不停机。
5. **账目计数会话级**: init 清零基线, 跨会话比较 dropped/written 无意义。
6. **读日志文件做校验请在 shutdown 后**: flush 有 fflush, 但 stdio 缓冲窗口内读到旧内容。
7. **RT 线程**: emit 无阻塞有界, 可用; 但常态化高频日志放非 RT 线程, RT 侧只打异常事件。
8. **多 DSO 共用** (评审 P0 + dlopen 实验): 单例入口已加 `ULOG_EXPORT` (default 可见性
   显式导出) —— 默认与 `-fvisibility=hidden` 配置下单实例 (均实验验证)。
   **仍禁止 `-Wl,-Bsymbolic`** (自绑定破坏合并, 无编译期防护); 所有 .so 必须**同一份
   `ULOG_QUEUE_CAPACITY` 配置编译** (不同容量 = 环类型不同 = ODR 违规 UB);
   搬 .cpp 的彻底方案挂 D4 演进位。
9. **文件 sink 故障语义 = fail-fast**: 轮转后重开失败 → stderr 诊断一次, 文件停写
   (终端 sink 照常), 不自动重试、不形成失败循环; rename 失败 (权限/磁盘, ENOENT 除外)
   直写 stderr。磁盘故障是运维事件, 恢复靠重启进程, 不靠日志组件自愈。

## 测试

```bash
cd unistackbot_common/ulog
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_ulog.cpp -o test_ulog && ./test_ulog
# TSAN (白名单仅一条已知误报, 出现其他报告视为失败):
g++ -std=c++17 -O1 -g -pthread -fsanitize=thread -I.. test_ulog.cpp -o test_tsan && \
  TSAN_OPTIONS="suppressions=tsan_suppressions.txt" setarch $(uname -m) -R ./test_tsan
```

实测: 48 项断言全过 (功能/级别/轮转/风暴/过载/引用计数/RAW+flush/轮转故障路径); emit 零 malloc, WCET max ~µs;
饱和 ≥200 千行/s; TSAN 零新增竞争。

## 复用

依赖闭包 = `ulog/` + `mpsc_ring/` 两个文件夹, 整体拷贝即可。

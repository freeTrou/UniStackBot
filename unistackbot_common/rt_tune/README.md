# rt_tune — RT 线程调优参数化应用

单头文件组件 (`rt_tune.hpp`): 供控制器在**自己的 RT 线程内**调用——亲和性与调度策略只能作用于调用线程自身, 不能跨线程设置。

```cpp
#include "rt_tune.hpp"
// 线程入口处:
unistackbot_common::rt_tune::apply(cpu, fifo_prio, nice_val, name);
```

| 参数 | 语义 |
|---|---|
| `cpu` | -1 = 不绑定; ≥0 = 绑定指定核 |
| `fifo_prio` | 0 = 不动调度策略; >0 = SCHED_FIFO (需 root/CAP_SYS_NICE, 失败降级 SCHED_OTHER 并 WARN) |
| `nice_val` | SCHED_OTHER 下的 nice (仅 fifo_prio=0 时应用) |
| `name` | 线程名 (prctl, top/gdb 可见; nullptr = 不设) |

返回值: 0 = 全部成功; 按位错误码 (`kAffinityFailed`/`kFifoFailed`/`kNiceFailed`)。失败不抛不崩——调优是性能优化, 不是正确性前提。

**分工边界**: 控制器管理器主线程的调优经预留 yaml 参数接口由我们设置 (`thread_priority`/`cpu_affinity`/`lock_memory` —— 参数名是 ros2_control 暴露的接口, 值是我们定的); `rt_tune` 只管自建线程 (如 CM worker、总线主站线程)。

## 测试/基准: bench_rt_tune.cpp

双线程钉核1/核2 (FIFO80) 周期唤醒, 自统计唤醒延迟 (min/avg/p50/p99/max + 超 10/50/100µs 计数); 断言 `rt_tune::apply` 绑核+FIFO 双成功, 失败退出码 1。

```bash
cd unistackbot_common/rt_tune
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic bench_rt_tune.cpp -o /tmp/bench_rt_tune -pthread
/tmp/bench_rt_tune <period_us> <duration_s> [prio=80]   # 如 1000 120
```

参考数据 (2026-09-20, **RT-clean 版**, lowlatency 内核 + isolcpus 全套 + 禁深睡 + performance, 无负载, 120s/档; 详见 `test/results/rt_baseline_isolcpus.md` E 节):

| 档位 | cpu1 p99/max | cpu2 p99/max | >50µs 拍数 |
|---|---|---|---|
| 500Hz | 10.6 / 53µs | 9.9 / 47µs | 1 / 0 |
| 1kHz | 10.0 / 78µs | 8.7 / 42µs | 1 / 0 |
| 2kHz | 9.8 / 19µs | 8.1 / 30µs | 0 / 0 |

RT 纪律修正记录 (2026-09-20): 初版在测量线程内 `push_back` (realloc 伪影) 且无栈/堆预热与 warmup——修正后 2kHz 档尾部从 ~180µs 降至 ~30µs, 48 万拍零次 >50µs, 此前部分"单发尾部"实为测量工具自身分配伪影。p99 不受影响 (伪影只污染尾部)。

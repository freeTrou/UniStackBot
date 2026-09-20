# 实时性基线（重启前，未优化内核）— 2026-09-20

> 对照组：用户将改低延迟内核 + 内核参数优化，重启后重跑同命令对比。
> 目标核：**1、2 号核**（1 = CM 的 update 线程，2 = 预留总线）。

## 机器与内核状态（优化前）

| 项 | 值 |
|---|---|
| 内核 | 6.8.0-138-generic（**CONFIG_PREEMPT_VOLUNTARY**，非 RT/非低延迟）|
| 启动参数 | 无 isolcpus / nohz_full / rcu_nocbs |
| sched_rt_runtime_us | 950000（默认）|
| timer_migration | 1（默认）|
| ulimit -l | unlimited（memlock 已放开）|
| 核心 | 28 逻辑核 |

## cyclictest 基线（核 1 + 核 2，同命令可复现）

命令：`cyclictest -m -p 80 -D 60s -h 100 -a 1,2 -t 2 -q`

| 线程（核）| Min | Avg | Max |
|---|---|---|---|
| 核 1 | 1 µs | 3 µs | **289 µs** |
| 核 2 | 1 µs | 2 µs | **130 µs** |

20s 短测参考：核2 出现过 3288 µs 尖峰（Histogram Overflow ×1 @cycle 50 附近——偶发，可能是后台任务抢核）。

## CM 链实测（参考，优化前）

| 指标 | 值 | 来源 |
|---|---|---|
| CM update WCET p50 | 1.2 µs | WCET 终报 |
| CM update WCET p99 | 4.8 µs | 同上 |
| CM update WCET max | 87.3 µs（含 JSB 噪声轮）/ 11 µs（干净轮）| 同上 |
| 预算 | 2000 µs（500Hz）| — |
| 缺页 | 隔离测量 12s Δmin_flt = 0 | flt_clean.sh |
| CM 线程 | FIFO 80 @ 核1（官方参数生效）| /proc 实测 |

## 重启后对比项（同命令重跑）

```
cyclictest -m -p 80 -D 60s -h 100 -a 1,2 -t 2 -q
```
预期优化方向：Max 从 ~300µs 级 → <50µs 级（低延迟内核 + isolcpus 后）；
Avg 变化不大（本就 2-3µs）；关注 Max 与直方图长尾消失。

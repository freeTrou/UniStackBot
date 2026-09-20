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

## 三档负载测试（同基线机, 2026-09-20 重启前补测）

负载发生: /tmp/cpu_load.py (N 个 python 自旋进程; 已入库 test/fault 床同目录风格)
28 逻辑核: 50% = 14 进程, 90% = 25 进程

| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |
|---|---|---|
| 无负载 (0/28核) | 1 / 4 / **193** | 1 / 3 / **77** |
| 50% (14/28核) | 1 / 3 / **3057** ⚠️ | 1 / 1 / **20** |
| 90% (25/28核) | 1 / 1 / **2062** ⚠️ | 1 / 3 / **962** ⚠️ |

实测发现 (纯记录, 不做归因预判):
- 核1: 无负载 Max 193µs → 50% 负载 Max 3057µs → 90% 负载 Max 2062µs
- FIFO80 下 Avg 全档稳定 1-4µs, Min 恒 1µs — **Max 长尾随负载显著放大**
- 50%/90% 档的 Max (2-3ms) 已达 500Hz 控制周期 (2ms) 的量级
- 注: 基线内核 = PREEMPT_VOLUNTARY; 重启后为**低延迟内核 (lowlatency, 非 PREEMPT_RT)**,
  参数优化 (isolcpus 等, 以 /proc/cmdline 实测为准)。对比时只比数据, 不带预期。

复测命令 (三档):
```
bash test/cpu_load.py start 0   # 或不启 (无负载)
cyclictest -m -p 80 -D 30s -h 100 -a 1,2 -t 2 -q
bash test/cpu_load.py start 14  # 50%
cyclictest ... ; bash test/cpu_load.py stop
bash test/cpu_load.py start 25  # 90%
cyclictest ... ; bash test/cpu_load.py stop
```

# RT 基准: mujoco_baseline (2026-09-21 12:32)

| 项 | 值 |
|---|---|
| 内核 | 6.8.0-138-lowlatency |
| PREEMPT | CONFIG_PREEMPT_BUILD=y CONFIG_PREEMPT=y CONFIG_PREEMPT_COUNT=y  |
| cmdline | `BOOT_IMAGE=/boot/vmlinuz-6.8.0-138-lowlatency root=UUID=66fc7927-f6e4-4c1f-b418-9c08f2eceb4a ro quiet splash usbcore.usbfs_memory_mb=512 usbcore.autosuspend=-1 isolcpus=domain,managed_irq,1,2 nohz_full=1,2 rcu_nocbs=1,2 skew_tick=1 nohz=on nosoftlockup nowatchdog nmi_watchdog=0 irqaffinity=0,3-27 vt.handoff=7` |
| isolcpus | isolcpus=domain,managed_irq,1,2 |
| sched_rt_runtime_us | 950000 |
| ulimit -l | unlimited |

## A. cyclictest 三档 (核1+核2, FIFO80, 120s/档)

| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |
|---|---|---|
| 无负载 (0/28) | 00001 / 00003 / 00022 | 00001 / 00002 / 00017 |
| 50% (14进程) | 00001 / 00001 / 00004 | 00001 / 00001 / 00003 |
| 90% (25进程) | 00001 / 00001 / 00006 | 00001 / 00001 / 00006 |

## B. SMI 与中断 (30s 采样)
- SMI 计数(30s, 全核): 工具不可用
- IRQ 活跃行数: 69 (优化后对比: isolcpus+managed_irq 应减少核1/2 命中)
- hwlatdetect(60s, 阈20µs): Max Latency: 22us Samples recorded: 2 Samples exceeding threshold: 2 
- RT 带宽: runtime=950000us / period=1000000us

## C. CM 链路 (mujoco, 三档负载 E2E)

| 负载档 | WCET p50/p99/max (µs) | 跟踪误差 (mm) | 收敛时间 (s) | 备注 |
|---|---|---|---|---|
| 无负载 | p50=1.5µs p99=55.5µs max=508.6 | 0.7377 | 1.025973935 | |
| 50% | p50=0.6µs p99=3.8µs max=508.6 | 0.7377 | 1.043194523 | |
| 90% | p50=0.7µs p99=4.2µs max=508.6 | 0.7377 | 1.276573031 | |

### max=508.6µs 归因 (2026-09-21 复核, 修正初版"物理线程争用"笼统归因)

**508.6 = 500µs IK 求解预算 (update_timeout_ns) + 8.6µs 迭代检查粒度 —— 预算剪枝签名, 非随机尾部。**

机制链: 每档 CM 重激活 → 首批流式 IK 跑冷路径 (RT 核 icache/dcache 被 mujoco 物理线程
mj_step 工作集冲掉, 冷迭代成本高两个量级) → 烧穿 500µs 墙钟预算 → dls_ik 在下一迭代
边界返回 (超出量 = 一次冷 FK+雅可比+SVD ≈ 8.6µs) → **三档 max 逐位相同 508.6 = 同一
机制三次盖的同一个章** (随机事件不可能三个不相交窗口 2522/6622/10991 拍逐位复现)。

日志证据: 每档开头 `流式连续失败 3 拍, 请求冷启动` → worker 35µs 解出同一目标
(题不难, 冷而已) → 流式自愈 (后续 p99 3.8-4.2µs)。

无害性: 周期 2000µs 未超 (500 预算 = 1/4 周期, 正是为突发留的余量); worker 降级路径
按设计工作; 收敛 ~1s 达标。mock 链 "max≈70µs 恒定单发" = 同机制小号 (冷成本 70µs
< 预算, 未被剪, 停在真实冷成本上)。可选加固: warmup() 改多轮带预算求解, 把冷成本
从首拍提前到激活期消化。

复测: bash test/rt_chain_bench.sh <新标签>; 对比 test/results/ 下两文件。

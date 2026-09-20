# RT 基准: baseline_optim_before (2026-09-20 10:28)

| 项 | 值 |
|---|---|
| 内核 | 6.8.0-138-generic |
| PREEMPT | CONFIG_PREEMPT_BUILD=y CONFIG_PREEMPT_VOLUNTARY=y CONFIG_PREEMPT_COUNT=y  |
| cmdline | `BOOT_IMAGE=/boot/vmlinuz-6.8.0-138-generic root=UUID=66fc7927-f6e4-4c1f-b418-9c08f2eceb4a ro quiet splash usbcore.usbfs_memory_mb=512 usbcore.autosuspend=-1 vt.handoff=7` |
| sched_rt_runtime_us | 950000 |
| ulimit -l | unlimited |

## A. cyclictest 三档 (核1+核2, FIFO80, 30s/档)

| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |
|---|---|---|
| 无负载 (0/28) | 1 / 2 / **715** | 1 / 2 / **565** |
| 50% (14进程) | 1 / 3 / **3374** ⚠️ | 1 / 1 / 23 |
| 90% (25进程) | 1 / 7 / **1322** ⚠️ | 1 / 1 / 17 |

## B. SMI 与中断 (30s 采样)
- SMI 计数(30s, 全核): 工具不可用
- IRQ 活跃行数: 69 (优化后对比: isolcpus+managed_irq 应减少核1/2 命中)

## C. CM 链路 (mock xarm7, 三档负载 E2E)

| 负载档 | WCET p50/p99/max (µs) | 跟踪误差 (mm) | 收敛时间 (s) | 备注 |
|---|---|---|---|---|
| 无负载 | p50=2.3 / p99=13.9 / max=74.8 | 0.0003 | 1.06 | |
| 50% | p50=1.7 / p99=11.8 / max=74.8 | 0.0003 | 1.03 | |
| 90% | p50=1.8 / p99=6.1 / max=74.8 | 0.0003 | 1.17 | |

复测: bash test/rt_chain_bench.sh <新标签>; 对比 test/results/ 下两文件。

## 观察 (纯数据, 无预判)

- 平台层 (cyclictest): 核1 Max 无负载 715µs → 50% 负载 3374µs → 90% 负载 1322µs (长尾毫秒级);
  核2 负载下反而更稳 (23/17µs) — 两核行为不对称, 值得优化后验证是否消失
- 链路层 (CM): 三档负载下 WCET p50/p99/跟踪误差/收敛时间 **几乎无差异** (max 恒 74.8 = 同一尖峰)
- 解读注意: CM 的 WCET 记账在 FIFO80 保护下, 平台 cyclictest 同优先级同样保护 —
  但 cyclictest Max 毫秒级尖峰说明保护不完整 (中断/SMI/内核线程不可屏蔽), CM 理论上同样暴露,
  只是 30s 窗口没采到等效尖峰; 优化后两层数据都要对比

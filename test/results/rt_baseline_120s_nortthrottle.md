# RT 基准: baseline_120s_nortthrottle (2026-09-20 10:48)

| 项 | 值 |
|---|---|
| 内核 | 6.8.0-138-generic |
| PREEMPT | CONFIG_PREEMPT_BUILD=y CONFIG_PREEMPT_VOLUNTARY=y CONFIG_PREEMPT_COUNT=y  |
| cmdline | `BOOT_IMAGE=/boot/vmlinuz-6.8.0-138-generic root=UUID=66fc7927-f6e4-4c1f-b418-9c08f2eceb4a ro quiet splash usbcore.usbfs_memory_mb=512 usbcore.autosuspend=-1 vt.handoff=7` |
| sched_rt_runtime_us | -1 |
| ulimit -l | unlimited |

## A. cyclictest 三档 (核1+核2, FIFO80, 120s/档)

| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |
|---|---|---|
| 无负载 (0/28) | 1 / 2 / **2858** | 1 / 2 / **19240** |
| 50% (14进程) | 1 / 1 / **843** | 1 / 2 / **953** |
| 90% (25进程) | 1 / 2 / **2981** | 1 / 8 / **2471** |

## B. SMI 与中断 (30s 采样)
- SMI 计数(30s, 全核): 工具不可用
- IRQ 活跃行数: 69 (优化后对比: isolcpus+managed_irq 应减少核1/2 命中)
- hwlatdetect 阈值标定 (用户三次实测, 2026-09-20):
  - 阈20µs: 0 超阈 (假阴性 — 工具分辨率不足)
  - 阈5µs: 3 超阈 (5/15/0µs inner), **15µs 那次为单发偶发** (复测未重现)
  - 阈10µs: **0 超阈** → 常态量级 <10µs
  - 结论: SMI ≤10µs 常态 + 15µs 偶发, 对 500Hz 占 <0.75%, 归档无害; 套件阈值已定 5µs (回归哨兵)
- RT 带宽: runtime=-1us / period=1000000us

## C. CM 链路 (mock xarm7, 三档负载 E2E)

| 负载档 | WCET p50/p99/max (µs) | 跟踪误差 (mm) | 收敛时间 (s) | 备注 |
|---|---|---|---|---|
| 无负载 | p50=2.1 / p99=10.3 / max=69.8 | 0.0003 | 1.06 | |
| 50% | p50=1.2 / p99=6.1 / max=69.8 | 0.0003 | 1.04 | |
| 90% | p50=1.6 / p99=5.6 / max=69.8 | 0.0003 | 1.08 | |

复测: bash test/rt_chain_bench.sh <新标签>; 对比 test/results/ 下两文件。

## 归因实锤: 500ms 事件 = RT throttling

| 指标 | throttle=950000 (前基线) | throttle=-1 (本基线) | 变化 |
|---|---|---|---|
| 50% 核1 Max | 500037 µs | **843 µs** | **-594 倍** |
| 90% 核1 Max | 500078 µs | **2981 µs** | -168 倍 |
| 50% 核2 Max | 500038 µs | 953 µs | -525 倍 |
| 90% 核2 Max | 499982 µs | 2471 µs | -202 倍 |

单参数修改, 负载档 Max 从 0.5 秒降到 <3ms — **500ms 事件归因 RT 带宽 throttling 实锤**。
(hwlatdetect 已排 SMI, 与本结论一致。)

## 剩余长尾 (throttle 关闭后仍在的)

- 无负载档反而出现 19240µs (核2) / 2858µs (核1) — 无负载时 CPU 进深 C-state,
  唤醒退出延迟大 (符合 RT 手册 §1.6 的预判路径); 负载档反而热核稳定 (843-2981µs)
- 2-3ms 级尾部仍在 = PREEMPT_VOLUNTARY 内核抢占粒度 + 中断 — **低延迟内核 + isolcpus 的目标**
- 注意: 此参数是 sysctl 运行时参数, **当前只 -w 设置, 未持久化** — 重启会回到 950000;
  已建议用户写 /etc/sysctl.d/90-unistackbot-rt.conf

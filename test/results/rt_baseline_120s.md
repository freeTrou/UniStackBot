# RT 基准: baseline_120s (2026-09-20 10:36)

| 项 | 值 |
|---|---|
| 内核 | 6.8.0-138-generic |
| PREEMPT | CONFIG_PREEMPT_BUILD=y CONFIG_PREEMPT_VOLUNTARY=y CONFIG_PREEMPT_COUNT=y  |
| cmdline | `BOOT_IMAGE=/boot/vmlinuz-6.8.0-138-generic root=UUID=66fc7927-f6e4-4c1f-b418-9c08f2eceb4a ro quiet splash usbcore.usbfs_memory_mb=512 usbcore.autosuspend=-1 vt.handoff=7` |
| sched_rt_runtime_us | 950000 |
| ulimit -l | unlimited |

## A. cyclictest 三档 (核1+核2, FIFO80, 120s/档)

| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |
|---|---|---|
| 无负载 (0/28) | 1 / 2 / **1116** | 1 / 2 / **1702** |
| 50% (14进程) | 1 / 10 / **500037** 🔴 | 1 / 10 / **500038** 🔴 |
| 90% (25进程) | 1 / 7 / **500078** 🔴 | 1 / 16 / **499982** 🔴 |

## B. SMI 与中断 (30s 采样)
- SMI 计数(30s, 全核): 工具不可用
- IRQ 活跃行数: 69 (优化后对比: isolcpus+managed_irq 应减少核1/2 命中)

## C. CM 链路 (mock xarm7, 三档负载 E2E)

| 负载档 | WCET p50/p99/max (µs) | 跟踪误差 (mm) | 收敛时间 (s) | 备注 |
|---|---|---|---|---|
| 无负载 | p50=2.1 / p99=10.5 / max=73.4 | 0.0003 | 1.06 | |
| 50% | p50=0.9 / p99=5.8 / max=73.4 | 0.0003 | 1.03 | |
| 90% | p50=1.5 / p99=6.2 / max=73.4 | 0.0003 | 1.16 | |

复测: bash test/rt_chain_bench.sh <新标签>; 对比 test/results/ 下两文件。

## B2. hwlatdetect (SMI/固件级延迟检测, 用户手动跑的基线)

```
sudo hwlatdetect --duration=60 --threshold=20
→ Max Latency: Below threshold; Samples: 0; 超阈: 0
```
**结论: 60s 窗口内无 20µs 以上的固件级延迟 — SMI 不是本机问题源。**

## 120s 长窗结论 (对比 30s 短窗的新发现)

1. **负载档 Max 飙到 500ms 级 (0.5 秒!)** — 50%/90% 两档、两核全部 ≈500000µs。
   这不是调度抖动, 是 **throttle/cgroup/饥饿级事件**: FIFO 线程被长期压制
   (疑因 sched_rt_runtime_us=950000 的 RT 带宽限: RT 线程每 1s 只许跑 950ms,
   超配额被强制休眠 50ms — 与实测 ~500ms 同量级放大后的表现)。
2. 30s 短窗完全没采到这类事件 (3337µs 级) — **短窗严重低估尾延迟, 120s 是必要的**。
3. 无负载档: 核1 1116µs / 核2 1702µs — 比 30s 窗的 715/565 大, 尾部随窗口增长 (正常: 极值分布)。
4. CM 链路层三档仍全稳定 (p99<11µs) — 500ms 平台尖峰期间 CM 恰好没在跑,
   但若发生在控制中 = 250 个丢失周期, 不可接受 → 优化后必须复测确认消失。
5. hwlatdetect 排除 SMI → 尖峰归因收窄到: RT throttling / 中断风暴 / 内核线程。

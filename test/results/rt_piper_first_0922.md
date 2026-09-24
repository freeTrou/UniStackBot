# RT 基准: piper_first_0922 (2026-09-22 14:21)

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
| 无负载 (0/28) | 00001 / 00003 / 00042 | 00001 / 00002 / 00047 |

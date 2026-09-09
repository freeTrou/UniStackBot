# Linux 实时调优手册(RT 装机 + 抖动排查)

> 配套文档:`docs/hardware_framework_design.md`(§3 线程模型、§3.5 时序预算与验收线)、`docs/rt_software_architecture.md`(七支柱总纲:本文的系统级配置是它的落地)。
> 适用平台:x86 控制机 + NVIDIA Orin(Jetson)两条线。
> 结构:**第一部分 系统优化**(装机配置,代码之外)/ **第二部分 代码中的写法**(RT 线程纪律)/ **第三部分 抖动检查专项**(诊断方法论)。
> 心法:**没有测量的实时是信仰** —— 每项配置都要有对应的验证手段。

---

# 第一部分 系统优化(装机配置,代码之外)

## 1.0 延迟敌人总览(每层敌人对应一件武器)

| 层 | 敌人 | 武器 | 所在节 |
|---|---|---|---|
| 调度 | 抢占、RT 限流 | SCHED_FIFO / SCHED_DEADLINE | §1.1 |
| 内核 | 不可抢占的关键区 | PREEMPT_RT | §1.2 |
| CPU | 调度器残留、tick、RCU | isolcpus / nohz_full / rcu_nocbs | §1.3 |
| 中断 | IRQ 落核、irqbalance | IRQ 亲和、线程化中断 | §1.4 |
| 内存 | 缺页、THP 整理、swap | mlockall(代码侧 §2.1)/ 关 THP | §1.5 |
| 电源 | 降频、深睡、睿频、SMI | 锁频 / 限 C-state | §1.6 |
| 数据 | 缓存乒乓、内存序 | 方向分离 / acquire-release(代码侧 §2.5) | — |
| 未知 | 一切 | cyclictest / rtla / perf | 第三部分 |

## 1.1 调度层

| 特性 | 干什么 | 备注 |
|---|---|---|
| `SCHED_FIFO` | 固定优先级抢占,同优先级不轮转 | 本项目:主站 90/核A,CM 80/核B;**禁止两个同优先级 RT 线程挤同核**(会强制轮转);优先级数值本身在代码里设(见 §2.1) |
| `SCHED_RR` | FIFO + 时间片轮转 | 基本不用 |
| `SCHED_DEADLINE` | EDF + 带宽隔离:(runtime, period, deadline) 三元组,**内核保证 CPU 配额** | 多 RT 任务要理论保证时替代拍脑袋优先级 |
| **RT throttling(陷阱)** | 默认 RT 任务合计最多占 CPU 95% | 经典翻车:RT 线程里忙等 → 被限流 → 神秘 50ms 空洞。上线前必查 |

```bash
chrt -f -p 90 <pid>                          # 事后改优先级(调试用)
cat /proc/sys/kernel/sched_rt_runtime_us     # 默认 950000(=950ms/1s)
```

## 1.2 内核抢占与 PREEMPT_RT(地基,不是最后一步)

> **立场:对 1kHz/力控目标,RT 内核是前置条件——不打入,后面的优化都是白费。**

### 为什么没有它其他优化白费

- 通用内核存在**不可抢占区**:自旋锁临界区、关中断的中断处理、softirq 批处理、内存管理慢路径——任何一处都能把 RT 线程按住几百 µs 到 ms;
- isolcpus / mlockall / 锁频消灭的是**你能控制**的延迟源(其他任务、缺页、变频),但**消不掉内核自身**的不可抢占区——你的线程哪怕只睡眠唤醒,路径也要穿过这些内核代码;
- 结果:其余优化把"坏事件的**频率**"压低,而**最坏情况不受界**。非 RT 内核负载下 cyclictest 的 max 可达数百 µs~ms;p99 或许体面,**咬人的是 max**——对 EtherCAT 的 DC 同步质量和看门狗裕量,max 才是关键指标;
- 结论:§3.5 的验收线含 max/p99,在 1kHz 目标下用通用内核达标是**赌博,不是工程**。

### 决策表(部署顺序)

| 场景 | 内核要求 |
|---|---|
| 500Hz + 位置模式(现阶段) | 通用内核 + 本手册全套优化,可达标 |
| **1kHz 总线 / 力控 / §3.5 验收线** | **PREEMPT_RT 前置**,与其余优化是乘积关系 |

部署顺序 = 确定目标 → **先上 RT 内核** → 在 RT 内核上做其余优化与验收。
(§3.2 把 RT 内核排在第 7 步是**诊断**用法——定位"残余是否内核层",不是部署顺序。)

### 获取与验证

```bash
uname -v                                        # 输出含 PREEMPT_RT 字样
zcat /proc/config.gz | grep CONFIG_PREEMPT_RT   # =y
```

- x86:主线 6.12+ 自带;Ubuntu 24.04 官方 real-time 内核包;22.04/Humble 用发行版 RT 补丁内核;
- Orin:NVIDIA 官方 RT 内核包(JetPack 6 时代 Orin 支持);**装前核对版本发布说明的已知限制**(部分电源管理/加速器特性在 RT 内核上受限)。

### RT 内核不替你做的事(乘积关系,不是替代)

- 硬件层延迟照旧:SMI、C-state 退出、DVFS → §1.6 照做;
- 代码坏习惯照旧:循环内 malloc/IO → 第二部分照做;
- 缓存与内存序照旧 → §2.5 照做。

一句话:**RT 内核拆"内核软件"这块天花板,其余优化拆各自的天花板;少了任何一块,最坏情况就由最高的那块决定——1kHz 目标下,内核这块通常最高。**

## 1.3 CPU 隔离与内核启动参数(完整版)

### 1.3.1 参数总表(按功能分组)

**① 隔离组(核心四件)**

| 参数 | 干什么 | 注意 |
|---|---|---|
| `isolcpus=domain,managed_irq,2-3` | 把核从调度域抠出 + 隔离内核管理的 IRQ(现代带标志写法) | 内核文档已标记 deprecated(推荐 cpuset),但一行见效仍是最快路径 |
| `nohz_full=2-3` | 隔离核关周期 tick(每秒少几千次中断) | **boot CPU(CPU0)不能进列表**;需 `CONFIG_NO_HZ_FULL`;列表里第一个核替其余核维护时间 |
| `rcu_nocbs=2-3` | RCU 回调卸载到别的核 | **必须与 nohz_full 同列表**,单独给 nohz 不配这个,RCU 唤醒照样来敲门 |
| `nosmt` | 整机关超线程 | 轻量替代:RT 核与其 SMT 兄弟核不分配负载 |

**② watchdog 组(最常被漏的大户)**

| 参数 | 干什么 |
|---|---|
| `nowatchdog` | 禁 NMI watchdog——**每个核一个周期性 hrtimer**,RT 核上的常驻骚扰,必关 |
| `nmi_watchdog=0` | 同上(x86 写法) |
| `mce=ignore_ce` | 忽略纠错型 MCE 的轮询中断(机器检查轮询也是周期定时器) |
| 运行时:`sysctl kernel.watchdog=0 kernel.soft_watchdog=0` | soft lockup 探测器 |

**③ 电源/idle 组**

| 参数 | 干什么 |
|---|---|
| `intel_idle.max_cstate=1` / `processor.max_cstate=1` | C-state 上限压到 C1(x86) |
| `cpuidle.off=1` | 整个子系统关掉(激进,验证用) |
| `intel_pstate=disable` / `amd_pstate=disable` | 交回传统 cpufreq 驱动,配 governor=performance 用 |
| `idle=poll` | idle 忙等——最强确定性、最费电最费核,只做对照实验 |

**④ 中断组**

| 参数 | 干什么 |
|---|---|
| `irqaffinity=0-1` | 全部 IRQ 的**默认亲和**压到家务核(逐个迁移的兜底) |
| `threadirqs` | **非 RT 内核也能强制中断线程化**——不上 RT 内核时的便宜大改善 |

**⑤ 时钟/NUMA/内存组**

| 参数 | 干什么 |
|---|---|
| `clocksource=tsc tsc=reliable` | x86 锁 TSC 时钟源(避免落到 HPET) |
| `skew_tick=1` | 多核 tick 错峰(减少同一瞬间的集群锁竞争) |
| `numa_balancing=disable` | **关自动 NUMA 平衡**——它会自动迁移你的页,RT 内存要钉死 |
| `transparent_hugepage=never` | 启动级关 THP(比运行时 echo 稳) |
| `audit=0` | 关审计子系统 |

**⑥ 模块黑名单组(modprobe.blacklist=)**

关掉"定时器大户"驱动:`modprobe.blacklist=i915,snd_hda_intel,...`(GPU 驱动的 housekeeping 定时器、声卡,不用就拔)。

### 1.3.2 参数间的依赖与互斥(配错 = 静默无效)

1. `nohz_full` ↔ `rcu_nocbs` **成对出现**,列表一致;
2. `nohz_full` 列表**不能含 boot CPU**;
3. 所有隔离参数依赖内核 `CONFIG_CPU_ISOLATION` / `CONFIG_NO_HZ_FULL` / `CONFIG_RCU_NOCB_CPU`——**没编进去的内核会静默忽略你的 cmdline,一个字都不报**;
4. `isolcpus` 与 systemd cpuset 方案二选一:混用容易出现"自以为隔离了"的核上还有任务。

### 1.3.3 验证(配完必做)

```bash
zcat /proc/config.gz | grep -E "CONFIG_(CPU_ISOLATION|NO_HZ_FULL|RCU_NOCB_CPU|PREEMPT_RT)"
dmesg | grep -iE "isolcpus|nohz_full|rcu_nocbs"     # 内核确实吃进了参数
cat /sys/devices/system/cpu/isolated                 # 隔离生效的核列表
cat /sys/devices/system/cpu/nohz_full                # nohz 生效列表
ps -eLo psr,pid,comm | awk '$1>=2'                   # 还有谁赖在隔离核上(应为空/仅自己的 RT 线程)
```

### 1.3.4 不重启的替代:systemd cpuset(现代方式)

```bash
systemctl set-property --runtime system.slice AllowedCPUs=0-1
systemctl set-property --runtime user.slice    AllowedCPUs=0-1
systemctl set-property --runtime init.slice    AllowedCPUs=0-1
# 之后自己 taskset 进 2-3;配合 IRQ 亲和,效果≈isolcpus 且可在线切换
```

### 1.3.5 平台写法与参考组合

- **x86**:GRUB `GRUB_CMDLINE_LINUX_DEFAULT` 追加,`update-grub` 后重启;
- **Orin/L4T**:改 `/boot/extlinux/extlinux.conf` 的 append 行;**L4T 内核配置未必开了 `NO_HZ_FULL`——先按 1.3.3 验证,没开就别指望这个参数**(Orin 的 cpuidle 深态根治在设备树/sysfs,见 §1.8)。

参考组合(x86,RT 核 2-3):

```
isolcpus=domain,managed_irq,2-3 nohz_full=2-3 rcu_nocbs=2-3 rcu_nocb_poll \
irqaffinity=0-1 nowatchdog nmi_watchdog=0 mce=ignore_ce threadirqs \
numa_balancing=disable transparent_hugepage=never audit=0
```

## 1.4 中断层

- IRQ 亲和:`/proc/irq/N/smp_affinity` 把网卡/GPU/USB 中断赶离 RT 核;
- **关 irqbalance 服务** —— 否则它会把中断自作聪明迁回来(经典坑);
- `threadirqs`(§1.3.1-④)或 PREEMPT_RT 下中断线程化,优先级可控。

```bash
watch -d -n1 cat /proc/interrupts          # 看谁在涨、落在哪个核
watch -d -n1 cat /proc/softirqs            # NET_RX/TIMER/RCU 增量(DDS loopback = NET_RX 周期涨)
systemctl disable --now irqbalance
```

## 1.5 内存系统项

- **透明大页关闭**:`transparent_hugepage=never` —— THP 后台整理是 µs~ms 级尖峰源,RT 机标配;
- swap 关;NUMA 机器内存绑执行核所在节点(`numactl --membind`),且关自动平衡(§1.3.1-⑤);
- `mlockall` + 预触页属于代码动作,见 §2.1。

## 1.6 电源与硬件层(最易忽略,杀伤力大)

| 敌人 | 对策 | 量级 |
|---|---|---|
| cpufreq 降频 | governor 钉 `performance` | 频率漂移 |
| **深度 C-state 退出** | `/dev/cpu_dma_latency` 写 0 并保持持有;或禁 cpuidle 深级 | **C6 退出 30~100µs,极其固定** |
| 睿频/温度降频 | 敏感场合锁基频 | 漂移 |
| **SMI(x86)** | 固件级、OS 不可见 | 单次几十~几百 µs;检测 `perf stat -e msr/smi_counter/`;根治靠选平台 |

```bash
# C-state 排查与禁用
cat /sys/devices/system/cpu/cpuN/cpuidle/state*/name
cat /sys/devices/system/cpu/cpuN/cpuidle/state*/usage
echo 1 | sudo tee /sys/devices/system/cpu/cpuN/cpuidle/state2/disable   # 从最深层逐级禁
```

timer slack 属于代码动作,见 §2.3。症状指纹学(固定几十 µs vs 随机毛刺)见 §3.4。

## 1.7 平台专页 A:x86 桌面控制机

- 检查 `thread_priority` 是否已被 Humble 后期 patch backport(有则删 /proc 扫描方案);
- governor=performance、关 THP、关 irqbalance、检查 RT throttling;
- 桌面环境是训练/推理机时的额外项:GPU 中断与驱动线程避开 RT 核;
- EC 1kHz 阶段:换 PREEMPT_RT(主线 6.12+ 或发行版 RT 包)。

## 1.8 平台专页 B:NVIDIA Orin

Orin 的通缉犯排行榜和 x86 不同,按命中率:

1. **DVFS / EMC 频率未锁**(Jetson 周期性几十 µs 抖动的最常见原因):
   ```bash
   sudo nvpmodel -m 0        # MAXN 功耗模式
   sudo jetson_clocks        # 锁全部时钟(含 EMC)—— Jetson 的 mlockall,不上线不算配好
   ```
2. **深度 idle / 簇空闲退出**:ARM idle 走 PSCI/EL3 固件,簇级深睡退出几十 µs 且固定;idle governor(TEO)按历史预测选状态,预测模式可以真周期性 —— 同时解释"正常 5µs"与"周期性 30µs"。禁深睡重测(§1.6 命令);
3. **timer slack / 相对睡眠合并**(见 §2.3);
4. **核分簇拓扑**:A78AE 分簇(簇内共享 L2,簇间走互连)。RT 线程**同簇不同核**(交换快且确定),housekeeping/GPU 相关赶去别的簇。`cat /sys/devices/system/cpu/cpu*/topology/cluster_id`;
5. **EMC 总线争用**:统一内存,GPU 推理批次与 CPU 实时线程抢内存控制器;锁 EMC 频率消除变频,**不消除带宽争用** —— `tegrastats` 盯 EMC% 与抖动对齐;
6. 工具限制:L4T 5.15 无 rtla → cyclictest + trace-cmd;RT 内核用 NVIDIA 官方包(JetPack 6 时代 Orin 支持);
7. 经验基线:非 RT L4T + 锁频,5µs 基线属正常水平 —— 问题指向平台层(时钟/idle/总线)而非调度。

## 1.9 装机 Checklist(新机器逐项勾)

- [ ] **内核:按 §1.2 决策表——1kHz 目标直接上 PREEMPT_RT(前置条件,非可选项)**;启动参数 `isolcpus= nohz_full= rcu_nocbs= nowatchdog irqaffinity=`(RT 核)
- [ ] 按 §1.3.3 验证参数真实生效(config.gz + dmesg + sysfs)
- [ ] governor=performance;关 THP;关 swap;关 irqbalance
- [ ] 检查 `sched_rt_runtime_us`
- [ ] `/dev/cpu_dma_latency` 由控制进程持有(=0)
- [ ] IRQ 亲和:全部赶离 RT 核(`/proc/interrupts` 复核)
- [ ] Orin:`nvpmodel -m 0` + `jetson_clocks`
- [ ] EtherCAT 口裸口专用,无 IP 无其他流量
- [ ] cyclictest 留基线数据(装机时跑一次存档)

---

# 第二部分 代码中的写法(RT 线程纪律)

## 2.1 RT 线程入口标准模板

顺序有讲究(先锁内存、再触页、再设调度):

```cpp
#include <pthread.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sched.h>
#include <cstring>
#include <ctime>

void* rt_thread(void* arg) {
        // 1. 关 timer slack(相对睡眠的合并窗口,默认 50µs)
        prctl(PR_SET_TIMERSLACK, 0);
        // 2. 锁内存:当前 + 将来,杜绝运行期缺页
        mlockall(MCL_CURRENT | MCL_FUTURE);
        // 3. 预触页:把要用的栈/堆先访问一遍(缺页集中在启动期)
        static char warm[64 * 1024];
        memset(warm, 0, sizeof(warm));
        // 4. 绑核 + FIFO 优先级
        cpu_set_t set; CPU_ZERO(&set); CPU_SET(RT_CORE, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        struct sched_param sp = { .sched_priority = 90 };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        // 5. 主循环:绝对睡眠(不吃 slack、误差不累积)
        uint64_t next = now_ns();
        for (;;) {
                next += PERIOD_NS;
                cycle();        // 干活,守 §2.2 禁令
                struct timespec ts = { (time_t)(next / 1000000000ULL),
                                       (long)(next % 1000000000ULL) };
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }
}
```

优先级方案(主站 90/核A,CM 80/核B)与 `/dev/cpu_dma_latency` 的持有,见设计文档 §3.1。

## 2.2 RT 循环禁令清单

循环体内:

- **零 `malloc`/`free`**(启动期分配完,循环内只碰预分配内存);
- **零 `snprintf`/iostream/异常**(µs~几十 µs,且可能分配);
- **零磁盘/网络 IO、零系统调用热路径**(需要外排的数据写环形缓冲,非 RT 线程取);
- **零无界等待**(任何 wait 带超时——设计文档原则 8);
- 每隔 N 拍的周期性杂务(统计/日志/发布)是**自伤型抖动**的经典来源,要么搬出循环,要么在排查时第一个怀疑它。

## 2.3 睡眠写法:绝对时间,不要相对

- 相对睡眠(`sleep_for`、不带 `TIMER_ABSTIME` 的 nanosleep)默认吃 **50µs timer slack**——内核遇到"顺路"定时器会合并,产生**固定增量的周期性延迟**;
- 绝对睡眠(`next += PERIOD; TIMER_ABSTIME`)两个好处:不吃 slack;上一拍晚了下一拍自动追回,**误差不累积**;
- 若必须相对睡眠:`prctl(PR_SET_TIMERSLACK, 0)` 兜底。

## 2.4 同步原语(具体写法)

### 2.4.1 分层总表

| 层 | 机制 | 阻塞 | 内核参与 | PI | 允许进热路径 | 典型代价 |
|---|---|---|---|---|---|---|
| ① 裸共享内存 + 原子 | 双 buff + 原子 seq | 否 | **零** | — | ✅ 唯一正确选择 | 跨核一次搬运 ~40-100ns |
| ② 信号量(sem/futex) | 敲门制 | 是(带超时) | 一次 syscall | 无 | 每拍一次(相位对齐) | ~百 ns syscall + 唤醒 5~30µs |
| ③ 互斥锁 + 条件变量 | futex + 调度器仲裁 | 是 | 每次 | **PI 版有** | RT 之间真要互斥时 | 同② + 反转风险 |
| ④ pipe/socket/eventfd | 内核拷贝 + 排队 | 是 | 每次 | — | ❌ 仅遥测外排/日志 | µs 级 + 队列抖动 |
| ⑤ 忙等自旋 | 原子标志轮询 | 否 | 零 | — | ❌ | 烧核 + 触发 RT 限流 |

项目内现成范本:`unistackbot_sim_control` 的 SPSC 命令队列(服务线程 → RT 循环)就是第①层的一个生产实现。

### 2.4.2 原子操作与内存序速查

| 序 | 语义 | 本项目用在哪 |
|---|---|---|
| `relaxed` | 只保证原子,不保证顺序 | 计数器(recheck 重试计数、遥测统计) |
| `release`(store) | 本次写入之前的所有读写,对看到该值的读者可见 | 发布槽位:**先写满数据,再翻 seq** |
| `acquire`(load) | 看到该值之后的所有读写,不被重排到之前 | 取槽位:**先看 seq,再读数据** |
| `acq_rel` | 上两者合并(RMW 用) | **宣告式 RMW 必用**——acquire 侧挡"后续数据写上移过宣告"(见下方 fence 方向铁律) |
| `seq_cst`(fence) | 双向全序屏障 | "拷贝 → 校验"的 seqlock 读侧(双向约束,tmp 不下移 + 校验不上移) |

铁律:`std::atomic<T>` 只用于**字大小**的对象(索引/seq/标志);大块数据靠"字大小原子 + 块搬运"组合,不要指望大 T 的 atomic(可能退化成带锁)。

### 2.4.2a fence 方向铁律(两个组件先后栽过的坑,标准和硬件的剪刀差)

**fence 是单向的——`release` 挡"前面下移",`acquire` 挡"后面上移",各自不约束另一个方向。**
写反方向 = 静默失效:x86(TSO) 硬件比标准强,任何 build 都测不出;ARM 上才是真实撕裂。

| 场景 | 正确姿势 | 反面(实战翻车记录) |
|---|---|---|
| **宣告后写数据**(seqlock 写侧: `fetch_add(奇数)` → 写槽) | 宣告用 **acq_rel** RMW(acquire 侧挡写槽上移) | SpLatest 初版 `relaxed + release fence`:不挡写槽上移,**下一次 publish 的写槽可跨边界上移穿过本次宣告**,与读旧槽的读者同槽交叠,撕裂通过校验(第七轮评审修复) |
| **拷贝后校验**(seqlock 读侧: `tmp=数据` → 读 seq 复核) | **seq_cst 全屏障**(双向: tmp 不下移 + 复核不上移) | MpscRing/SpLatest 初版 `acquire fence`:不挡 tmp 下移,复核通过后才读数据 = 读到未来值(第六/七轮评审修复) |

配套纪律:跨线程成员一律原子量(大 T 走"字原子 + 块搬运"或原子字节,见上方铁律);fence 处注释**必须写明挡哪个方向、保护什么**——方向写错时代码照样能跑、压测照样全绿,只有注释里的意图能暴露推理错误。范本:`unistackbot_common/` 三原语(sp_latest / sp_ring / mpsc_ring)的 fence 注释。

**重算法不侵入固定节拍线程**(裁定): 1kHz 的重算法 (WBC 类 QP) 不进 CM 线程 (串行链会拖累全环),
也不迁框架版本换 async 特性 —— 自起专用线程 (优先级介于 CM 与总线之间, 如 87),
与上下游走 SpLatest 双向交接, 敲门级联延伸到该线程。框架 update() 只做双缓冲交换 (~1µs)。
范本: 算法团队 WBC 控制器模板(规划中, 见 hardware_framework_design.md §14.4)。

### 2.4.3 敲门信号量(第②层,完整写法见设计文档 §3.2-敲门制)

```cpp
sem_t knock_;                                   // sem_init(&knock_, 0, 0)

// 写侧(主站)每拍末尾:发完数据敲铃
state_ex_.publish(snap);
sem_post(&knock_);

// 读侧(CM 插件 read() 开头,lockstep 开启时)
int v = 0;
sem_getvalue(&knock_, &v);
while (v-- > 1) sem_trywait(&knock_);           // 只清积压:写侧略快时丢旧铃
struct timespec dl = deadline_ns(now_ns() + 1500 * period_ns);
if (sem_timedwait(&knock_, &dl) != 0) {          // 超时 = 写侧失联(活性探测)
        if (++miss_ >= kMissLimit) return ERROR;
} else {
        miss_ = 0;
}
```

要点:超时即看门狗(不需要额外心跳);读侧迟到是良性的(铃已在,不等待)。

### 2.4.4 PI 互斥锁(第③层,例外路径专用)

```cpp
pthread_mutexattr_t a;
pthread_mutexattr_init(&a);
pthread_mutexattr_setprotocol(&a, PTHREAD_PRIO_INHERIT);   // 低优先级持锁时临时抬升
pthread_mutex_init(&m_, &a);
```

适用条件:**确有低优先级线程持锁、高优先级线程等待**的场合(如 RT 线程与非 RT 线程共享一块懒初始化资源)。本设计的 RT 热路径不含任何共享锁——PI 锁是"违反不了设计时的逃生门",不是日常工具。普通 `std::mutex` 没有 PI,RT 上下文里默认禁用。

---

## 2.5 跨核通信(具体写法)

### 2.5.1 物理背景一页

- 每核私有 L1/L2,一致性协议(MESI)以 **64 字节缓存行**为搬运/作废单位;
- 核 A 写 → 核 B 的副本作废 → B 读时从 A 拉整行(~40-100ns);
- **乒乓**:两个核反复写同一行,行来回飞——带宽占比在 1kHz 下微不足道,但引入**抖动毛刺**;
- **伪共享**(乒乓的阴险形态):两个"不相干"变量挤在同一 64B 行里:

```cpp
// BAD:两个方向挤同一缓存行,每拍互踢
struct Shared {
	double cmd_pos;     // 核A写
	double act_pos;     // 核B写
};                      // ← 全在 64B 内:乒乓

// GOOD:方向分离,各占各的行(alignas 自动把 sizeof 补齐到 64 的倍数)
struct alignas(64) CmdBlock   { double pos; };   // 核A写 → 核B读
struct alignas(64) StateBlock { double pos; };   // 核B写 → 核A读
```

### 2.5.2 可直接抄的"取最新"交换类(本项目交换层的参考实现)

```cpp
#include <atomic>
#include <type_traits>

/// 单生产者-多读者,"覆盖写 + 取最新"双缓冲(写者翻牌、读者只读)。
/// 单生产者是硬约束;读者不限量,各读者独立取各自时刻的"最新"。
/// T 必须是平凡可拷贝的快照(POD、无指针):
///   - 无指针 → 跨进程/共享内存的逃生舱保持打开;
///   - 平凡拷贝 → memcpy 语义,recheck 才有意义。
template <typename T>
class SpLatest {
	static_assert(std::is_trivially_copyable_v<T>, "snapshot must be POD");

	struct alignas(64) Slot { T data; };          // 每槽独占缓存行

public:
	SpLatest() : seq_(0) {}

	/// 构造后未发布过任何数据(seq==0)——读侧用它做"首帧有效"判断(防零值泄漏)。
	bool ready() const { return seq_.load(std::memory_order_acquire) != 0; }

	/// 写侧调用(每拍一次)。写侧私有计数,从不回读共享 seq。
	void publish(const T& v) {
		const uint32_t s = ++w_;                  // 递增,不是 xor 翻转!
		buf_[s & 1].data = v;                     // 写后台槽
		seq_.store(s, std::memory_order_release); // 先写满数据,再翻牌子
	}

	/// 读侧调用。load → 整块拷 → 复查,变了取更新的。
	T take() {
		uint32_t s = seq_.load(std::memory_order_acquire);
		T out = buf_[s & 1].data;
		for (;;) {
			const uint32_t s2 = seq_.load(std::memory_order_acquire);
			if (s2 == s) {
				return out;                       // 复查通过:快照一致
			}
			++retries_;                           // 遥测:长期应恒 0(§3.5)
			s = s2;
			out = buf_[s & 1].data;               // 写入期间换槽了,取更新侧
		}
	}

	uint32_t retry_count() const { return retries_.load(std::memory_order_relaxed); }

private:
	Slot buf_[2];
	uint32_t w_ = 0;                              // 写侧私有
	alignas(64) std::atomic<uint32_t> seq_;       // 牌子独占一行
	alignas(64) std::atomic<uint32_t> retries_{0};
};
```

**四个关键设计点**(每一处都对应一类真实 bug):

1. **递增 seq,不是 xor 翻转**:读方若拿住槽位超过一个写周期,xor 翻转偶数次会**回到原值,recheck 骤然失灵**、读到撕裂数据;递增 seq 把"变过"变成单向事实,双拍回绕也检出。这是本类正确性的核心细节;
2. **release/acquire 的位置**:publish 里 `store(release)` 保证"数据先于牌子"可见;take 里 `load(acquire)` 保证"牌子先于数据"被读。x86 上免费,ARM(Orin)上是必需;
3. **写侧私有 `w_`**:生产者从不回读共享 seq——牌子那一行本来就归它写,但省掉 relaxed load 让 publish 在任何架构上都是纯 store 序列;
4. **`retries_` 是验收遥测**:重试次数 = 读方与写方相位追尾的频率,长期恒 0;一旦增长,说明调度被打乱或拿住时间超标(§3.4 指纹);
5. **冷启动语义**:seq 从 0 起、0 = "从未发布"——防止把零初始化的槽当有效快照读出去(设计文档 P1"首帧有效"在原语层的落点),`ready()` 即此判断。

使用范式(与设计文档 §3.2 对应):

```cpp
SpLatest<RobotStateSnapshot> state_ex_;   // 主站 publish,CM take
SpLatest<JointCmdSnapshot>    cmd_ex_;    // CM publish,主站 take —— 两个方向,两个实例
// 约束:读方 take 后尽快用完(拿住 < 一个写周期);publish/take 皆无锁、无阻塞、无分配。
```

**多读者规则**:
1. 读者只读、写者唯一翻牌 → 多读者机制安全,且 MESI 多读者(Shared 态)无乒乓;
2. **各读者的"最新"互相独立**——需要多消费者基于同一版本协作的算法,此结构不提供(那是版本化快照/RCU 的领域);
3. **快读者直接读(RT 消费者),慢读者读复制者的输出**:非 RT/会缺页的慢消费者可能拷贝时间 ≥ 写周期,重试循环追不上(活锁)——加一个快取快放的复制者线程扇出,慢消费者永远不碰 RT buffer;
4. 多读者时 `retries_` 变为多写计数(轻微争用,无害;要洁癖就每读者一份)。

### 2.5.3 三纪律(把上面收拢成可背的三句)

1. **方向分离 + 单写者**:状态与命令是两个物理隔离的实例——乒乓的结构性解(方向分离 × 方向内双缓冲,是乘积不是二选一);
2. **内存序**:先数据后牌子(release),先牌子后数据(acquire),拷完复查(recheck)。x86 碰运气能跑,**ARM(Orin)必炸**;
3. **缓存行对齐 + 无指针**:`alignas(64)` 每槽/每方向/牌子各占行;快照内用下标不用指针。

**定量**:跨核搬运 ~40-100ns,1kHz 下占周期 ~0.06%——带宽从来不是问题,**确定性和内存序才是**。

---

# 第三部分 抖动检查专项(诊断方法论)

## 3.1 工具箱

| 工具 | 用途 | 平台备注 |
|---|---|---|
| **cyclictest**(rt-tests) | 经典延迟测量 | `cyclictest -m -p95 -a<cores> -h400`;**同核对照可分流"平台 vs 自身代码"** |
| **rtla timerlat / osnoise** | 新一代:分解延迟来源(调度/中断/硬件),不止测量还能诊断 | 需内核 ≥5.17;L4T 5.15 没有,退回 ftrace |
| `perf c2c` | **伪共享专业检测**:直接报争用的缓存行和线程 | x86 成熟;ARM 支持有限 |
| trace-cmd/ftrace | irqsoff / preemptoff / wakeup 深挖个案 | 通用 |
| 自家 seq/ts 遥测 | 应用层最后一道观测(数据年龄、周期 min/max/avg) | 见设计文档 §3.6 |
| 外部总线测量 | 示波器量差分对 / 监听口硬件时间戳 / 从站 DC 偏移 | **金标准**;注意 DC 读数含补偿量化,解释时剥一层 |

## 3.2 排查方法论(七步分流)

```
1. cyclictest 同核对照 ──→ cyclictest 也脏 = 平台层;只有应用脏 = 自身链路(搜索空间砍半)
2. 锁频(jetson_clocks / performance)──→ 消除 DVFS/EMC 变频
3. 禁深睡 + timerslack=0 ──→ 消除唤醒路径增税
4. 量尖峰周期,对照 §3.4 指纹表
5. 循环体自查:每 N 拍的杂务(统计/日志/snprintf/malloc)
6. perf c2c 查伪共享(方向分离后应查无)
7. 仍有残余 → 上 RT 内核对照(诊断用法;部署上它本就是前置条件,见 §1.2)
```

## 3.3 EtherCAT / IgH 专属排查

外测(总线帧时刻)看到周期性抖动时,链路后段两个 IgH 特色嫌疑:

1. **EtherCAT 网口不专用**:该口挂了 IP/有其他流量(SSH/DDS/ARP),qdisc/驱动队列让帧排队 —— **IgH 的口必须裸口专用**,`ip addr` 确认;
2. **循环内周期性后台事务**:IgH FSM 把后台 SDO 读(轮询温度/故障码)、EEPROM、EoE **摊进总线周期** —— 每隔几拍轮到一次,该拍执行时间就长一截,天然周期性。停掉所有 `ecrt_sdo` 后台请求重测。

相关性指认法:把外测尖峰时间戳与嫌疑事件流(DDS 发布节拍 / SDO 轮询周期 / GPU 批次 / 4ms tick)对齐,谁对上谁是真凶。

## 3.4 症状指纹表

**按形态分**:

| 形态 | 指认 |
|---|---|
| 周期性、固定几十 µs | 唤醒路径(深睡退出/timer slack)/ SMI |
| 随机毛刺 | IRQ/软中断落核、抢占、缓存乒乓 |

**按尖峰间隔分**(用于周期性抖动):

| 尖峰间隔 | 指认 |
|---|---|
| 每控制周期 | 线程交互(同步原语/写冲突) |
| ~4ms(250Hz) | tick/softirq 落核(IRQ 亲和) |
| 几十 ms | governor 残留(DVFS/EMC 采样) |
| 不规则簇发 | GPU/EMC 争用 |
| 与某话题频率一致 | DDS loopback 软中断 |

## 3.5 验收线(与设计文档 §3.5 一致)

| 指标 | 线 |
|---|---|
| CM 周期 | p99 < 2.2ms @500Hz |
| 命令年龄 | p99 < 5ms |
| 总线循环抖动 | p99 < 周期的 10% |
| cyclictest 同核 | max < 周期的 10%(与装机基线对比) |
| 双 buff recheck 计数 | 长期恒 0 |

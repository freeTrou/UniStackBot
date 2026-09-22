# SocketCAN 实时主站 · 完整设计方案(CAN FD 后端,形态 B 线程1)

> 配套:`docs/architecture/hardware_framework_design.md`(形态 B / master.hpp 五要素 / v3 语义 / 协议层)、`docs/guides/linux_rt_guide.md`(§2.1 线程模板、§2.5 SpLatest)、`docs/bus/ethercat_master_design.md`(EC 分支,结构平行)。
> 纯 C++17 + POSIX + SocketCAN;目标平台 NVIDIA Orin + RK3576 双平台,用户态代码同源,平台差异收进配置与自检。
> 定位:master.hpp 接口的第二个真实后端(fake 为第一个)。

## 0. 总览

```
线程1 · CAN FD 主站(1kHz,FIFO 90,专用核)
┌────────────────────────────────────────────────────────┐
│ cycle()(每拍,预算 1ms,全程无睡眠调用):                │
│  1. rx_drain()        非阻塞排空,latest-wins 每轴       │
│  2. watchdogs()       心跳超时/错误队列 → 故障 → 急停旗  │
│  3. step()            状态机权威(中立词汇,v3 语义)      │
│  4. tx_commands()     非阻塞写,内核 qdisc 仲裁          │
│  5. state_ex.publish  快照(关节+协议状态+seq/ts)         │
│  6. ring.push         尾部复制(egress)                  │
└────────────────────────────────────────────────────────┘
节拍:绝对时间 clock_nanosleep(主站即时钟——CAN 无 DC)
交互:state_ex ↓ / cmd_ex ↑(与 CM,形态 B 两通道)+ ingress/egress(域界)
```

设计四支柱(按疼的程度):①节拍源(主站即时钟)②负载预算(仲裁制,1kHz 是算出来的)③故障感知(无 WKC,看门狗自己造)④socket 用法(五选项)。

## 1. 节拍架构(主站即时钟)

- **时钟形态**:CAN 无 DC,1kHz 由主站循环产生——循环节拍质量 = 总线调度质量,一切 RT 手段为此服务;
- **paced loop**:绝对时间睡眠(`next += PERIOD; TIMER_ABSTIME`),不吃 slack、误差不累积;TIMERSLACK=0;
- **overrun 遥测**:`cycle()` 结束时 `now > next` 则计数——**overrun 率是主站第一验收指标**;
- RT 前提(引用 RT 手册):PREEMPT_RT(1kHz 前置)、FIFO 90、专用核、mlockall+预触、`/dev/cpu_dma_latency` 持有、两平台各存 cyclictest 基线。

```cpp
void* bus_thread(void*) {
        prctl(PR_SET_TIMERSLACK, 0);
        mlockall(MCL_CURRENT | MCL_FUTURE);
        /* FIFO 90 + 驻核(配置化) + 预触 */
        uint64_t next = now_ns();
        for (;;) {
                next += PERIOD_NS;
                cycle();
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ns_to_ts(next), nullptr);
        }
}
```

## 2. Socket 层(五个必须对的选项 + 收发纪律)

```c
int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on));   // ① FD 帧
struct can_filter flt[] = {{/* 只订本站 ID */}};                   // ② 内核层过滤(否则杂帧进用户态,softirq 变重)
setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, flt, sizeof(flt));
setsockopt(s, SOL_SOCKET,  SO_TIMESTAMPNS, &on, sizeof(on));      // ③ 内核收帧时间戳 → 快照 ts
can_err_mask_t em = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_TX_TIMEOUT;
setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &em, sizeof(em));  // ④ 错误队列(链路健康线)
ioctl(s, FIONBIO, &on);                                           // ⑤ 非阻塞
bind(s, &addr, ...);  // if_nametoindex(cfg.interface)
```

**收发纪律(RX/TX 不对称)**:
- **RX 必须非阻塞(无条件)**:反馈到达时刻由从站决定,阻塞等帧 = 从站晚回你也晚、从站死你卡死、看门狗失效;排空量 ≤ 负载预算,时间确定;
- **TX 非阻塞写**:CAN 的 `write()` 返回 ≠ 帧上总线,= 已交内核 qdisc——**仲裁竞争由硬件做(µs 级)**,用户态久留纯属白付抖动;
- 调用点失败只产出两个遥测:RX 无(心跳看门狗负责发现),TX `ENOBUFS`(qdisc 溢出 = 循环落后于总线,与 overrun 同级报警);
- 整个 cycle() 无任何睡眠调用 → "一拍执行时间是确定小量"成立。

## 3. 负载预算与发帧策略(hardest part)

帧时间公式:`t帧 ≈ 仲裁位/仲裁速率 + (开销+payload×8)/数据速率`(例:40bit@1M + 32B@5M ≈ 95µs)。
**1ms ÷ t帧 ≈ 每拍帧数上限,再打 70% 折扣**(错误帧/重发余量;负载率长期 >70% 会在错误风暴时排队雪崩)。

三策略组合:

| 策略 | 做法 | 代价 |
|---|---|---|
| 命令复用 | 一帧 FD 打包多轴目标 / 命令变化才发(事件驱动) | 协议支持 |
| 反馈分频 | 关键轴每拍,其余轮询(N 拍一轮) | 反馈年龄 1~N 拍(per-source ts 诚实标注) |
| 广播同步 | 同步帧触发从站按槽回反馈 | 协议支持 |

流程:先 `candump` 抓真实流量 → 算实际负载 → 定策略 → 长期监控负载率遥测。

## 4. 故障感知三件套(P2 的 CAN 落码)

**错误帧感知与处理细节**:物理 CAN 错误帧(6 显性位)用户态不可见——被控制器硬件消化为 TEC/REC 计数与状态迁移(error-active → error-passive → bus-off);用户态感知的是错误队列里的**状态报告帧**(`CAN_ERR_FLAG` 置位,与正常帧同队列到达,rx_drain 分流)。

**错误分类全集**(linux/can.h 9 类,按处理等级分组):

| 组 | 错误类 | 含义/系统解读 | 处理 |
|---|---|---|---|
| 链路生死 | `BUSOFF` / `TX_TIMEOUT` | 控制器 bus-off(TEC≥256)/ 驱动层发不出 | **本地急停旗 + 停 TX** |
| 链路生死 | `RESTARTED` / `CRTL_ACTIVE` | bus-off 恢复到 error-active | **状态机回 INIT 重走全链**,重锚定后放行(=P4 链路层分支) |
| 控制器健康 | `CRTL` data[1]:WARNING(≥96)/PASSIVE(≥128) | 总线质量劣化(线缆/终端电阻) | 记账+预警,不停机 |
| 控制器健康 | `CRTL` data[1]:RX/TX_OVERFLOW | **自己没排空快** | 自锅报警,查 rx_drain 耗时 |
| 接线诊断 | `TRX`(data[4]):CANH/CANL 断线/短路 | 直接的接线故障诊断,bring-up priceless | 记账+诊断输出 |
| 接线诊断 | `NOANSWER` / `PROT` LOC_ACK | 请求无应答/无节点 ACK(帧实际丢失) | 与该轴心跳看门狗交叉验证 |
| EMI 指标 | `PROT` data[2]:STUFF/BIT/FORM | EMI 指纹——**电机功率级开启后频发 = 功率线缆耦合**,bring-up 必抓 | 记账+与运行状态相关性分析 |
| 负载健康 | `LOSTARB`(data[0]=输在哪个位) | 仲裁失利=正常竞争现象;**频率=负载健康度**(频繁→查负载预算) | 记账 |
| 洪水风险 | `BUSERROR` / `PROT` 高频 | 每次检测都报,可能每秒成百条 | **只递增计数**(RT 纪律循环内零打印),遥测线程按秒汇总;必要时订阅期滤掉 |

**错误读取通道清单**(五条,各有分工):

| # | 通道 | 类型 | 读到什么 | 用途 |
|---|---|---|---|---|
| 1 | `CAN_RAW_ERR_FILTER` + 普通 `read()` | 事件·主路径 | 全部 9 类错误报告帧 | rx_drain 常驻分流 |
| 2 | `recvmsg(MSG_ERRQUEUE)` | 事件·次路径 | **本机 TX 失败**(帧被本地丢弃) | TX 健康遥测(与通道1互补:1=总线/控制器怎么了,2=自己的帧怎么了) |
| 3 | netlink `RTM_GETLINK` / `ip -s -d link` | 轮询 | 链路计数(rx/tx_errors、bus-off 次) | 周期对账 |
| 4 | `/sys/class/net/canX/statistics/` | 轮询 | 同上 sysfs 面 | 脚本/CI |
| 5 | `ethtool -S` / debugfs | 轮询 | 驱动私有计数(部分含 TEC/REC 原始值) | bring-up 深挖 |

主路径三细节:①订阅即开关(默认不投递);②**错误帧永远以 `struct can_frame`(16B)到达,即使开了 FD**——rx_drain 必须两种 MTU 都认,否则错误帧被当垃圾或错位解码;③payload 布局 per `linux/can/error.h`(data[0]=类, [1]=控制器状态, [2]=协议类型, [3]=位置, [4]=收发器, [5..6]=私有)。

**驱动能力警告 + 双平台错误面测绘**:错误帧由驱动构造,**类别覆盖度是驱动能力不是协议保证**(有驱动漏报 passive 迁移的先例)——Orin(M_TTCAN)与 RK3576(CANFD)必须各自实测订阅面(人为制造:拔线→BUSOFF?短路→TRX?干扰→PROT?),实测写进平台能力声明,盲区由通道 3~5 轮询兜底;此测绘并入 §8 第 3 步装机自检。

**订阅三注意**:①`CAN_ERR_FLAG` 须同时置入 filter 的 can_id 和 can_mask(漏一处收不到或全收);②handler 只计数不打印(洪水防护)+ 错误帧本身占带宽,吵闹总线订阅面收窄;③分级订阅:链路生死+TRX/NOANSWER 常驻,PROT/LOSTARB bring-up 全开、量产收窄(写进能力声明运行模式)。

**事件槽记录结构**:`ErrEvent{type, axis, ts_ns}` 8~16 字节环形覆盖,循环内只递增计数 + push;打印/汇总/上报在遥测线程(非RT)——错误记录复用遥测骨架,零新机制。

**第一版处理策略(2026-09-07 用户拍板:记录为主)**:9 类错误中 7 类纯记录(计数器 + 事件槽)——**但 BUSOFF 必须置急停旗、RESTARTED 必须通知状态机回 INIT(各一行)**,否则 TX 死后系统继续"成功"地 write、命令静默丢失(分界判据:错误发生后系统继续运行是否安全自洽)。恢复阶梯/重锚定/放行本来就长在状态机里,非新增处理。bring-up 阶段全量记录积累健康指纹(LOSTARB/PROT/overflow 曲线 = 总线基准),量产时再按数据逐类升级处理策略。

配套:`restart-ms 100` 链接配置时设置;周期兜底 `ip -s link` 计数对账;故障注入必测"拔线→bus-off→急停→恢复→INIT 重走→重锚定→放行"全链。

**错误计数与状态升级机制**(每种错误背后的账本):每节点有 TEC(发送)/REC(接收)两个计数器——检测到错误:发送方 TEC+8、接收方 REC+1;成功发/收各 -1(下限 0,**这就是偶发错误可长期共存的原因:生产消费相抵**)。升级阶梯:TEC/REC ≥96 → error warning(仅标记);≥128 → error-passive(TX passive 帧后强制等 8 位=变相降优先级;RX passive 不能发主动错误旗打断别人);**TEC ≥256 → bus-off(节点整体断开)**,检测到 128 次 11 连续隐性位后待重启(restart-ms 自动化 → CAN_ERR_RESTARTED 通知)。

**BRS 两层开启**:①链路配置一次 `ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on`(无 fd on 时发 canfd_frame 直接 EINVAL——启动自检校验;ISO 模式默认,non-iso 仅全总线老设备时);②每帧 TX 手动置位 `f.flags = CANFD_BRS`(不置位 = 全程仲裁速率,慢 4~10 倍);RX 不设(硬件自带 BRS/ESI 信息位)。dbitrate 按负载预算表定,两平台实测回填。

### 4.1 五种帧级错误检测机制(分类的根)

每个节点收发每帧时硬件并行跑五道检查:

| 机制 | 检测逻辑 | 通俗版 |
|---|---|---|
| 位错误(Bit) | 发出的位回读总线比对:发隐性读到显性(仲裁期除外=合法失利) | "我说的和我听到的自己不一样" |
| 填充错误(Stuff) | 5 个同电平后必须插反相位,检出 6 连续同电平 | "该插同步位的地方没插" |
| 格式错误(Form) | 固定格式位场被违反(定界符/EOF 非隐性) | "帧的骨架不对" |
| CRC 错误 | 接收方算出 CRC ≠ 收到 | "内容在半路被改了" |
| ACK 错误 | ACK 槽发隐性,**无任何节点**拉显性 | "没有任何人收到我" |

**关键理解:五种机制是同一批物理根因的不同投影**——一束 EMI 可能表现为 CRC(接收算错)、stuff(位翻转凑满 6)、bit(回读错);排查看分布与相关性,不看单条。

### 4.2 症状 → 根因排查表(现场对照用)

| 观察到的模式 | 最可能根因 | 验证动作 |
|---|---|---|
| 电机通电才出现 STUFF/BIT | **EMI 耦合**(功率线缆×CAN 线平行/非屏蔽/共地不良) | 电机断线再跑;查屏蔽/走线/共地 |
| 随关节转动/线缆弯折零星错误 | **线缆内部断股**(运动磨损) | 弯折处晃动复现 |
| 上电就有、恒定 STUFF/FORM | 采样点/波特率配置不一致,或 FD **TDC 未开** | 对齐三平台(Orin/RK/驱动器)位时序 |
| 高 dbitrate 才错、仲裁段干净 | 信号完整性:终端电阻缺失(两端各 120Ω)/反射/TDC | 量终端;降 dbitrate 对比 |
| ACK error 成串 | 拓扑/供电:从站没活 | candump 看还有谁在说话 |
| FORM 错误自某时刻起恒定 | 坏节点入网 / ISO 与 non-ISO 混用 | 逐个拔从站定位 |
| LOSTARB 高频 | 负载超预算 | 回预算表,启用分频/复用 |
| BUSOFF 反复 | 上述任一长期恶化滚到 TEC≥256 | **先修根因——restart-ms 只是止痛药** |

**CAN FD 专项注意**:数据段速率高(dbirate 2~5M)位宽缩到 200~500ns,对位时序/线长敏感度放大数倍;高速数据段需 **TDC(收发延迟补偿)**,控制器未开或配错 → 数据段成片 bit/stuff 错而仲裁段干净——FD 时代新坑,排查看"分段"。

## 5. 与已定架构的映射

| 已定设计 | 本方案落点 |
|---|---|
| 形态 B 线程1 | bus_thread 骨架 |
| v3 语义 | step() 权威;状态机词汇中立(Piper 协议映射到 READY/ENABLED/QUICK_STOP/FAULT) |
| master.hpp 五要素 | ①生命周期 start/stop+state() 出口,调度参数配置化 ②中立词汇+能力声明(无DC/心跳看门狗/收帧级ts) ③快照语义接口/双缓冲内部 ④quick_stop 原子旗+fault 查询同形 ⑤零协议类型泄漏 |
| 看门狗链 | CM 盯 state_ex 陈旧;线程1 盯 cmd_ex 陈旧 + 心跳 + 错误队列;驱动器失能保护兜底 |
| 延迟档案 | CAN 项 = 收帧级 ts + 实测抖动 → 喂 RL 训练侧延迟注入 |

## 6. 双平台兼容(Orin + RK3576)

- 用户态代码 100% 同源(SocketCAN + POSIX 公共面);平台差异在内核驱动/设备树,不进代码;
- 配置文件每平台一份:接口名、驻核、帧速率、预期帧时间;
- 启动自检:内核 CAN FD 支持、驱动版本、实测一帧往返时间(两板 t帧 可能差 20~50%,负载预算按实测填);
- 驻核:Orin 同簇大核;RK3576 只用 A72(A53 不放 RT 线程)。

### 4.3 五种帧级错误的物理根因速查(与 SocketCAN 类的映射)

| 帧级错误 | SocketCAN 报告类 | 典型物理根因 |
|---|---|---|
| Stuff | `PROT` data[2]=STUFF | EMI 翻位、采样点不一致、线长×速率、FD TDC 未配 |
| Bit | `PROT` data[2]=BIT、`LOSTARB`(仲裁期=合法) | 总线竞争、收发器退化、EMI |
| Form | `PROT` data[2]=FORM | 噪声砸固定位、坏节点发畸形帧、ISO/non-ISO 混用 |
| CRC | `PROT` data[2]=`CAN_ERR_PROT_CRC` 类(部分驱动) | 噪声突发改写内容、信号裕量边缘、终端电阻缺失反射 |
| ACK | `PROT` data[3]=LOC_ACK、`NOANSWER` | 唯一节点在线、从站全掉、断线、波特率完全不匹配 |

> 注:五种机制是同一批物理根因的不同投影(一次 EMI 干扰可分别表现为 stuff/CRC/bit),**排查看分布和相关性,不看单条**。参考:内核文档 docs.kernel.org/networking/can.html、linux/can.h。

## 7. 验收指标

| 指标 | 线 |
|---|---|
| 循环 overrun 率 | 长期 0(偶发要归因) |
| cyclictest 同核基线 | 两平台各存档,max < 周期 10% |
| 心跳看门狗误报率 | 0(拔线测试必须 100% 触发) |
| ENOBUFS 计数 | 恒 0 |
| 负载率 | <70%(峰值场景复测) |
| state_ex 年龄 | ≤1 拍;反馈分频轴按策略标注 |
| bus-off 恢复 | 全链路(急停→恢复→重锚定→放行)注入测试通过 |

## 8. 实施顺序(先证据后代码)

```
1. candump 抓真实流量 → 协议帧格式/节奏/负载确认
2. 负载预算表 → 定发帧策略
3. 裸 socket pacing 测试(无协议内芯):两平台 1kHz 循环抖动
   + cyclictest 同核对照 ← 与用户代码基准对比的尺子
   + 错误面测绘:两平台人为制造错误,核对 CAN_ERR_* 类实际到账
     (驱动能力差异,盲区写进能力声明,轮询兜底)
4. 协议内芯(状态机映射/编解码)
5. 看门狗 + 错误队列 + 恢复阶梯
6. 故障注入全套(拔线/bus-off/ENOBUFS/overrun)
```

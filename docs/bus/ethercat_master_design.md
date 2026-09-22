# IgH EtherCAT 后端 · 完整设计方案(形态 B 线程1,EC 分支)

> 配套:`docs/architecture/hardware_framework_design.md`(§4 多主站矩阵 / master.hpp 五要素 / v3 语义 / 协议层)、`docs/guides/linux_rt_guide.md`(§2.1 线程模板、§2.5 SpLatest)、`docs/bus/socketcan_master_design.md`(CAN FD 分支,结构平行)。
> 定位:master.hpp 接口的 EC 后端;IgH ecrt 用户态 API,主站线程内跑总线循环,1kHz + DC。
> 状态:错误处理(§1–§3)为完整定稿;DC/预算/验收为设计要点,随实测回填;§8 坑点清单为调研定稿(出处随附)。

## 0. 总判断:错误可见性是协议原生的

EtherCAT 的错误可见性**每帧、每从站、可拓扑定位**——CAN FD 里要"自己造三件套"(心跳看门狗/负载预算/看门狗)的活,EC 全部协议原生白拿。这是它在主站矩阵里的真实能力差异,能力声明如实反映(不是谦虚)。

```
线程1 · EC 主站(1kHz,DC 对齐,FIFO 90,专用核)
  cycle(): ecrt_send_receive_process_data()      ← 收发(DC 硬边界)
         → domain_state 检查(WKC)               ← 事件通道,免费
         → slave states 检查(AL)                 ← 从站语义故障
         → 急停旗直查 + 链路层故障本地置旗
         → step() 状态机(v3 权威)
         → state_ex.publish / 发帧 / 尾部复制
  慢通道(非RT):mailbox 读端口计数器/寄存器      ← 劣化记账+段定位
  轮询:ip link / NIC 计数对账                    ← 防驱动漏报
```

## 1. 错误分类学(八类,机制→检测→语义)

### 1.1 WKC 异常(Working Counter,每拍王牌)

**机制**:每个 ESC 若完整处理一帧数据报,把 WKC +1(读/写/读写组合各有期望贡献);主站每拍将实际 WKC 与期望值比对。

**能区分的子情况**(按期望值分解):

| 现象 | 语义 |
|---|---|
| WKC = 期望 - k(k 固定,反复出现) | k 个从站不处理该数据报:掉电/复位/未到 OP/内部挂死 |
| WKC 波动(每拍不同的 k) | 边缘性故障:接触不良、EMI 打坏帧被下游 ESC 丢弃(CRC 检查失败不转发) |
| WKC = 0 | 帧整体没回来:断链/主站 NIC 故障 |

**检测**:`ecrt_domain_process()` → `ecrt_domain_state()` → `working_counter` / `wc_state`(COMPLETE/INCOMPLETE/ERROR),**每拍免费**。
**对照 CAN**:ACK 缺失 + 心跳超时 + 逐轮询定位三件事的合体,且每拍都有。

### 1.2 断链(拓扑定位,CAN 做不到的王牌)

**机制**:EtherCAT 端口自动转发——环断/线断时,帧从断点反向回到主站,从站数下降;每个 ESC 端口有链路信号监测。

**定位逻辑**:`slaves_responding` 下降 + `ecrt_master_link_state()` 的 `lost_links/signal/up` → **定位到"第 N 与第 N+1 从站之间"的段**,秒级,无需人肉分段。

**处理**:断链 = **链路层本地急停**(≤1 拍);定位信息进诊断输出;恢复后(re-up)从站状态机重走,重锚定后放行。

### 1.3 AL 状态码(从站主动上报的语义故障)

**机制**:从站 AL 状态机(error-active 层)出错时,经状态/寄存器**主动上报带语义的错误码**:DC 同步失败、SM watchdog 触发、无效输入/输出配置、应用层故障、意外状态迁移等(码表见 ETG.1000.6)。

**检测**:`ecrt_slave_config_state()` → `al_state`(INIT/PREOP/SAFEOP/OP + ERROR)/`online`/`operational`,每拍可得;码本身经 SDO/寄存器读(慢通道)。

**语义分层**:AL 迁移到非 OP = 从站不再执行运动(自动安全)→ 状态机层故障处理(不是链路层急停——链路还活着,只是从站自己撤了);恢复阶梯消费错误码决定走哪级(应用复位 vs 重初始化)。

### 1.4 ESC 端口错误计数器(EMI 指纹 + 段定位)

**机制**:每个 ESC 端口维护 RX 错误计数、转发错误计数、转发丢链计数(寄存器 0x0300 起,ETG.1000.6)。

**关键性质**:**WKC 还没坏先爬升**——EMI/线缆劣化的早期信号;且按端口归属**能定位到劣化的线缆段**。

**检测**:mailbox SDO 慢读(非RT 周期 1~10Hz,不进 RT 循环)。
**处理**:纯记账/预警(增长趋势遥测);触发排查动作的是人,不是控制逻辑。

### 1.5 SM Watchdog(从站侧安全终点,P2 的原生实现)

**机制**:从站的 Sync Manager watchdog(0x0410/0x0420 配置窗口):主站过程数据停发/迟到超窗 → 从站**自行**撤出 OP、输出进安全态、报 AL 错误。

**这是整个安全链的从站侧终点**:进程死(IgH fd 关→master 释放→帧停)之后,真正让电机安全的就是它 + 驱动器自身保护。**CAN 里这要自建心跳+失能保护,EC 是两个寄存器**。

**配置纪律**:窗口 = 主站周期的合理倍数(如 100ms,覆盖几十拍);太小则正常抖动误触发,太大则死亡发现慢——数值写进能力声明。

### 1.6 DC 时钟错误(1kHz CSP 关键健康指标)

**机制**:从站 DC 偏差超配置窗 → 报 DC 错误(AL 状态码);偏差本身寄存器可读。

**意义**:CSP 模式下从站在 DC 边沿采样目标值——偏差大 = 采样时刻抖 = 控制质量退化,**先于功能故障出现**。CAN 无此概念(也无同步时钟需求)。

**处理**:偏差进遥测(趋势);持续超窗 → 记账/状态机(视从站 AL 行为)。

### 1.7 主站/NIC 自锅层

IgH 统计的 tx 错误/丢帧、NIC ring 溢出、`ecrt_master_receive` 异常——"自己没跟上"的报警线,与 CAN 的 ENOBUFS/overrun 同级语义。

### 1.8 Mailbox/SDO 错误(慢通道语义)

参数读写的 abort code(谁拒绝/为什么)、mailbox CRC 错——恢复阶梯和参数管理的输入,不影响 RT 循环。

## 2. 三通道检测结构(与 CAN 后端同构)

| 通道 | 载体 | 周期 | 内容 |
|---|---|---|---|
| 事件(免费) | cycle 内 ecrt 调用 | 每拍 | WKC/domain state、slave states(AL)、急停旗 |
| 慢通道 | mailbox SDO(非RT) | 1~10Hz | 端口错误计数器、DC 偏差、错误码详情 |
| 轮询对账 | `ip link` / NIC 计数 | 1s | 防驱动漏报;与事件计数差值异常 = 本身是诊断 |

**驱动能力警告**(同 CAN 后端):错误报告覆盖度取决于 **NIC 驱动 + ESC 固件**——不同 NIC×ESC 组合对各类错误的上报完整度不同(有驱动漏报 passive 迁移的先例)。**两平台错误面测绘是装机自检**:人为制造(拔线→WKC/断链定位?从站断电→AL 码?干扰→端口计数爬升?主站停发→SM watchdog?),实测写进平台能力声明,盲区由轮询兜底。

## 3. 处理分级(对齐架构,零新增概念)

| 等级 | 触发 | 动作 | 对应不变量 |
|---|---|---|---|
| **链路层急停** | WKC 持续 ERROR / 断链 / slaves_responding 下降 | 线程1 本地置急停旗(≤1 拍),**不经状态机** | G3 双源置旗权之一 |
| **状态机故障** | AL 错误码(从站语义故障) | CiA402 线程消费 → 恢复阶梯(INIT 重走)→ 重锚定 → 放行 | v3:cmd_valid 自然为 false,上层零感知 |
| **记账/预警** | 端口计数爬升、DC 偏差趋势、NIC 偶发 | 遥测(EMI 指纹 + 段定位输出) | — |
| **自锅报警** | NIC 溢出 / 主站 overrun | 报警 + 查预算/调度 | — |

**分级判据(与 CAN 分支同一句)**:错误发生后系统继续运行是否安全自洽——链路生死类必须上报(急停旗),劣化类记录即可。

**第一版策略**(同 CAN 拍板):记账类全量记录(计数器+事件槽,循环内零打印零分配,遥测线程汇总);急停类两行置位;**WKC 是唯一每拍免费且必须查的**——它是 EC 后端的"seq"。

## 4. DC 与循环要点

- DC:循环内每拍 `application_time` + `sync_slave_clocks`;**`application_time` 必须喂 CLOCK_MONOTONIC 衍生值**(喂墙钟会被 NTP 回跳打乱 DC 补偿基准,发作呈周期性、极难排查,见 §8.2);`sync_slave_clocks` 按 1.6 文档推荐模式调用;偏差进遥测;
- 负载:帧时间确定(调度制),预算表按从站数×过程数据量算,余量策略同 CAN §3;
- 循环 overrun 率 = 第一验收指标(同 CAN);
- SM watchdog 窗口:略大于主站周期的合理倍数(如 100ms),与驱动器侧保护构成从站安全终点。

## 5. 验收指标

| 指标 | 线 |
|---|---|
| 循环 overrun 率 | 长期 0 |
| WKC 事件 | 稳态 COMPLETE;INCOMPLETE 频率归因清零 |
| cyclictest 同核基线 | 存档,max < 周期 10% |
| DC 偏差 | < 配置窗的 50% |
| 端口错误计数 | 稳态零增长(增长 = EMI/线缆段排查) |
| 故障注入 | 拔线(定位到段)/从站断电(AL 码)/主站停发(SM watchdog 触发)/DC 扰动 全链路通过 |

## 6. 实施顺序(先证据后代码,同 CAN)

```
1. 两平台错误面测绘(NIC × ESC 组合):人为制造错误,核对到账类别
2. 裸循环 pacing 测试(无从站逻辑):1kHz 抖动 + cyclictest 对照
3. 预算表(从站数×过程数据量)
4. 协议层内芯(CiA402 映射——协议层形态盲,见设计文档 §4)
5. 看门狗链 + WKC/AL 检查 + SM watchdog 配置
6. 故障注入全套(拔线定位/从站断电/主站停发/DC 扰动)
```

## 7. 与 CAN FD 分支的共享件

master.hpp 契约、SpLatest 通道、快照契约、协议层(形态盲)、看门狗链语义、急停旗、事件/慢/轮询三通道结构、错误面测绘方法论——**两后端的差异只在内芯**(帧层 API、错误分类学、能力声明),骨架零重复实现。

## 8. IgH 使用已知坑点与规避(外部经验,2026-09 调研定稿)

来源:IgH 官方文档 1.6、etherlab-users 邮件列表、Intel/TI 论坛装机案例、stable-1.6 NEWS(出处见 §8.7)。覆盖状态标注:✅ 已结构性规避 / ⚠️ 需注意(有残余动作) / 🔧 设计输入(实现 EC 后端时落实)。

### 8.1 帧调度类(最高频,几乎人人踩过)

| 坑 | 机制与症状 | 规避 | 状态 |
|---|---|---|---|
| "Datagram SKIPPED n times" 刷屏 | 上一拍的帧还没发出,本拍 send 又到 → 数据报标记跳过、该拍命令作废。根因:RT 调度缺失 / cycle 过快吃满周期 / 负载尖峰 | Form B(专用 RT 线程 + FIFO 90 + 隔离核)从源头压 jitter;负载尖峰期残余的偶发 SKIPPED 是**健康信号不是 bug** → 记日志(ulog)不告警 | ✅ + ⚠️ 记账 |
| receive→process→send 顺序错/漏调 | 顺序乱 → WKC 异常/数据陈旧;漏 receive → WKC 陈旧;漏 send → 从站看门狗触发。每拍必须恰好一次三连 | master.hpp 把三步封装为原子序列(骨架件),调用方无法拆散 | ✅ |
| "Datagram UNMATCHED" | 帧回来了但主站已不再期待该数据报——cycle 与帧回程竞速;常伴负载升高,多数为噪声但指向调度问题 | 同 SKIPPED:压 jitter + 记账 | ✅ + ⚠️ 记账 |

### 8.2 DC 时钟类(第二高频,且症状隐蔽)

| 坑 | 机制与症状 | 规避 | 状态 |
|---|---|---|---|
| `application_time` 喂墙钟 | CLOCK_REALTIME 被 NTP 回跳 → DC 补偿基准跳变 → 从站采样时刻跳 → CSP 控制质量退化,**且随对时周期性发作**(查不到的"周期性怪病") | 必须喂 CLOCK_MONOTONIC 衍生值——RT 手册 MONOTONIC 规则在 EC 后端的直接落点,已并入 §4 | 🔧 |
| `sync_slave_clocks` 调用模式错 | 每拍 vs 周期调、漏调 → 从站时钟漂移 | 按 1.6 文档推荐模式调用;DC 偏差进遥测(§1.6 验收线) | 🔧 |

### 8.3 驱动/NIC 类(装机期最常见)

| 坑 | 机制与症状 | 规避 | 状态 |
|---|---|---|---|
| native 驱动与内核网卡驱动冲突 | 内核驱动仍占 NIC → master 卡 "waiting for devices" / 起来后无帧;blacklist 因 early-load 顺序有时不够,须显式 rmmod | bringup 装机脚本:rmmod + blacklist 双管;启动失败第一查此项 | ⚠️ 脚本待补 |
| generic 模式性能上限 | 多一跳内核栈拷贝,jitter 特征不同 | 起步用 generic(已定),预算不达标再换 native | ✅ 已定 |

### 8.4 RT 循环禁忌类

mailbox SDO 进 RT 循环(阻塞)、循环内 malloc/printf——本架构 RT 手册禁令清单 + ulog 替代 + 慢通道独立非 RT 上下文,已全覆盖 ✅。RTDM 扩展有已知死锁(Vectioneer 出过非官方补丁)——不用 RTDM 即不涉及,记录在案。

### 8.5 生命周期类

| 坑 | 机制 | 状态 |
|---|---|---|
| ORPHANED 从站 | 主站释放/卸载时从站还在 OP → 从站成"孤儿"留在 OP,直到自身 watchdog 触发 | ✅ 设计已定:进程死 → fd 关 → 帧停 → SM watchdog + 驱动器保护兜底(§1.5) |
| slave config 泄漏 | 长跑进程 `ecrt_slave_config` 不释放 → 内存增长 | ✅ 每硬件组件固定配置一次,无泄漏面 |

### 8.6 版本/平台类

- EEPROM/SII 读取在部分嵌入式平台失败(AM335x 先例)→ 装机自检(§2 错误面测绘)覆盖。
- 1.5.2 卡 "waiting for devices" → 即 §8.3 驱动冲突,同解。
- 选版本核对 stable-1.6 NEWS(数据报重排序保护等修复)。

### 8.7 结论与出处

**坑点分布规律**:运行期坑(帧调度/DC)被 Form B + MONOTONIC 规则结构性规避大半;装机期坑(驱动冲突)一条 rmmod/blacklist 脚本闭环;真正剩下的**实现期设计输入只有两个**——`application_time` 喂单调钟衍生值、`sync_slave_clocks` 正确调用模式(均已并入 §4)。

出处:

- [IgH EtherCAT Master 1.6 文档](https://docs.etherlab.org/ethercat/1.6/pdf/ethercat_doc.pdf)(DC/application_time/sync 官方口径)
- [etherlab-users 2019 "Random Datagram Unmatched" 线程](https://lists.etherlab.org/pipermail/etherlab-users/2019-May/011166.html)(SKIPPED/UNMATCHED 与负载关系)
- [TI E2E AM335x "SKIPPED" 帖](https://e2e.ti.com/support/processors-group/processors/f/processors-forum/541398/am335x-icev2-issue-with-igh-ethercat-master-for-linux)(嵌入式平台 SKIPPED + SII 读取失败)
- [Intel 社区 "Stuck in waiting for devices" (1.5.2)](https://community.intel.com/t5/Intel-Edge-Software-Hub/Stuck-in-quot-waiting-for-device-s-quot-v1-5-2-IgH-EtheCAT/m-p/1617825)(驱动冲突,rmmod+blacklist)
- [stable-1.6 NEWS](https://gitlab.com/etherlab.org/ethercat/-/blob/stable-1.6/NEWS.md)(版本修复清单)
- [Vectioneer RTDM 补丁集](https://git.vectioneer.com/pub/etherlab/-/tree/48f32505b1066ae471a4c0a83e3b1d8612cdc77c/patches-default-33b922/0001-unoffical-patchset-20190904)(RTDM 死锁)

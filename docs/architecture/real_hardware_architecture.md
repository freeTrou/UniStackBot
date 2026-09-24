# 真机架构 — 多总线主站 × 多臂本体（定稿视图）

> 定位：从 `hardware_framework_design.md`（讨论纪要）**分叉的架构定稿视图**——2026-09-24
> 真机专题（串口主站设计 + 多臂形态 + 调度实测）收口后的总览与思路存档。
> 讨论过程、未决项、历史裁决细节见母文档（§4 多主站矩阵 / §17 多臂与调度）；各总线
> 完整设计见 `bus/rs485_master_design.md` / `bus/socketcan_master_design.md` /
> `bus/ethercat_master_design.md`；调度实测数据见 `unistackbot_common/rt_tune/README.md`。
> 第一消费者：**R1-7A 七轴臂**（宇树 IM 系协议，RS-485@6M）。

## 0. 一页总图

```
算法层 (RL/VLA/规划, 非RT, 门外, 不可信命令源)
   │  统一接口: 整身命令流/反馈流 (身份在消息里)
   ▼
controller_manager (单 update 线程, FIFO 80, 核1)          ← ros2_control: 串行确定性
   ├─ 硬件组件 ×N (每总线段一个, read/write=缓冲交换)
   ▼
泵线程 ×N (总线节拍, FIFO 85-95, 核2)                       ← 我们: 并行总线节拍 (形态B)
   ├─ serial_master  (485/232/TTL 骨架 + 协议内芯)
   ├─ canfd_master   (SocketCAN CAN FD)
   └─ ethercat_master(IgH ecrt + DC)
   ▼
总线 → 驱动器 (固件看门狗 = 安全链终点) → 电机
```

**两层正交**：框架管"控制器串行确定性"（单 update 线程是有意设计——顺序固定/控制器间
零锁/成本可加和），我们管"总线并行节拍"（形态 B 把时序敏感工作移出 CM 线程，CM 循环
只剩换缓冲+µs 级数学——"想阻塞都没地方"）。

## 1. 设计原则（七条，全部来自已裁决案例）

1. **骨架只认字节和时间**——总线骨架代码 grep 不到任何协议词汇（零类型泄漏的反向版）。
2. **机器=配置，协议=内芯**——换臂/换协议/加臂，骨架零改动；"第二台机器人成本=一份描述文件"。
3. **确定性来源是轮询表和优先级阶梯，不是核**——串口连仲裁上界都没有，主站完全掌控时序；
   用优先级买确定性，不用核堆叠。
4. **身份在消息里，不在 topic 名里**——命令按身份路由进来（关节名/frame_id），
   反馈带着身份出去（frame_id/chain）；同语义流一个 topic。
5. **状态机翻译到唯一中立机，绝不并集**——粗态（READY/ENABLED/QUICK_STOP/FAULT）
   是控制交集且恒真；协议差异经**细分格**（IS-A 加细，能力位声明：STANDBY/瞬态终态）
   与 plan() 动作序列吸收，异构活动状态归正交轴；并列式并集机没人能证明对。
6. **异构不拉平**——能力位图让上层看见差异（MIT 前馈 vs 仅位置），不把强臂降级成弱臂。
7. **数据触发纪律**——预算+计数器+实测，没有证据不动配置（加核/调参/改机制全适用）。

## 2. 分层：四轴绑定表

```
机器人 (机器)
 └─ 臂/链 arm ——— 描述层实体 (URDF 子链 + IK 链)          ← 轴④ 形态
     └─ 总线段 segment ——— 一个 hardware 组件 + 一个泵线程  ← 轴① 总线
         └─ 节点 node ——— 协议内芯实例                     ← 轴② 协议
             └─ 状态翻译器 ——— 映射到唯一中立状态机          ← 轴③ 状态机
```

正交性判据：换任一轴，其余三轴零改动。协议诚实分两族——**帧式**（请求-应答字节流：
unitree_im/Dynamixel/Modbus → FrameSpec+codec 纯函数）与**对象式**（对象字典+PDO/SDO：
CiA402 over EC/CANopen → mailbox+字典映射）；共享的是中立词汇/能力声明/快照交换，
**不是 codec 形状**。能力声明位图（HAS_HW_WATCHDOG/HAS_BROADCAST/FIXED_LEN...）防
最小公分母——骨架按位适配不假设。反模式三不：不做万能协议描述框架 / ratio 关节名
不进代码 / 协议词汇不越内芯边界。

### 2.1 组合架构与统一接口（插件点定义，2026-09-24 补全）

**组合发生在装配期，运行期结构固定**——组合根（SystemInterface）读 URDF/yaml
（master/protocol/nodes）装配 Transport+Framer+ProtocolCore+Scheduler+Health，
启动期校验（fail-fast），运行期零重组。**没有运行期插件化需求**（机器装配完不换协议），
换来完全可分析的结构。

**三个插件点 + 一个契约层**：

- **插件点① IoTransport**（轴①总线）：只管字节，各总线一实现（Termios/Fake/CAN/EC）。
- **插件点② ProtocolCore**（轴②协议）：一个类内含三面——
  **控制面** `encode(cmd, ratio)→帧`；**反馈面** `decode(帧)→通用物理量+遥测+raw码`；
  **错误面** `interpret(raw)→{类别,人话,严重度提示}`；外加状态翻译
  （`plan(中立命令)→动作序列` / `observe(反馈)→中立态` / `special(清除/复位/广播)`）。
  **三面是一个插件的三面，不是三个插件**——三面永远同变（unitree 的 codec 必配
  unitree 错误表），拆开=非法组合空间（A codec+B 错误表=胡说）；面间复用按
  "两处以上消费才上收"。
- **中立状态机唯一归属 Health**（骨架）：协议状态机差异的实质 = ①到达路径不同
  （CiA402 controlword 三步链 vs unitree 一步——故 plan() 返回**动作序列**，泵逐拍执行）
  ②观察词汇不同（observe() 归一）。绝不并集。
  **细分格修正（2026-09-24 用户纠偏: 四态不足以表达 CiA402 与自定义协议的细微差距）**:
  中立机为**两层词汇**——粗态恒真（READY/ENABLED/QUICK_STOP/FAULT，控制器与
  ros2_control 生命周期只看这层）+ **细分格按能力位可选声明**：READY→READY(高压未上)
  |STANDBY(CiA402 "Switched on"，高压已上未使能——**deactivate"高压保留"语义的落点**，
  母文档 §3.3 生命周期表本就用了三个 CiA402 态，四态中立机写不出那张表)；QUICK_STOP 与
  FAULT → 减速中|已停（瞬态与终态，安全状态显示需要）。**细分格保持 IS-A 关系——
  细态属于唯一粗态，粗态视角对一切协议永远合法（抽象不破），这是它与并集反模式的本质
  区别**（并集是异构状态并列，如把 HOMING 塞进电源态机——活动类状态归 activity 正交轴，
  命令模式 CSP/CSV/MIT 同理）。协议原态（statusword 解码枚举）作为**一等遥测**随反馈走
  （排障要真波形）。框架代码: 粗代码查粗态；生命周期钩子按能力位用细格
  （有 STANDBY 格则 on_deactivate→STANDBY，无则退化 READY）。
- **错误面统一机制 = 分类在内芯，策略在骨架**：内芯 interpret 出类别+严重度提示，
  Health 施加对所有协议同一套的升降级策略（info→计数 · warn→告警 · fault→段隔离/
  广播停）——错误语义的丰富性在内芯，处置的统一性在骨架。
- **能力声明做组合守门员**（configure 期全查任一即拒）：内芯↔骨架能力匹配
  （无硬件看门狗则必须配软件看门狗）/ 机器配置↔内芯匹配（ratio·ID·限位，无默认纪律）/
  时隙预算装配打印。
- **插件机制**：SystemInterface 用 pluginlib（ros2_control 要求）；内芯/transport 用
  包内**静态注册表**（name→make()，加协议=加文件夹+一行注册）——插件体验、可进 gdb、
  零动态加载边界。
- **组合矩阵验证纪律**：不测 N×M——**声明式支持对**（unitree_im×serial、CiA402×EC、
  Piper协议×CAN），每对一个 fake-bus 集成测试；新增组合=新增声明+一个测试。

### 2.2 包结构与职责（2026-09-24 定案：每个功能包职责单一）

```
unistackbot_interface        通信契约 (msgs/RobotCommand/RobotFeedback/kMaxJoints) —— 协议词汇零进入
unistackbot_hardware/        ★真机域容器 (仿 sim_control 先例: 目录归家, 内含两个 colcon 子包)
  ├─ protocol/    (包 unistackbot_protocol)   codec 轴: ProtocolCore 父类(framespec/encode/decode)
  │               + UnitreeImCore 子类 + 工厂 createProtocolCore —— 已落地, 甲骨文 256 cases 绿
  ├─ statemachine/(包 unistackbot_statemachine) 状态翻译轴 (2026-09-24 拆轴独立): StateTranslator
  │               父类(map_state) + UnitreeImTranslator 子类 + NeutralState 四态 + 工厂 —— 9 cases 绿
  ├─ bus/          (包 unistackbot_bus)   总线交换轴 (2026-09-24 立轴, 同款组织):
  │               MasterBase 父类 (五要素最小集代码化, 纯交换零线程可见) + BusCommand/BusState
  │               自持 (零 ROS 链) + 工厂 (注册表空占位, serial 随泵骨架落地) —— 7 cases 绿
  ├─ serial_master/  ethercat_master/  canfd_master/   三骨架目录 (IO 工程, 骨架子类 SerialMaster 等)
依赖方向: interface ← protocol ← statemachine ← bus ← 骨架子类 (单向, 逐层);
         SystemInterface (ROS 适配层) = 唯一碰 ROS 的总线路径, 只握 MasterBase
```

**拆轴裁决 (2026-09-24 用户拍板)**: 状态机与协议**同款组织、分居两包、同居一容器**
(父类+子类+工厂, 各自注册表一行加新)。协议=字节↔物理量 (codec), 状态机=反馈→中立态
(翻译)——两轴独立演化 (CiA402 的富状态机/影子机将来只动 statemachine 包)。

**状态获取谱系（"粗态必报"的修正——粗态是框架对上的承诺，不是对协议的假设）**：
直报（状态随反馈帧，CiA402/unitree）/ 查询（主动发查询命令，plan() 在调度里交错
查询帧并为其留预算）/ **影子推断**（裸协议不报状态——Health 跑影子机：命令轨迹+ack
证据 → "推断为 X"，`STATE_IS_SHADOW` 能力位诚实标注开环信念，安全锚只剩看门狗）。
协议良莠不齐被吸收为获取策略+能力声明。

### 2.3 最小集与加法纪律（2026-09-24 定稿：先最小框架, 再做+法, 不破原设计）

**判据: 最小 = 第一个消费者（R1-7A）+ fake 能完整跑起来的最小区面; 凡"为将来"留的一律
显式延后。** 完整目标接口是 §2.1（图纸不动）, 本节定义切片 1 只实现其中哪些。

**最小接口集（MasterBase 后经 SDO 类别纠偏扩为 8 方法, 全栈 ~16）**:

- `MasterBase`（8）: start/stop · publish_cmd/take_state（**周期交换 PDO 形**, RT）·
  read_param/write_param（**慢通道事务 SDO 形**, 2026-09-24 用户纠偏补类别——参数/对象
  读写带超时、非 RT 上下文专用、无能力协议返 kUnsupported; "④慢通道 mailbox"的父类投影）·
  quick_stop · state()→粗态（无细分格）——交互类别 = 周期交换 + 事务访问 + 即发命令
- `ProtocolCore`（4）: encode · decode · map_state→粗态 · framespec（帧头/定长/CRC 位置）
- `IoTransport`（3）: read/write/close（全非阻塞, open 走构造）
- `Framer`（1）: push(bytes)→[帧]（定长+帧头+CRC+坏帧重同步）
- （泵/health 是骨架内部类, 不跨包, 不计入接口面）

**显式延后清单（=加法地图, 第二个消费者出现才动）**: Caps 位图 / plan() 动作序列
（等 CiA402; unitree 使能=一帧不需序列）/ 影子机·查询式获取（等裸协议）/ 细分格
（等 CiA402）/ interpret 完整错误表（先返 raw 码）/ special() clear·reset·广播 /
Framer 长度域·静默间隔模式（等 Dynamixel/Modbus 类）/ 调度分组交错·速率几何
（等第二总线段）/ 双段聚合（等双臂）。

**加法不破框架三机制**: ①接口只加不改（新方法=带默认实现的新虚函数, 既有签名语义
永不变更）; ②测试是回归锚（切片1 的 golden frames+fake 全链永久保活, 每加法=新测试
+旧测试全绿）; ③延后项归宿已在 §2.1 图纸上（加=填空, 非重构）。

**切片 1 完成定义（R1-7A, 全零硬件）**: ①unitree_im codec+golden frames 对拍厂商 CRC
②FakeTransport+假从站→帧泵+四态机全链（使能即锁定/quick_stop/追帧跳过）
③TermiosTransport+本机回环 ④MasterBase+SystemInterface 接 ros2_control（chain:=real
能起）。四步走完, 真机到货只剩上机清单+点火。

**脱 ROS 纪律（2026-09-24 用户考量定稿）**: 大部分组件不耦合 ROS——脱开时现成可用
（common 七组件/algorithm 已兑现; protocol+serial 骨架是动工目标; 控制器插件/msgs/
SystemInterface 是 ROS 面=换壳层）。**新纪律: 核心文件零 ROS include**——protocol/
骨架/契约头不得 include 任何 rcl*/ros2_control 头; **验收=每步都有 g++ 直编测试且绿**
（"现成可用"是测试文件里的一条编译命令, 不是宣称）。纯度在文件级不在包级（包可 ament,
核心文件纯 std——拷 protocol/+serial_master/+common/ 三目录裸 g++ 能编）。
脱开 ROS 的组合: common+algorithm+protocol+serial骨架 = 无 ROS 控制核心（配自定义
main 即裸 Linux 部署）, ros2_control 只是当前外壳（形态 B 隐含承诺的显式化）。
**动工顺序=从最纯往最脏**: ①契约头(半天,图纸→代码首次受控碰撞) → ②unitree_im
codec+golden frames(g++对拍厂商CRC,脱ROS第一实证) → ③Framer+FakeTransport+假从站
(模糊测试) → ④泵+四态机(全链完成且100%零ROS) → ⑤ROS适配(TermiosTransport是Linux
非ROS; SystemInterface=全工程唯一新 include ros2_control 的文件, 薄到一眼看完=终审)。

### 2.4 业界参考实现调研（2026-09-24, 动工前; raw 直取核实）

| 实现 | 分层 | 体量 | 对我们的价值 |
|---|---|---|---|
| DynamixelSDK (ROBOTIS) | PortHandler(transport) × PacketHandler(协议v1/v2) × GroupHandler(批量) | 13+15+3 | **timeout 住 transport** / 错误人话化在协议层(getRxPacketError) / 坏帧重同步姿势 / SyncWrite 按地址批量写不同值 |
| unitree_actuator_sdk (厂商自家) | IOPort→serialPort + unitreeMotor + crc 独立 | 4 | sendRecv 事务形(一次=全扫, 放泵层) / **MotorType 枚举+每机型序列化结构体=同族多代电机共存解法** / MIT 五元组确认; **反面教材: IOPort include unitreeMotor.h = 协议类型泄漏进 transport** |
| rm_hw (rm-controls) | CanBus(bus,dataPtr,prio) 构造起 RT 线程 | 5 | 极简共享交换 + 优先级构造参数 |
| legged_control (qiayuanl) | 同类分层 | — | 已停维护; 侧面证明领域收敛 |

**结论**: ①我们的 transport×protocol 分离与业界四方收敛一致; ②最小集体量(14)落在业界
区间(4-15); ③零泄漏纪律被宇树反面印证; ④Dynamixel "PacketHandler 拿 PortHandler*
当参数"的组合姿势同我们哲学。**净修正三处**: IoTransport 补 bytes_available()(3→4方法);
unitree_im 预留 MotorType 枚举位(6014 起步同族扩展); 泵内部吸收 sendRecv 事务形代码组织。

## 3. 总线主站层

**三骨架**（`unistackbot_hardware/` 三目录，设计各成文）：串口骨架五件套
（transport/framer/pump/scheduler/health）+ `protocols/` 内芯；CAN/EC 结构平行。
**契约**：master.hpp 五要素（生命周期+健康出口/中立词汇+能力声明/快照接口
RobotFeedback·RobotCommand/quick_stop 统一面/零协议类型泄漏）。

**帧泵模型**（串口实例化，机理通用）：周期拍内 = 取最新命令 → 编码 → 单帧单 write
（绝不合并——USB 会拼帧 → 线上背靠背碰撞）→ 非阻塞排干读 → framer → 分发 → 快照发布。
**命令年龄恒定 1-2 拍**（不等本拍响应——lock-step 在 USB 延迟下每事务 1-4ms 必死）。
**落后追帧纪律**：逾期超一个时隙即跳到未来期限、绝不补发——FIFO 同优先级不时间片
轮转，补发狂奔会饿死同核兄弟（实测教训）。半双工逐节点时隙调度是物理必需
（时隙 = 帧上行 33µs + 固件响应 X + 帧下行 43µs + guard；R1-7A 官方 700Hz 上限反推
X≈108µs，update_rate 定案 500Hz 全扫）。

## 4. 调度设计（全部实测背书）

**优先级阶梯（定稿）**：EC **95** > CAN **90** > 串口 **85** > CM **80**；同总线实例
同优先级（双泵是对称实体）。排序依据 = **分发延迟容忍度**（容迟最低最受保护：EC 硬
时钟节律 DC+WKC > CAN 仲裁+socket 队列 > 串口时隙余量+1s 固件看门狗）。

**核数哲学（定稿）**：隔离的作用是**把 RT 与家务隔离**，不是 RT 线程互相隔离（互相靠
阶梯+突发链预算）——**隔离核按账最小化 1-2 个，与机器大小无关，核多也不开 4 个**。
部署阶梯：4 核级板卡 = 1 隔离核（全系统同核形态）/ 常规 = 2 隔离核。**单核最多 2 条
总线**；更多实例双方案并存：①首选上核1 与 CM 同住（优先级天然高于 80，CM 核占用
~1%）②加隔离核仍是正当备选。

**低优先级最坏延迟公式**：不是"对方一个突发"，是**更高优先级突发链之和**（phase5
实测 S85 p99=156.7µs 恰=EC 突发 152µs）。

**实测摘要**（bench_dual_pump 七相位，详见 rt_tune README）：

| 相位 | 布局 | 关键数 |
|---|---|---|
| 双泵错相（当前部署） | 核2 × 串口泵×2 | p99 4.2µs = 与 solo 无差别 |
| EC+串口对 | 核2 两总线 | S85 p99 157µs < 285µs 时隙 |
| 四线程全布局 | 核1 CM+泵A / 核2 EC+泵B | CM 45µs=2.3% 周期，全绿 |
| 全系统单核 | 四线程一核 | 双泵 max 192µs / CM 126µs=6.3%，全预算内 |

**SCHED_DEADLINE 备注**：理论替代（runtime/period 声明+EDF 准入）——突发链公式是
FIFO 口径、生态先例薄；RT 线程 10+ 异构速率时再评估，当前规模 FIFO 阶梯已实测完备。

## 5. 多臂本体

**部署拓扑**：双臂 = 两条总线段 = 两个主站实例（**共享代码绝不共享状态**——段间故障
隔离的全部来源；总线 ID 原样重复零改 ID；同型号 USB 转换器 by-id 撞名 → udev
by-path 兜底）。

**控制形态（市场调研后定稿：模式②单进程+按臂控制器）**：市场三模式——①命名空间
双进程（多机器人隔离）/②单 CM+按臂控制器（TIAGo++/人形产品主流）/③单控制器管
双臂（MoveIt both_arms 路线）。**选②**：一个 controller_manager + 每 485 总线一个
`<ros2_control>` 块 + 按臂成对实例化 `js_left⇄cm_left / js_right⇄cm_right` + ee×2 +
supervisor×1。核心纪律：**切换按臂成对**（任一关节永远恰有一个 active 控制器——避开
JS 整体退位导致对侧失管）。**镜像在基座摆放**（右臂基座 180°，关节符号不翻，两臂
插件参数除链名外一致）。**单 update 线程成本 5%、余量 20×**——官方 scaling 阀门
（单拍 update 逼近预算才拆双进程）不触发。

**接口一致性（定稿）**：拆控制器 ≠ 拆接口。

```
命令面统一: /joint_stream_controller/command (14 关节一条消息, 桥按关节名拆)
           /cartesian_motion_controller/target (按 header.frame_id 路由)
反馈面统一: /joint_states (本来一个) · /ee_state (remap 合流, frame_id 区分)
           /status (remap 合流 + 显式 chain 字段——唯一 msg 改动)
桥接层:    非 RT 外观节点 (命令拆分, 零控制语义) + 纯 remap (反馈合流)
```

原则：**同语义流一个 topic；身份在消息里，不在 topic 名里**——命令按身份路由进来，
反馈带着身份出去，对称。红利：录包/分析工具链 topic 清单**单双臂完全同名**（一致性
传导到整条验收资产）；RL/VLA 观测出口 = 整身一个源。

**同构 vs 异构**：同构 = 镜像配置标志+同 solver；异构 = 速率几何（各段自持节奏取最新
衔接——形态 B 结构强制性兑现：异构速率互不拖拽）+ 能力位图不拉平 + 按段隔离。
本体论口径（讨论稿，§17.8）：双臂同桌 = 部署场景思路；单本体 vs 两本体建模是开放
选择（机制层都支持），真到双臂同控再裁。

## 6. 安全与错误面

- **安全终点在驱动器固件**（README 立场成立）：unitree timeout 位 1s 自停 / CiA402
  SM watchdog / CAN 驱动器失能。崩溃语义天然：进程死 → 帧停 → 看门狗跳。
- **使能即锁定**：随使能发实测位姿+kp，消灭零力矩窗口（重力掉臂）；使能序列失败
  → 回滚全体 stop（半使能不可接受）；过流/过热不自动 clear（防复撞）。
- **广播急停**（ID15 同类）：quick_stop 原子旗每拍直查，机器人级急停 = 各段旗标
  各自广播本段。
- **看门狗链**：整链无无人监视的生产者（非RT←CM←泵←固件看门狗）；**反馈盲区冻结**
  （单节点 stale→冻结保持位，过半→广播停——kp/kd 锁定不需要 host 反馈，冻结安全）。
- **write() 防线同型照抄**（sim_control 预演形态：NaN 门→限位→步长+故障计数器）。
- **错误面全景**：串口无协议原生错误面，全部自造——三层计数器（响应缺失会计/CRC 错/
  TIOCGICOUNT）+ 主机侧防呆（USB autosuspend 必禁/设备消失禁自动重连/部分写/开机
  有效帧率自检）+ 电机侧（自发重启 mode 位对账/位置跳变 Δq 断言/过温降额/电压监控）。
  详见 `rs485_master_design.md` §8（表可按骨架泛化到其他总线）。

## 7. 测试与验证资产

| 资产 | 用途 | 位置 |
|---|---|---|
| bench_dual_pump（七相位） | 每台新部署机器的**装机自检**（不同板卡跑一遍出核预算） | `rt_tune/` |
| FakeTransport 假从站 | 零硬件全链（codec golden frames / framer 模糊 / 泵+调度+安全注入）——P14 fake master 的镜像 | serial_master 设计时落地 |
| 双 FakeTransport | 双段整机零硬件台架（镜像逻辑/速率掩码/聚合 worst-of） | 同上 |
| 点火四步 | 盲读→零速锁定→微动（ratio 配错签名=幅度差 32/12.67/16 倍）→跟流（rate 阶梯 50→100→250→500） | `rs485_master_design.md` §10 |
| 上机第一天清单 | 回环/echo/量端接·偏置·共地→低速→6M→TIOCGICOUNT 压测→rtt 分布 | 同 §8 |
| 机型验收 pipeline A-G | 单机器人一次跑完（双臂按 14 关节一份登记表） | `sim/sim_validation_pipeline.md` |
| **sim2real 迁移方法（§6）** | 现实梯度（mock→gz→mujoco→假总线→真机四步）+ 七步路径 + 数字迁移账 + RL 策略两件套（延迟档案随机化/不可信命令源进门）+ 诚实清单（sim 永远验不了的五样）+ 毕业五条定义 | 同文档 §6（2026-09-24 定稿） |

## 8. 决策索引

**定稿**（用户拍板）：优先级阶梯 95/90/85/80 · 核数哲学 1-2 个 · 单核最多 2 总线 +
第三条双方案 · update_rate 500Hz（R1-7A）· 控制形态=市场模式② · 接口统一（身份在
消息里）· 单 update 线程保持一个 · 使能即锁定/回滚/不自动 clear。

**讨论稿/待实测**：EC/CAN 突发量假设值（EC 后端落地复核）· 适配器选型（宇树模块 vs
FT232H，rtt 分布判据）· 双臂本体建模单/双（机制层都支持，届时裁）· 同步双臂的
DualCartesianController（一等公民候选，按需求排期）· kp/kd per-robot yaml 键位。

**母文档**：`hardware_framework_design.md`（§3 形态B线程模型 / §4 多主站契约 /
§16 命令平滑 / §17 多臂与调度——本文件的讨论过程与完整论证）。

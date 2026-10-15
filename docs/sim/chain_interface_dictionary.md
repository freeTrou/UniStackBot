# 链上接口词典（以 piper mock 链为基线；三链同源，差异见 §6）

> 运行面可观测的一切：话题名 / 类型 / 字段含义 / 频率 / 查看方式 / 服务 / 控制器状态。
> 定义先行：本词典是接口的规范描述，实现与词典冲突 = 实现 bug。
> 字段定义以 msg 文件为权威（`unistackbot_interface/msg/`）。

## 速查：我要 X，用哪条（2026-10-13 外部评审落地；新人第一屏）

| 我想… | 用这个 | 注意 |
|---|---|---|
| 看臂的末端在哪 | `/ee_state` 50Hz | **位姿事实源**；yaml `publish_hz` 可调 |
| **高频位姿回路（>50Hz）** | **`/joint_states` 500Hz + urdf_fk 自算 FK** | JTC 官方消费者同款姿势；或拉高 publish_hz——两条路都正 |
| 看离目标多远 / 到位判定 | `~/status` 20Hz | **仅 CM active 时存在**（inactive 一帧后静默，不是故障）；**到位用 `arrival`**：POSITION=到了 / SETTLED=到了且停住（逐条下发场景用 SETTLED；converged 已 deprecated） |
| 看为什么被拒 | `~/status.last_result` | IkResult 码表 `ik_result.hpp`；求解器健康用 **`solver_health`** 三档（全机型正确；min_sigma 恒 -1 勿建逻辑） |
| 发笛卡尔目标 | `~/command`（CM）reliable+KeepLast(1) | 每条消息 = 最新终点；四元数入口归一化 |
| 发暂停/恢复 | `~/pause` ——**发布方也必须 transient_local** | 否则**静默失联无任何报错**（QoS 不匹配互相看不见，排查 30 分钟起步） |
| 发关节目标 | `/joint_stream_controller/command` | joint_names **必须列全**命令关节，缺 = 整条拒（无警告） |
| 监控链健康 | `/supervisor/alerts` 50Hz | SystemAlert 板，`state` 四值可直接 switch（OK/DEGRADED/FAULT/UNKNOWN） |
| 记录数据 | `record_verify.sh <task>` | 必录 8 话题 + sidecar 自动；裸 ros2 bag 是废数据 |

## 0. 查看方式速查

```bash
ros2 topic list                    # 全部话题
ros2 topic info <话题>             # 类型 + 发布/订阅者数
ros2 topic echo <话题> --once      # 看一帧内容
ros2 topic hz <话题>               # 实测频率
ros2 service list | grep sim_control
ros2 control list_controllers     # 控制器状态 (active/inactive)
ros2 control list_hardware_components
```

## 1. 话题总表

| 话题 | 类型 | 方向 | 频率 | 作用 |
|---|---|---|---|---|
| `/joint_states` | sensor_msgs/JointState | 链→上层 | 500Hz（控制频率） | 关节空间**单一事实源** |
| `/joint_stream_controller/command` | unistackbot_interface/JointCommand | 上层→链 | 上层定（点流） | 关节命令入口 |
| `/joint_stream_controller/applied_command` | unistackbot_interface/JointCommand | 链→上层 | 全率（=控制频率；`observe_applied_command` 默认关） | 实际下发命令观测（跟踪账） |
| `/cartesian_motion_controller/command` | geometry_msgs/PoseStamped | 上层→链 | 上层定 | 笛卡尔命令入口（base 系） |
| `/cartesian_motion_controller/status` | unistackbot_interface/CartesianMotionStatus | 链→上层 | 20Hz，仅 CM active | CM 健康/误差遥测 |
| `/cartesian_motion_controller/pause` | unistackbot_interface/CartesianControl | 上层→链 | 事件 | TRACKING/HOLD 运动门控（语义见 §2 ~/pause） |
| `/ee_state` | geometry_msgs/PoseStamped | 链→上层 | 50Hz（`publish_hz` 可调） | EE 位姿独立反馈流 |
| `/supervisor/alerts` | `unistackbot_interface/SystemAlert` | 链→上层 | 50Hz | 统一错误状态板（三域+state；详见 §2） |
| `/tf` `/tf_static` | tf2_msgs/TFMessage | 链→上层 | 随 /joint_states | 全身位姿树（RSP 发布） |
| `/robot_description` | std_msgs/String | — | latched | URDF 全文 |
| `/clock`（仅 gz/mujoco） | rosgraph_msgs/Clock | 仿真器→全链 | 仿真步频 | 仿真时间源 |
| `/stats`（仅 gz） | ros_gz_interfaces/WorldStatistics | 仿真器→上层 | ~1Hz | RTF / 暂停态 |
| 基础设施 | 见 §3 | — | — | transition_event / parameter_events / rosout / dynamic_joint_states |

## 2. 核心话题字段详解

### 命令语义总则（~/command 与 ~/command 共用；2026-10-12 P0 契约批定案）

**点流**：每条消息 = 一条"最新终点"意图，**覆盖前一目标**（latest-wins）。无队列、
无 goal/结果握手。消息在单个回调线程串行处理——旧消息的计算结果不可能晚于新消息
生效；回调忙期间到达的中间消息被 QoS 历史丢弃（reliable + KeepLast(1)），回调空闲
后直接处理最新一条。

**最新目标的采纳规则（有效性）**：
- JS（~/command）：joint_names 必须列全命令关节（缺 → 整条拒绝）；NaN → 整条拒绝、
  保持原目标；限位越界 → clamp 进限位后采纳。
- CM（~/command）：frame_id 非 base 系 → 丢弃（每帧名单次警告）；四元数入口归一化
  （零向量丢弃）；超臂展几何预检 → 诚实 UNREACHABLE（不烧求解预算、不覆盖已采纳
  目标）；**求解成功才更新跟踪目标**——失败保持上一有效终点（连续性根）。

**断流（2026-10-10 统一裁决，两通道同款）**：最后有效目标**无限期保留**——控制器
无超时、无 stale 态，不感知"流"的存在；RT 每拍照常下发，臂收敛到最后目标保持。
长静默后恢复的大跳变与任何大跳变同源，由通用超限防线兜底（OTG v/a/j 界 + 目标
跳变界 + 步长饱和 + 限位 clamp）。**何时才算通信故障**不由命令通道判定：
/joint_states >1s 无帧 = ERROR、CM status 在 active 之后 >2s 静默 = WARN、
/ee_state >1s 无帧 = WARN（三者皆 supervisor 裁定）；真机总线真死由驱动器看门狗
兜底（安全终点）。重激活丢弃陈旧目标（seq 水位预读）。

**频率解耦**：上层命令频率与控制频率（update_rate，yaml `/**` 单一事实源）完全
解耦。上层 10-100Hz 点流 → JS ruckig 档做率失配填充，hold 档在满速流（命令率=
控制率）下退化为透传（ruckig 档始终 C2 整形，满速下贴目标走）。命令频率不影响
控制周期，控制周期不假设命令频率。CM 的 IK 预算可选 message_rate 自适应档
（0.8 × 消息间隔滑窗中位数）——高频轨迹流自动收紧预算。

### /joint_states（消费必读：name 与数值按索引对应，但**顺序无保证 → 按名对齐**）

| 字段 | 含义 |
|---|---|
| name[] | 关节名（乱序坑：消费必须按名索引，勿按位置） |
| position[] | 关节角 [rad] |
| velocity[] | 关节速度 [rad/s] |
| effort[] | 仿真链恒 0（真机才是真实力矩） |

### ~/command（JointCommand——点流，共用语义见本节上方总则）

| 字段 | 含义 |
|---|---|
| mode | 0=NONE 1=CSP 2=CSV 3=CST 4=MIT（实现状态见下表） |
| joint_names[] | 关节名序，**必须列全命令关节**（缺 → 拒绝） |
| position[] | 目标关节角 [rad]（CSP 主载荷） |
| velocity[] / effort[] | 前馈（CSP 下可 0） |
| kp[] / kd[] | MIT 增益（真机批次启用） |

**控制模式实现状态（三态，2026-10-12 立）**：

| 模式 | 状态 | 说明 |
|---|---|---|
| CSP | **已实现** | 仿真链唯一消费的模式 |
| CSV / CST / MIT | **已定义未实现** | 消息字段已冻结；JS 现实现丢弃 + 计数 + WARN，真机 MIT 硬件接口到位后启用 |
| 后端能力 | 仿真后端对 CSV/CST/MIT **明确不支持** | URDF 只声明 position 命令接口，物理无处可写——拒绝是能力诚实，不是缺陷 |

**插值档（interpolation 参数；采纳/推进分离——消息只采纳目标，推进每控制周期）**：

| 档 | 执行行为 | 运动约束 |
|---|---|---|
| `hold` | 每拍朝已采纳目标步进 min(剩余距离, vmax/hz) | 速度受限（一阶）；满速流下自动退化透传 |
| `ruckig`（**链上默认**；2026-09-23 A/B 定音：同轨迹 dv/拍 平滑 42-100×） | 每拍从当前状态重规划 C2 至最新目标（OtgStream = 全工程平滑唯一入口） | v/a/j 全受限：vmax=URDF、a/j=per-robot yaml 机型资产；目标跳变界 max_target_jump=0.3 rad |

路径插值永不在本控制器——形状轴归上层/MoveIt；两档都只做"点流整形"。

### ~/command（PoseStamped，base 系；点流/断流/频率语义见总则）

| 字段 | 含义 |
|---|---|
| header.frame_id | 应为目标系（base），非本帧丢弃（每帧名单次警告） |
| pose.position | xyz [m] |
| pose.orientation | 四元数（入口自动归一化，零向量丢弃） |

求解语义：每条消息一次 IK（种子 = 实测关节——物理链下垂按消息率闭环补偿）；
**求解成功才更新跟踪目标**，失败/不可达保持上一有效终点（诚实拒绝，结果码经
~/status 透传，不污染链路状态）。

### ~/pause（CartesianControl——事件通道）

| 字段 | 含义 |
|---|---|
| mode | TRACKING=恢复跟踪 / HOLD=冻结推进 |

**QoS 匹配警告（发布方责任，2026-10-13 升格）**：本通道 transient_local——
**上层发布器必须同为 transient_local**（`rclcpp::QoS(1).transient_local()` /
`QoSProfile(durability=TRANSIENT_LOCAL)`），否则**静默失联无任何报错**
（QoS 不匹配时双方互相看不见对方，发出去像进了黑洞，排查 30 分钟起步）。
链上消费者（teleop/demo）已对齐；新接入方第一件事是核对这个 QoS。

**HOLD 语义（2026-10-12 裁决定案：运动推进门控，非目标过滤器）**：
进入 HOLD 后，CM **停止向外部目标推进**——OTG 每拍以当前命令位置为终点重规划，
运动中的臂 C2 平滑收敛停在原地（非瞬时锁死）；但**继续接收、求解并采纳**符合规则的
新目标（goal_ 照常更新）。解除 HOLD 后从当前状态重规划 C2，追**最新**有效目标。
"冻结期间连目标都不采纳"如未来需要，定义为另一种显式策略，不改 HOLD 既有含义。
消费者注意：遥操作 HOLD 乒乓（Space 键）依赖此语义——按住挡运动、松手追最新。
**安全定位（勿误用）**：HOLD 是常规运动原语（= ABB StopMove 层；口语=软停止，
正式=**IEC 停止类别 2 形态的功能停止**：受控减速+保持使能+即时恢复），**不是急停**——
与命令同链同故障域，无延迟保证，不占 Cat.2 的安全等级。安全终点在软件栈外：
总线级停=真机 quick_stop（停机帧+看门狗位），急停（Cat.0/1）=驱动器看门狗/硬件安全回路。
安全逻辑不得建立在 ~/pause 上。

### ~/status（CartesianMotionStatus——CM 激活才有；inactive 只发一帧 mode=0 后静默）

| 字段 | 含义 |
|---|---|
| mode | 0=INACTIVE 1=IDLE 2=TRACKING 3=HOLD 4=DEGRADED（连续求解失败，保持上一命令） |
| converged | 误差进容差（容差=控制器参数 converge_pos_tol/rot_tol，默认 1mm/0.01rad，per-robot 可调） |
| position_error / rotation_error | 位置 [m] / 姿态 [rad] 误差（rotation_error 符号无关） |
| min_sigma | 雅可比最小奇异值；**-1 = 求解器不提供**（trac_ik 路径恒 -1，仅 dls_ik 有效——数值解机型默认 trac_ik） |
| timed_out / last_result | 求解预算耗尽 / IkResult 结果码 |
| current_pose / target_pose | CM 视角 FK 当前位姿 / 当前采纳目标 |

**双 FK 位姿源的权威声明（2026-10-12 裁决）**：本消息 current_pose 与 `/ee_state`
是同一道确定性 FK（同 /joint_states、同 URDF），数值一致；但**位姿事实源以
`/ee_state` 为准**（常驻、与控制器无关），current_pose 是 CM 的误差计算基准
（500Hz 粒度更细、仅 CM active 时存在）——**不作为位姿事实源消费**。CM 与
EEB 不互抄数据（确定性同源，无抄数必要；控制器间订阅是 ros2_control 反模式）。
四元数符号未规范化警示随 msg 定义（自算姿态差先 abs(dot)）。

### /ee_state（PoseStamped，base 系）——**位姿事实源**

同 ~/command 结构，但方向相反：FK(当前关节) 的末端位姿只读反馈流，与哪个控制器 active 无关。
**反馈选型指引**：看臂在哪 → 本话题（50Hz 常驻）；看离目标多远/求解器健康 → ~/status（20Hz，仅 CM active）；看关节物理真值 → /joint_states（500Hz）。
**姿态载体定案 = 单位四元数，永久**（2026-10-12 裁决）：与 ROS 机面生态主流一致（geometry_msgs 无欧拉类型，RViz/tf/MoveIt 零适配）；其他表示（欧拉/轴角等人读形态）如需要走**并行加法**（新话题或新字段），永不动四元数载体。消费者跨生态注意字段序差异（geometry_msgs 按字段名访问无序陷阱；Isaac/MuJoCo 为 wxyz）。

### /supervisor/alerts（SystemAlert——统一错误状态板，50Hz）

**原则**：错误处理在内部，状态在此暴露——单向观察窗不接指令。supervisor 死亡 =
话题静默（订阅方 20ms 级发现）。纯数值契约：人话一律查 `err_code_message(code)`
（`error_codes.hpp` 码表，加码不破 msg）。

三错误域（= 三个恢复语义世界；**变长平行数组，零写死容量**——加关节/总线/组件源
msg 永不改）：

| 域 | locations 内容 | 恢复语义 |
|---|---|---|
| `device_*` | 关节名（per-joint，仿真链空） | 清故障握手+重新使能（statemachine 轴） |
| `bus_*` | 总线端点（仿真链空） | 自愈/重同步（bus/serial 轴）；驱动器看门狗兜底 |
| `system_*` | 组件名（仿真链恒 cm/joint_states/ee_state 三条） | 拒绝/降级/重规划（控制器轴） |

`code`：0=正常；0xFFFE=源静默；其余见码表（js 断流=FAULT 级，cm 降级/静默、ee 断流=
DEGRADED 级）。`count` = 异常累计拍数（趋势差分用）。

`state`（唯一需上层 switch 的量）：UNKNOWN(0 源数据不足) / OK(1) / DEGRADED(2 可用
但降质) / FAULT(3 不可派活)。聚合 = worst-of(err_code_severity)，唯一裁决处 =
supervisor；任一 DEVICE 非零码或源静默 → FAULT（宁误停不误跑）。

### ~/applied_command（JointCommand——JS 观测面）

当拍**插值后实际写硬件接口**的命令全率回读。三层对账语义：
与 ~/command 的差 = 插值整形量；与 /joint_states 的差 = 执行层跟踪误差
（ruckig 滞后与伺服滞后的归因分离账）。
参数 `observe_applied_command` 默认关（RT 发布带宽考虑），stationlite yaml 开；
录制规范 v1 列为必录话题。QoS reliable + KeepLast(50)。

## 3. 基础设施话题（知道即可，一般不消费）

| 话题 | 说明 |
|---|---|
| `*/transition_event` | 每个控制器一个，生命周期迁移事件（lifecycle_msgs/TransitionEvent） |
| `/parameter_events` | ROS 参数变更事件 |
| `/rosout` | 日志聚合 |
| `/dynamic_joint_states` | controller_manager 硬件接口状态（值多为 0，勿与 /joint_states 混淆） |

## 4. 服务

| 服务 | 类型 | 语义 |
|---|---|---|
| `/sim_control/reset` | std_srvs/Trigger | 回确定性初态（**需 pause 下用**； gz 拒绝——Fortress 有毒；mujoco 原生支持） |
| `/sim_control/pause` / `resume` | std_srvs/Trigger | 暂停/恢复仿真推进 |
| `/sim_control/step` | std_srvs/Trigger | 暂停态单步推进 |
| `/sim_control/set_joint_state` | unistackbot_interface/SetJointState | 瞬移关节（仅 mock；**需 pause 下用**——激活控制器每拍覆盖瞬移；gz/mujoco 拒绝） |
| `/controller_manager/*` | — | 用 CLI 封装：`ros2 control list_controllers` / `switch_controllers` |

`success=true` 只代表**已接受**，不代表执行完成。mujoco 链另有 `/mujoco_ros2_control_node/apply_external_wrench`（扰动注入，gz 没有）。

### 操作态通道 `/sim_control/paused`（std_msgs/Bool @10Hz 心跳，2026-10-15）

暂停的**可观测出口**：生产者=各链 `/sim_control` 适配器（gz 译 `/stats` 真值含 GUI 暂停；mujoco 命令镜像——窗口 GUI 暂停盲区 off-contract）；mock 不发（缺席=默认运行）。消费者=supervisor：暂停期间 `/supervisor/alerts` 的 state=**PAUSED(4)**、静默类裁决挂起——"故意停"不是故障。心跳断（适配器死）自动回落正常裁决（宁误报不漏报）。契约与挂账（通用操作态框架待真机软停需求）见 `sim_control_contract.hpp`。

## 5. 控制器状态（链健康的"仪表盘"）

```bash
ros2 control list_controllers
# 预期态: joint_state_broadcaster active / joint_stream_controller active
#         cartesian_motion_controller inactive(注册待命) / ee_state_broadcaster active
ros2 control switch_controllers --deactivate joint_stream_controller --activate cartesian_motion_controller
ros2 control switch_controllers --deactivate cartesian_motion_controller --activate joint_stream_controller
```

## 6. 链间差异速查

| 项 | mock | gz | mujoco |
|---|---|---|---|
| /clock | 无（墙钟） | ✓ | ✓ |
| /stats RTF | — | ✓ | — |
| reset / set_joint_state | 全支持 | 拒绝 | reset 原生 / set_joint_state 拒绝 |
| 扰动注入 | — | — | apply_external_wrench |
| JS 命令关节集（piper） | 9（含手指直令） | 7 | 7（手指 equality 跟随） |

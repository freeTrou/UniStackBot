# 机型控制器 YAML 配置规范（框架级，全机型通用）

> **性质**：normative——本文件与实现冲突时，要么改实现要么改本文件，不许默默分歧。
> 依据：六机型（piper / xarm7 / stationlite / r1_7a / r1_pro / mos_p20）实证。
> 位置：`docs/guides/`（与 cpp_style_guide.md 同级）。

---

## 0. 配置面全景（一台机器的全部配置文件）

```
<robot>_description/                    ← 机型资产包（产品域 or 框架域）
├── urdf/
│   └── <robot>.urdf.xacro            ← 唯一入口（物理 URDF + ros2_control + backend 参数）
├── config/
│   ├── <robot>_controllers.yaml      ← 控制器配置（本文档主题）
│   └── <robot>_acceptance.yaml       ← 验收指纹（verify_robot.sh 消费）
└── meshes/
```

框架侧零机型硬编码（launch 按命名约定自动发现）。

---

## 1. `/**` 全局频率通配节

```yaml
/**:
  ros__parameters:
    update_rate: 500
```

| 字段 | 类型 | 必填 | 取值 | 说明 |
|---|---|---|---|---|
| `update_rate` | int | ✅ | 500 / 1000 | 所有控制器节点的更新频率（Hz），也是总线/CM 的控制频率。 |

**⚠️ 必须有此节**。缺它 CM 落"激活窗口中位数"兜底，中位数被激活瞬间 µs period 污染 → OTG 时间尺度错 → 臂以 ~1000× 龟速爬（r1_pro 2026-10-15 实锤）。

**与 controller_manager.update_rate 的关系**：两处必须同值。`/**` 是给控制器节点的，`controller_manager` 节是给 CM 本身的。冗余但 ros2_control 架构要求。

---

## 2. controller_manager 段

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    thread_priority: 80
    cpu_affinity: 1
    lock_memory: true
    # ---- 以下每个要加载的控制器各一条 ----
    joint_state_broadcaster:
      type: unistackbot_controller/UniJointStateBroadcaster
    js_arms:
      type: unistackbot_controller/JointStreamController
    # ...
```

### 2.1 RT 线程调优

| 字段 | 类型 | 必填 | 默认 | 说明 |
|---|---|---|---|---|
| `update_rate` | int | ✅ | — | 与 `/**` 同值。 |
| `thread_priority` | int | 推荐 | 80 | RT 线程 SCHED_FIFO 优先级（0-99）。调度阶梯：EtherCAT 90 > CiA402 85 > **CM 80** > 总线预留 75-85。一般不改。 |
| `cpu_affinity` | int | ✅ | — | RT 线程钉核编号。**因部署机器而异**：x86 隔离核（isolcpus=1,2）→ `1`；ARM big.LITTLE（RK3576）→ 大核簇核号（`/sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_max_freq` 实测，通常 4-7）。 |
| `lock_memory` | bool | 推荐 | true | `mlockall(MCL_CURRENT\|MCL_FUTURE)` 锁内存。**三档纪律**：`true` = 启用（需 `limits.conf` 配 memlock unlimited）；`false` = 保险丝（失败仅 WARN）；**勿造有限大值毒区**（memlock 部分成功 → MCL_FUTURE 武装 → 后续 malloc 全 EAGAIN）。 |

### 2.2 控制器 type 注册表

每个要加载的控制器**必须**在此声明名称 + type，否则 spawner 找不到。

| type | 功能 | 常驻状态 | 说明 |
|---|---|---|---|
| `unistackbot_controller/UniJointStateBroadcaster` | 关节状态广播 | **active** | 自研 JSB（URDF `<ros2_control>` 声明序契约）。实例名固定 `joint_state_broadcaster`。 |
| `unistackbot_controller/JointStreamController` | 关节级 topic 流式（CSP） | **active** | 每实例一个 yaml 段。多实例编组见 §3。 |
| `unistackbot_controller/CartesianMotionController` | 单臂笛卡尔流式（IK 伺服） | **inactive** | switch 切换使用。实例名固定 `cartesian_motion_controller`。 |
| `unistackbot_controller/EeStateBroadcaster` | 末端位姿 FK 反馈 | **active** | 只读，与 JS/CM 共存。实例名固定 `ee_state_broadcaster`。 |
| `unistackbot_controller/WholeBodyController` | 双臂/全身解算壳（worker 域） | **inactive** | switch 切换使用。认领 = js_arms 同集（互斥）。 |
| `joint_trajectory_controller/JointTrajectoryController` | 关节轨迹（MoveIt 执行） | **inactive** | Humble 需 spawner `-t` 传 type。实例名固定 `joint_trajectory_controller`。 |
| `unistackbot_controller/BaseVelocityController` | 底盘速度控制 | active | 仅轮式机型。实例名固定 `base_velocity_controller`。 |

---

## 3. JointStreamController（JS）编组段

**多实例编组**：每个 JS 实例一个 yaml 段。实例名 = 段名 = launch `js_controllers` 参数值。

### 命名约定

| 实例名 | 覆盖 | 适用机型 |
|---|---|---|
| `joint_stream_controller` | 全部关节（默认单实例） | piper / xarm7 / r1_7a / arm7_test |
| `js_arms` | 双臂 14 关节（一条消息原子驱动） | r1_pro / mos_p20 / stationlite |
| `js_head` | 头部 2 关节 | mos_p20 |
| `js_torso` | 升降/腰 1-4 关节 | r1_pro / mos_p20 |

### 字段

```yaml
<实例名>:
  ros__parameters:
    joints: [<关节名显式列表>]
    interpolation: ruckig
    observe_applied_command: false
```

| 字段 | 类型 | 必填 | 默认 | 说明 |
|---|---|---|---|---|
| `joints` | string[] | ✅ | — | 该实例认领的关节名显式列表。**必须列全**且与 URDF `<ros2_control>` 声明一致（子集）。遗漏 = 该关节不受控；未知/重复 = configure 拒。多实例集合必须互斥。空列表 = URDF 全合并（旧行为，不推荐）。 |
| `interpolation` | string | 推荐 | "hold" | `ruckig` = C2 平滑点流整形（**生产默认**）；`hold` = 步长饱和逼近（满速透传语义）。 |
| `observe_applied_command` | bool | — | false | 发布 `~/applied_command`（全率插值后实际下发）。**录制规范必录**——录制链开 true。 |

---

## 4. CartesianMotionController（CM）段

```yaml
cartesian_motion_controller:
  ros__parameters:
    base_link: <基座link>
    tip_link: <末端link>
    ik_solver: <插件名>
    ik_timeout_ms: 50.0
    ik_timeout_mode: message_rate
    ik_rate_share: 0.8
    max_acceleration: 5.0
    max_jerk: 20.0
```

| 字段 | 类型 | 必填 | 默认 | 说明 |
|---|---|---|---|---|
| `base_link` | string | ✅ | — | IK/FK 基座 link 名。从 URDF 树根取。 |
| `tip_link` | string | ✅ | — | IK/FK 末端 link 名。从 URDF 链尾取。多臂机型配左臂。 |
| `ik_solver` | string | ✅ | **无默认** | IK 求解器插件名（pluginlib 运行期装载）。选错 = init 拒并列出可选。 |
| `ik_timeout_ms` | float | ✅ | — | 单次求解墙钟预算（ms）。典型 50.0（保守）或 5.0（高频）。 |
| `ik_timeout_mode` | string | — | "explicit" | `explicit` = 固定预算；`message_rate` = 自适应（按消息间隔 × rate_share 动态）。高频轨迹流推荐 `message_rate`。 |
| `ik_rate_share` | float | — | 0.8 | `message_rate` 模式的时间份额（0-1）。 |
| `max_acceleration` | float | ✅ | — | OTG 加速度限值（rad/s²）。**URDF 无此项**——必须从机型规格取。缺 = configure 拒。 |
| `max_jerk` | float | ✅ | — | OTG 加加速度限值（rad/s³）。同上。 |

### ik_solver 可选值

| 值 | 类型 | 适用 | 说明 |
|---|---|---|---|
| `trac_ik` | 数值 | 7-DoF 泛用 | SQP+DLS 双线程竞争。回调线程专用（禁 RT 环）。 |
| `dls` | 数值 | RT 场景 / 调试 | 零依赖 RT 合规 fallback。 |
| `analytic_piper` | 解析 | piper 球腕 6 轴 | IKFast 闭式解，需结构指纹通过。 |
| `analytic_stationlite` | 解析 | stationlite | 三平行轴 Pieper 闭式解。 |

---

## 5. EeStateBroadcaster（EEB）段

```yaml
ee_state_broadcaster:
  ros__parameters:
    publish_hz: 50.0
    base_link: <基座link>
    tip_link: <末端link>
```

| 字段 | 类型 | 必填 | 默认 | 说明 |
|---|---|---|---|---|
| `publish_hz` | float | — | 0 | `/ee_state` 发布频率（Hz）。0 = 全率（录制/跟踪账用）；50.0 = 日常节流。 |
| `base_link` | string | ✅ | — | FK 基座。与 CM 同源。 |
| `tip_link` | string | ✅ | — | FK 末端。与 CM 同源。 |

---

## 6. JointTrajectoryController（JTC）段

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [<同 js_arms 关节集>]
    state_interfaces: [position, velocity]
    command_interfaces: [position]
```

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `joints` | string[] | ✅ | JTC 管的关节。通常 = js_arms 同集。 |
| `state_interfaces` | string[] | ✅ | Humble 要求显式。标准 `[position, velocity]`。 |
| `command_interfaces` | string[] | ✅ | Humble 要求显式。仿真 `[position]`。 |

---

## 7. WholeBodyController（WB）段

```yaml
whole_body_controller:
  ros__parameters:
    solver: <插件名>
    joints: [<同 js_arms 关节集>]
    end_frames: [<末端帧集合>]
    base_frame: <参考系link>
    dt_s: 0.005
    max_delta_rad: 0.01
```

| 字段 | 类型 | 必填 | 默认 | 说明 |
|---|---|---|---|---|
| `solver` | string | ✅ | **无默认** | WholeBodySolver 插件名。算法团队 HQP 到位后改此行即换。 |
| `joints` | string[] | ✅ | — | 认领关节集。**必须与 js_arms 一致**（互斥，switch 切换）。 |
| `end_frames` | string[] | ✅ | — | 末端帧集合（双臂=2，单臂=1）。从 URDF link 取。顺序 = `~/command` PoseArray 位姿序。 |
| `base_frame` | string | ✅ | — | 所有目标的参考系。从 URDF 树根取。 |
| `dt_s` | float | ✅ | — | worker 线程节拍（秒）。**必须与实际频率一致**（0.005=200Hz）。 |
| `max_delta_rad` | float | — | 0.01 | 安全落地步长上限（rad/拍）。 |

---

## 8. URDF xacro 侧要求（与 yaml 联动）

每关节在 `<ros2_control>` 块中**必须**声明：

```xml
<joint name="<关节名>">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
  <param name="min"><下限></param>       <!-- JS 限位 clamp 用 -->
  <param name="max"><上限></param>       <!-- JS 限位 clamp 用 -->
  <param name="max_velocity"><速度></param>  <!-- JS OTG 步长计算用 -->
</joint>
```

**⚠️ 缺 `max_velocity` = JS 的 OTG 不动**（P20 Phase 3 实锤——UDP 后端缺了此参数，关节停在零位不响应目标）。

### backend 切换（xacro 参数）

```xml
<xacro:arg name="backend" default="kinematic"/>
```

| backend 值 | 硬件插件 | 用途 |
|---|---|---|
| `kinematic` | SimControlHardware | mock 链（日常开发，默认） |
| `udp_board` | UdpBoardSystem | FakeBoard（仿真）/ 真机（改 board_ip） |

launch 传参：
```bash
ros2 launch unistackbot_bringup control.launch.py robot:=<robot> backend:=udp_board board_ip:=<板IP>
```

---

## 9. 新机型配置检查清单

| # | 要配的 | 从哪取值 | 忘了会怎样 |
|---|---|---|---|
| 1 | `/**.update_rate` | 总线频率（500/1000） | CM 龟速爬 |
| 2 | `controller_manager.cpu_affinity` | 部署机器核拓扑 | RT 线程钉错核 |
| 3 | `js_*.joints` 列表 | URDF `<ros2_control>` 声明序 | 关节不受控 / configure 拒 |
| 4 | `cm.base_link / tip_link` | URDF 树根 / 链尾 | init 拒 |
| 5 | `cm.ik_solver` | 机型 IK 决策卡 | init 拒（列可选） |
| 6 | `cm.max_acceleration / max_jerk` | 厂商规格 / 实测 | configure 拒 |
| 7 | `whole_body.solver` | 算法团队 | configure 拒 |
| 8 | `whole_body.dt_s` | 与总线频率对齐 | 积分步长错 |
| 9 | URDF 每关节 `min / max / max_velocity` | 厂商规格 | **JS OTG 不动** |
| 10 | `<robot>_acceptance.yaml` 指纹 | URDF 展开实测 | verify 拒启动 |

---

## 10. 机型配置差异速查

| 机型 | JS 编组 | ik_solver | 特别件 |
|---|---|---|---|
| piper | 单 JS | analytic_piper | — |
| xarm7 | 单 JS | trac_ik | — |
| r1_7a | 单 JS | trac_ik | — |
| stationlite | 单 JS | analytic_stationlite | 双臂但 CM 只左臂 |
| r1_pro | js_arms + js_torso + BaseVelocity | trac_ik | 四件式首例 |
| mos_p20 | js_arms + js_head + js_torso + WB + CM | trac_ik | 首个 WB + udp_board |
| arm7_test | 单 JS | trac_ik | — |

---

## 11. 算法域包结构（2026-11-01 定稿）

> 详见 `docs/architecture/algorithm_architecture.md`（normative）。

| 包 | 内容 | ROS | 维护者 |
|---|---|---|---|
| `unistackbot_algorithm` | SDK: 契约头 + urdf_fk + base_kinematics | ✗ 零 | 框架组 + 算法组 |
| `unistackbot_algorithm_solvers` | 参考实现 + plugin 注册 | ✓ | 臂算法组 |
| 外部仓 × N | 算法团队的实现 | ✓ | 各算法团队 |

**外部仓接入**: depend `unistackbot_algorithm` → 实现契约 → pluginlib 注册 → yaml 改一行。我们的仓零改动。


---

## 12. 拆分完成状态（2026-11-01 更新）

两包拆分已执行完毕，全机型回归零差异：
- Pkg 1 `unistackbot_algorithm`（SDK）：零 ROS，ARM 交叉编译 ✓
- Pkg 2 `unistackbot_algorithm_solvers`（参考实现）：pluginlib 插件库，TRAC-IK 可选编译
- 控制器 `<depend>` 两者
- 外部仓 depend Pkg 1 即可接入

调试工具（fk_tool/ik_tool/ik_demo_node）已移入 `BUILD_DEBUG_TOOLS=OFF` 条件编译（默认跳过，需开时 `-DBUILD_DEBUG_TOOLS=ON`）。

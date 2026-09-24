# 链上接口词典（以 piper mock 链为基线；三链同源，差异见 §6）

> 运行面可观测的一切：话题名 / 类型 / 字段含义 / 频率 / 查看方式 / 服务 / 控制器状态。
> 定义先行：本词典是接口的规范描述，实现与词典冲突 = 实现 bug。
> 字段定义以 msg 文件为权威（`unistackbot_interface/msg/`）。

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
| `/cartesian_motion_controller/target` | geometry_msgs/PoseStamped | 上层→链 | 上层定 | 笛卡尔命令入口（base 系） |
| `/cartesian_motion_controller/status` | unistackbot_interface/CartesianMotionStatus | 链→上层 | 20Hz，仅 CM active | CM 健康/误差遥测 |
| `/cartesian_motion_controller/control` | unistackbot_interface/CartesianControl | 上层→链 | 事件 | TRACKING/HOLD 冻结指令 |
| `/ee_state_broadcaster/ee_state` | geometry_msgs/PoseStamped | 链→上层 | 50Hz（`publish_hz` 可调） | EE 位姿独立反馈流 |
| `/supervisor/alerts` | diagnostic_msgs/DiagnosticArray | 链→上层 | 1Hz | 编排层诊断告警 |
| `/tf` `/tf_static` | tf2_msgs/TFMessage | 链→上层 | 随 /joint_states | 全身位姿树（RSP 发布） |
| `/robot_description` | std_msgs/String | — | latched | URDF 全文 |
| `/clock`（仅 gz/mujoco） | rosgraph_msgs/Clock | 仿真器→全链 | 仿真步频 | 仿真时间源 |
| `/stats`（仅 gz） | ros_gz_interfaces/WorldStatistics | 仿真器→上层 | ~1Hz | RTF / 暂停态 |
| 基础设施 | 见 §3 | — | — | transition_event / parameter_events / rosout / dynamic_joint_states |

## 2. 核心话题字段详解

### /joint_states（消费必读：name 与数值按索引对应，但**顺序无保证 → 按名对齐**）

| 字段 | 含义 |
|---|---|
| name[] | 关节名（乱序坑：消费必须按名索引，勿按位置） |
| position[] | 关节角 [rad] |
| velocity[] | 关节速度 [rad/s] |
| effort[] | 仿真链恒 0（真机才是真实力矩） |

### ~/command（JointCommand——点流：每条消息只是"最新目标"）

| 字段 | 含义 |
|---|---|
| mode | 0=NONE 1=CSP 2=CSV 3=CST 4=MIT；**仿真链只消费 CSP**，其余计数丢弃 |
| joint_names[] | 关节名序，**必须列全命令关节**（缺 → 拒绝） |
| position[] | 目标关节角 [rad]（CSP 主载荷） |
| velocity[] / effort[] | 前馈（CSP 下可 0） |
| kp[] / kd[] | MIT 增益（仿真链不用，真机批次启用） |

### ~/target（PoseStamped，base 系）

| 字段 | 含义 |
|---|---|
| header.frame_id | 应为目标系（base） |
| pose.position | xyz [m] |
| pose.orientation | 四元数（入口自动归一化） |

### ~/status（CartesianMotionStatus——CM 激活才有；inactive 只发一帧 mode=0 后静默）

| 字段 | 含义 |
|---|---|
| mode | 0=INACTIVE 1=IDLE 2=TRACKING 3=HOLD 4=DEGRADED（连续求解失败，保持上一命令） |
| converged | 误差进容差 |
| position_error / rotation_error | 位置 [m] / 姿态 [rad] 误差 |
| min_sigma | 雅可比最小奇异值（接近 0 = 接近奇异） |
| stream_stale | ~/target 断流标志（受控减速中/已停） |
| timed_out / last_result | 求解预算耗尽 / IkResult 结果码 |
| current_pose / target_pose | FK 当前位姿 / 当前采纳目标 |

### ~/ee_state（PoseStamped，base 系）

同 ~/target 结构，但方向相反：FK(当前关节) 的末端位姿只读反馈流，与哪个控制器 active 无关。旋转表示决策挂起，先四元数载体。

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
| `/sim_control/reset` | std_srvs/Trigger | 回确定性初态（**需 pause 下用**；gz 拒绝——Fortress 有毒；mujoco 原生支持） |
| `/sim_control/pause` / `resume` | std_srvs/Trigger | 暂停/恢复仿真推进 |
| `/sim_control/step` | std_srvs/Trigger | 暂停态单步推进 |
| `/sim_control/set_joint_state` | unistackbot_interface/SetJointState | 瞬移关节（仅 mock；**需 pause 下用**——激活控制器每拍覆盖瞬移；gz/mujoco 拒绝） |
| `/controller_manager/*` | — | 用 CLI 封装：`ros2 control list_controllers` / `switch_controllers` |

`success=true` 只代表**已接受**，不代表执行完成。mujoco 链另有 `/mujoco_ros2_control_node/apply_external_wrench`（扰动注入，gz 没有）。

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

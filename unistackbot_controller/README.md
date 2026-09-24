# unistackbot_controller

控制器集成包：三个自研控制器插件 + ROS 工具节点。算法库住 `unistackbot_algorithm`（2026-09-17 拆分），本包只消费类型。

## CartesianMotionController（CM, P1.5）

IK 上环宿主：`cartesian_motion_controller/` 自包含文件夹。双机型三链验收。

**契约**：
- `~/target` PoseStamped（值通道，reliable+KeepLast(1)，base 系）
- `~/control` CartesianControl（事件通道：TRACKING/HOLD，transient_local）
- `~/status` CartesianMotionStatus（误差/min_sigma/结果码/stream_stale，20Hz）
- 关节反馈复用 `/joint_states`，不新增

**机制**：流式 IK 墙钟预算（`update_timeout_ns` 500µs，冷启动 `cold_timeout_ns` 50ms，超限走 worker 低优线程冷启动并回灌）；安全层独立于 IK（NaN 门 → 限位 clamp → 步长饱和）；断流受控减速（0c，默认关——`--once` 单发是合法用法，流式场景 yaml 开 `stale_timeout_ms`）。

**边界**：折叠零位直发远目标需预摆位或等 worker 冷启动；加速度界挂真机前。

## JointStreamController（JS, 2026-09-18）

关节级 topic 流式控制器，取代 JTC action 层：`joint_stream_controller/` 自包含文件夹。

**契约**：`~/command` JointCommand（reliable+KeepLast(1)，点流语义=每条消息是"最新目标"；CSP/CSV/CST/MIT 四模式，仿真链只消费 CSP）。`joint_names` 必须列全命令关节（mujoco 链只列 7 主关节）。

**插值** `interpolation`：`hold`（默认，每周期步长饱和逼近）| `ruckig`（OtgStream 整形；路径插值永不在控制器——形状轴归上层/MoveIt）。

**断流受控减速**（0c）：链上默认开（200/200ms）——hold 档线性刹停 / ruckig 档 OTG 刹停，恢复自动续接。

## EeStateBroadcaster（EE, 2026-09-21）

EE 位姿独立反馈流：`~/ee_state` PoseStamped（base 系，FK(关节状态)，50Hz）。**只读**（零命令接口，CM 切走后反馈不断流），三链第四 spawner。旋转表示决策挂起，先四元数载体。

## OtgGateController（OTG 门, P1, 2026-09-23）

设计 §16.4 路线 A 的壳：**"上层只有终点"的对齐服务** —— 笛卡尔终点 →（回调线程一次
IK, 50ms 预算）→ 关节终点 →（RT 环 OtgStream 每拍整形）→ **总线频率 JointCommand
点流** → JS（hold 档满速退化 = 透传）→ write 层防线。本路径引入关节 C2 平滑与
加速度界（消解 CM"加速度界挂真机前"挂账）。

- 契约: 订阅 `~/target`(PoseStamped, base 系, 点流) + `/joint_states`(实测快照, 非链
  关节回显源); 发布 `/joint_stream_controller/command`(CSP, 500Hz 实测)。**零接口认领**
  （与 JS/CM/EE 全共存, inactive 注册同 CM）
- 线程: IK 在订阅回调线程（RT 环零运动学）; RT = OtgStream(~µs) + RealtimePublisher
  trylock（joint_names 激活期预填, RT 只写 position 数值, 零分配）
- 关节表: 全 position 命令关节（`controller_common/urdf_command_joints.hpp`, 与 JS 同源）;
  链关节 = OTG 输出, 非链关节 = 实测回显（门不发明夹爪运动）
- 参数纪律: `base_link`/`tip_link`/`ik_solver` **必填无默认**（同 CM; dls | analytic_piper）;
  `max_acceleration`/`max_jerk` = OTG 界（机型资产 yaml）; dt = 首拍 16 中位数校准（同 JS/CM）
- 用法: `ros2 run controller_manager spawner otg_gate_controller` → 发
  `/otg_gate_controller/target`。不可达终点 → 求解拒绝 WARN 保持上一目标（诚实失败）
- mock E2E (2026-09-23): +20cm 终点到位误差 0.088mm(账地板), 命令流 500.03Hz,
  不可达 (result=1) 拒绝保持, 回程到位

## 工具节点

- `fk_tool` / `ik_tool`：算法调试 CLI（fk_tool 的 `--base/--tip` **必填无默认**，piper 用 `base_link`/`link6`，xarm7 用 `link_base`/`link7`）
- `ik_demo_node`：RViz 拖动 → 发 CM `~/target`

## 范围

本包承载通用基础设施控制器（OTG ingest gate 等）+ 自研算法控制器 + 插件模板。边界分析见 `docs/architecture/hardware_framework_design.md` §14。

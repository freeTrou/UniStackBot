# unistackbot_controller

控制器集成包：三个自研控制器插件 + ROS 工具节点。算法库住 `unistackbot_algorithm`（2026-09-17 拆分），本包只消费类型。

## CartesianMotionController（CM, P1.5）

IK 上环宿主：`cartesian_motion_controller/` 自包含文件夹。双机型三链验收。

**契约**：
- `~/command` PoseStamped（值通道，reliable+KeepLast(1)，base 系）
- `~/pause` CartesianControl（事件通道：TRACKING/HOLD，transient_local）
- `~/status` CartesianMotionStatus（误差/min_sigma/结果码，20Hz）
- 关节反馈复用 `/joint_states`，不新增

**机制**：安全层独立于 IK（NaN 门 → 限位 clamp → 步长饱和）；断流语义（2026-10-10 统一，JS/CM 同一套）：~/command 停发 = 最后一条消息的最新终点仍是目标，臂收敛到它保持，无专门机制——恢复的大跳变由通用超限防线兜底（OtgStream v/a/j 界 + 步长钳位）。

**边界**：折叠零位直发远目标需预摆位或等 worker 冷启动；加速度界挂真机前。

## JointStreamController（JS, 2026-09-18）

关节级 topic 流式控制器，取代 JTC action 层：`joint_stream_controller/` 自包含文件夹。

**契约**：`~/command` JointCommand（reliable+KeepLast(1)，点流语义=每条消息是"最新目标"；CSP/CSV/CST/MIT 四模式，仿真链只消费 CSP）。`joint_names` 必须列全命令关节（mujoco 链只列 7 主关节）。

**插值** `interpolation`：`hold`（默认，每周期步长饱和逼近）| `ruckig`（OtgStream 整形；路径插值永不在控制器——形状轴归上层/MoveIt）。

**断流语义**（2026-10-10 统一，与 CM 同一套，无参数无机制）：`~/command` 停发 = 最后一帧即当前目标，臂收敛到它保持；恢复的大跳变由通用超限防线兜底（ruckig v/a/j 界 + hold 档步长饱和 + 限位 clamp）。

## EeStateBroadcaster（EE, 2026-09-21）

EE 位姿独立反馈流：`/ee_state` PoseStamped（base 系，FK(关节状态)，50Hz）。**只读**（零命令接口，CM 切走后反馈不断流），三链第四 spawner。旋转表示决策挂起，先四元数载体。

## OtgGateController —— 已退役（2026-09-24 删代码）

OTG 门（2026-09-23 落地）在 §16.6 语义统一裁决后架构并入 CM：**其通路（订阅回调线程
一次 IK → RT 环 OtgStream 输出级）就是 CM 的新实现**，独立控制器不再存在。终点命令
直接发 `/cartesian_motion_controller/command`（CM 默认可打断；停发 = 收敛到最后目标保持，2026-10-10 统一语义）。
决策记录见 `docs/architecture/hardware_framework_design.md` §16.4/§16.6。

## 工具节点

- `fk_tool` / `ik_tool`：算法调试 CLI（fk_tool 的 `--base/--tip` **必填无默认**，piper 用 `base_link`/`link6`，xarm7 用 `link_base`/`link7`）
- `ik_demo_node`：RViz 拖动 → 发 CM `~/command`

## 范围

本包承载通用基础设施控制器（OTG ingest gate 等）+ 自研算法控制器 + 插件模板。边界分析见 `docs/architecture/hardware_framework_design.md` §14。

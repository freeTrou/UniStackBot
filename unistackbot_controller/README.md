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

## 工具节点

- `fk_tool` / `ik_tool`：算法调试 CLI（fk_tool 支持 `--base/--tip`；默认是 xarm7 约定，piper 要 `--base base_link --tip link6`）
- `ik_demo_node`：RViz 拖动 → 发 CM `~/target`

## 范围

本包承载通用基础设施控制器（OTG ingest gate 等）+ 自研算法控制器 + 插件模板。边界分析见 `docs/architecture/hardware_framework_design.md` §14。

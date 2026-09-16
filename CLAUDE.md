# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UniStackBot is a ROS 2 Humble workspace for a general-purpose, multi-morphology real-time robot control framework. It targets fixed-base arms, wheeled bases, quadrupeds, wheeled-arm humanoids, and bipedal humanoids from a single layered architecture. The first concrete robot is the **Piper arm**: URDF/Xacro description + the unified sim-control layer (`unistackbot_sim_control`, ros2_control hardware plugin with pluggable backends) + Gazebo integration. `unistackbot_controller` and `unistackbot_hardware` (real driver) are still skeletons (empty `placeholder.cpp`).

## Build & Run

Build from the **workspace root** (`UniStackBot_ws/`), not from this repo directory:

```bash
cd <path-to>/UniStackBot_ws
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package: `colcon build --symlink-install --packages-select <pkg>`

Tests (per package, via `ament_lint_auto` — lint checks only, no unit tests yet):
```bash
colcon test --packages-select <package>
colcon test-result --verbose
```

`unistackbot_common` 组件**不进 colcon**——在组件目录下用 g++ 直接编译运行测试（规范命令见 `unistackbot_common/ulog/README.md`，其余组件同型）:
```bash
cd unistackbot_common/ulog
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_ulog.cpp -o test_ulog && ./test_ulog
# TSAN 变体: -fsanitize=thread，且 TSAN_OPTIONS="suppressions=tsan_suppressions.txt" setarch $(uname -m) -R ./test_tsan
```
测试二进制（test_ulog/bench_ulog 等）已在 .gitignore，勿提交。

Three launch entry points, all currently Piper-specific:

```bash
# 1. Visualization only (RViz + joint_state_publisher_gui; ros2_control off by default)
ros2 launch unistackbot_description display.launch.py
#    args: model:=<xacro> use_gripper use_ros2_control use_world gui rviz

# 2. Mock control chain: standalone controller_manager + SimControlHardware (kinematic backend), no simulator
ros2 launch unistackbot_bringup control.launch.py robot:=piper   # robot:=xarm7 等; use_rviz:=true|false

# 3. Gazebo Sim / Fortress (current chain): controller manager inside gz_ros2_control
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper        # gui:=true|false, use_rviz:=true 可选
```

All control/sim launches take a **required** `robot:=<机型>` argument (resolves `unistackbot_description/arms/<robot>/` + `unistackbot_bringup/config/<robot>_controllers.yaml`; unknown robots fail fast with the available list). Current robots: `piper` (6-DoF arm + gripper), `xarm7` (7-DoF arm, vendor: UFACTORY — see `arms/xarm7/README.md` + `arms/xarm7/ik_decision_card.md`). Command the arm via the JTC action (`/joint_trajectory_controller/follow_joint_trajectory`, type `control_msgs/action/FollowJointTrajectory`) over all the robot's joints. Intended workflow (README has the switch table): iterate algorithms on the mock chain, regress each version on Gazebo, both green = pass.

## Architecture

Three layers orchestrated by `unistackbot_bringup`. `unistackbot_description` is横切层 (cross-cutting), consumed by all layers via URDF/TF:

```
unistackbot_bringup            → 顶层启动 / 参数编排
unistackbot_controller         → 运动控制 / 运动学解算  (rclcpp, geometry_msgs, nav_msgs, tf2)
unistackbot_hardware           → 硬件抽象 / 驱动通信     (hardware_interface, pluginlib, serial)
  ├─ unistackbot_sim_control  → 统一仿真控制层（kinematic 后端 + /sim_control 服务）
  └─ unistackbot_gazebo        → Gazebo 集成（world + launch）
物理硬件 / 仿真器
```

Morphology differences are isolated to: `description` model files, `hardware`/`sim_control` backend plugins, and `controller` kinematics/balance algorithms. Upper-layer logic and the bringup framework stay shared.

### Package responsibilities

- **unistackbot_bringup** — top-level launch + controller config. `control.launch.py robot:=<机型>` starts `robot_state_publisher`, a standalone `ros2_control_node`, and spawners for `joint_state_broadcaster` + `joint_trajectory_controller`; `robot` is required and no robot name is hardcoded anywhere in the launch. Controller config is per-robot: `config/<robot>_controllers.yaml` (update_rate 500 Hz; JTC with position command interface), shared by both the standalone node and Gazebo mode. `scripts/demo_motion.py` (`ros2 run`) sends a round-trip demo trajectory for **any** robot — it reads the joint list from the controller's params, limits/max_velocity from `robot_description`, and generates waypoints as range-percentages (no robot is hardcoded). Goals must list ALL the robot's joints (JTC rejects subsets unless `allow_partial_joints_goal`).
- **unistackbot_controller** — placeholder only; empty `placeholder.cpp` built as a SHARED library. Scope decision (2026-09-09): this package hosts **only generic infrastructure controllers** (OTG ingest gate for external commands — safety chain member — plus FK from URDF and a plugin template for algorithm controllers). Form-specific algorithms (gait/WBC/ZMP/wheel kinematics) live in algorithm teams' own repos and connect via two first-class paths: external process (ingress+OTG) or CM plugin (framework provides the template). Full boundary analysis: `docs/hardware_framework_design.md` §14.
- **unistackbot_description** — URDF/Xacro, RViz config, `display.launch.py`.
- **unistackbot_hardware** — real-driver plugin skeleton (empty `placeholder.cpp`); deps (`hardware_interface`, `pluginlib`) and the `device`/`baudrate`/`loop_rate` params in `piper_ros2_control.xacro` anticipate the real Piper CAN driver.
- **unistackbot_sim_control** — 统一仿真控制层（ 对 ros2_control 提供统一插件接口，对内按后端分类）:
  - `SimControlHardware` (`SystemInterface`): URDF `<hardware>` 里固定写 `<plugin>unistackbot_sim_control/SimControlHardware</plugin>` + `<param name="backend">kinematic</param>`。接口按 URDF 声明镜像导出（effort 恒 0）；关节动态数量 ≤16，限位/`max_velocity`/mimic 全部来自 `<ros2_control>` 的 `<param>`。
  - 后端 `kinematic`（`BackendKinematic`）: 理想执行器 —— 限位 clamp + 每关节 `max_velocity` 饱和的一阶逼近；mimic 关节按 multiplier/offset 从源关节推导。
  - `/sim_control/*` 服务（reset / set_joint_state / pause / resume / step）: 插件进程内自建（`on_configure` 启动、`on_cleanup` 销毁），命令经 SPSC 无锁队列交给实时循环。`/sim_control` 契约（`unistackbot_interface/srv/SetJointState` + `sim_control_contract.hpp` 的 SimCommand/SimCmdType/SimControlServer）已上收至 `unistackbot_interface`（2026-09-16），本包经 `sim_command_queue.hpp` 的 using-alias 保持既有引用；应答 `success=true` 只代表命令已被接受（入队/ign 请求已发出），不代表执行完成。注意：JTC 激活时其保持命令每周期都会覆盖瞬移，**set_joint_state/reset 需在 pause 下使用**。
  - 日志走 ulog 宏（`ULOG_INFO`/`ULOG_ERROR`，非 RCLCPP）: `on_init` 里 `ulog_init` —— 终端 sink 恒开（launch 捕获 stdout），URDF `<ros2_control>` 块加 `<param name="ulog_file">` 可选开文件 sink；`on_shutdown` 里 `ulog_shutdown` 排空落盘。
  - gz 链路的 `/sim_control/*` 由 `unistackbot_gazebo` 包里的 `sim_control_gz_node` 承载（2026-09-16 迁出：gz 知识归集成层；契约在 `unistackbot_interface`，本包与 gz 包都只依赖它、互不依赖）——见 unistackbot_gazebo 条目。
  - 回归测试: 仓库根 `test/verify_robot.sh <robot> [--with-gazebo]`（机型参数化链路验收: 静态 URDF + launch 错误路径 + mock/Fortress 链 JTC 到位，目标位置从 URDF 限位自动推算、不写死任何机型）；`test/smoke_sim_control.sh` 深测 `/sim_control` 服务语义（piper 关节表，断言含 mimic/限位拒绝/reset）。两者都自包含（需先 colcon build + source，脚本自行定位工作区并套用 `~/cyclonedds.xml`）。
- **unistackbot_interface** — 公共接口定义包（ROS msg/srv/action + 纯 C++ 共享契约头）：跨包/跨仓库共享的类型放在这里（单一事实源，供算法团队外部仓库依赖）。触发场景：master.hpp 的 RobotStateSnapshot/JointCmd、RL ingress 命令 schema、state 出口消息。首批契约已落地（2026-09-16）：`/sim_control` 契约（`srv/SetJointState.srv` + `sim_control_contract.hpp` 的 SimCommand/SimCmdType/SimControlServer/kMaxJoints），消费方为 sim_control 插件（mock 链）与 gazebo 的 gz 适配器——两个实现互不依赖，都只依赖本包。
- **unistackbot_common** — 组件库，**不是 ROS 包**（无 package.xml/CMakeLists，colcon 自动忽略）：纯代码存放层，保持可在非 ROS 环境（RT 主站线程/单元测试/ARM 交叉编译）中直接复用。现有组件（每个独立子文件夹 = 文档 + 实现 + 测试三件套）：
  - `sp_latest/` — 双缓冲覆盖写/取最新原语（最新 **1** 个，"值通道"，seqlock 宣告式，seq 位宽自适应 64/32 位）；sim_control 的 `threaded` 后端三通道用它
  - `sp_ring/` — SPSC 无锁环形队列（**逐条必达**，"事件通道"，满拒新 push 语义）；sim_control 的 `sim_command_queue.hpp` 经 using-declaration 引用 `SpscRing`，域类型留在 sim_control
  - `mpsc_ring/` — 多写单读覆盖式无锁环（**丢旧保新**：日志缓冲/滑动窗口/音视频环；Vyukov 每槽 seq 2g/2g+1 编码 + 逐出 CAS；六轮外部评审定稿，载荷原子字节存储零 UB，seq_cst 全屏障，TSAN 零竞争）
  - `ulog/` — 高性能异步日志组件（前端宏：级别过滤→snprintf 定长 POD→无锁入队；后端单线程双 sink 终端+文件、error 强刷、100ms 周期 flush、10MB×5 轮转；emit 零 malloc、WCET ~µs；TSAN 白名单一条已知工具误报见 tsan_suppressions.txt）
  - sim_control 的 CMake 以 `$<BUILD_INTERFACE:...>/../unistackbot_common` 直引组件库整个目录（sp_ring 队列 + ulog 日志；monorepo 内，未 install——对外发布前需调整）
- **unistackbot_gazebo** — Gazebo integration, single chain: `ign.launch.py` (Gazebo Sim/Fortress via `ros_gz_sim` + `empty_ign.world` + bridges + `sim_control_gz_node`). The Gazebo Classic chain was removed 2026-09-17 (EOL, never verified here; recover from git history if ever needed). Both take a required `robot:=<机型>`; no robot name is hardcoded. In both, the controller manager lives inside the sim's ros2_control plugin — no standalone `ros2_control_node`. Constraints baked into the launches, each fixes a hard failure observed on dev machines:
  - URDF is re-serialized to a **single line** before use: the plugin forwards it to the CM as a `--param robot_description:=<urdf>` rule and rcl's parser rejects newlines → CM never starts.
  - The sim gets the URDF via a temp file (`-file` for spawn_entity / `create`), not the `/robot_description` topic: TRANSIENT_LOCAL latched re-delivery is unreliable under iceoryx/SHM CycloneDDS configs.
  - DDS comes from the machine-wide `~/cyclonedds.xml` (`CYCLONEDDS_URI` in `.bashrc`): binds `lo` + unicast `Peers 127.0.0.1`. This machine's `lo` lacks the MULTICAST flag, so unicast-only discovery intermittently dropped late joiners (services visible at startup, gone minutes later); the Peers bootstrap fixed it. The launches deliberately do NOT override `CYCLONEDDS_URI`. Run at most ONE launch stack at a time — leftover same-name nodes (robot_state_publisher / controller_manager) poison new runs ('Controller already loaded', stale robot_description).
  - The ign chain prepends the description package's ament share root to `IGN_GAZEBO_RESOURCE_PATH`: URDF→SDF conversion rewrites `package://` to `model://`, and Fortress resolves those only via that env var — without it the GUI/server can't find meshes (Classic's gazebo_ros did this automatically).
  - `scripts/gz_clean.sh` (`ros2 run unistackbot_gazebo gz_clean.sh`) kills the whole launch process family — ign servers routinely survive launch shutdown and poison the next run; run it before every launch.
  - `src/sim_control_gz_node.cpp` — `/sim_control/*` 的 gz 适配器（**纯 ROS 构建，零 ign 编译依赖**，2026-09-16 重写）。gz 链路的 ros2_control 插件是 `GazeboSimSystem` 而非 SimControlHardware，故 `/sim_control/*` 由这个 side-car 独立节点承载：pause/resume/step 经 launch 里 parameter_bridge 桥接的 `/world/<world>/control`（`ros_gz_interfaces/srv/ControlWorld`）下发；`reset` 拒绝（Fortress 实测有毒：返回 success=true 但世界 negative-timestep 停摆）、`set_joint_state` 拒绝（无原生等价，Garden+ 才有）。复用 `unistackbot_interface` 的契约（依赖方向：gazebo → interface，**与 sim_control 零依赖**）。由 `ign.launch.py` 启动，`world` 参数（`unistack_world`，共享 world 的名字、与机型无关）须与 `empty_ign.world` 的 `<world name>` 及桥接服务名一致。

## ros2_control wiring (cross-file contract)

`piper.urdf.xacro` declares four args: `use_gripper`, `use_ros2_control`, `use_world`, `use_gazebo`. `use_gazebo` is three-way (xacro quirk: `$(arg …)` turns `true` into Python `True`, so comparisons must match both):

- `use_gazebo:=false` → `unistackbot_sim_control/SimControlHardware` (backend=kinematic; bringup's standalone `ros2_control_node`)
- `use_gazebo:=true`/`classic` → Gazebo Classic `gazebo_ros2_control/GazeboSystem`（**链已删**，仅 xacro 分支保留为惰性能力，无消费者）
- `use_gazebo:=ign` → Gazebo Sim (Fortress) `gz_ros2_control/GazeboSimSystem` + `gz_ros2_control-system` — the current sim chain

All `<gazebo>` plugin blocks live in each robot's `<robot>_ros2_control.xacro` and point at `$(find unistackbot_bringup)/config/<robot>_controllers.yaml` — the *description* package depends on bringup's installed config in both sim modes.

Contracts to keep in sync when changing joints or interfaces:

- `SimControlHardware` accepts dynamic joints (≤16). Each joint must declare exactly one `position` command interface; state interfaces mirror the URDF (position required, velocity/effort optional, effort is always published as 0). Per-joint `max_velocity` `<param>` caps the kinematic backend (default 5 rad/s).
- `gripper_joint1`/`gripper_joint2` follow `gripper` via URDF `<mimic>` tags **plus** `<param name="mimic">`/`multiplier` entries in the `<ros2_control>` block (both simulators' ros2_control plugins require the block entries); the kinematic backend derives their state in `step()`.
- Gotcha: inside `piper_ros2_control.xacro` the gripper block tests `$(arg use_gripper)` — the *global* xacro arg, not the macro's `use_gripper` param, which is therefore dead there. It only works because `piper.urdf.xacro` passes the arg through unchanged.

## Description package conventions

Robot models live under `unistackbot_description/arms/<robot>/urdf/` with mesh assets under `arms/<robot>/meshes/{visual,collision}/`. Mesh paths in URDF use the `package://unistackbot_description/...` URI — keep this prefix when adding new robots so resolved paths work after `--symlink-install`.

The **Piper arm** is the reference model and the pattern to follow for new robots:
- `piper.urdf.xacro` — top-level entry; declares the xacro args above and conditionally includes the macros below.
- `piper_macro.xacro` — `piper_arm` macro defining `base_link` → `link6` with a `parent` param. With `use_world:=true` a `world` link is added and the arm is fixed to it; otherwise `base_link` is the root (what Gazebo spawns).
- `piper_gripper_macro.xacro` — `piper_gripper` macro (flange + gripper_base + prismatic `gripper` joint + two mimic fingers). Keep an `<inertial>` on the otherwise empty `gripper_link` — Gazebo drops inertia-less links, which would remove the `gripper` joint the JTC config depends on.
- `piper_ros2_control.xacro` — `<ros2_control>` block with the plugin switch described above; joints declare a `position` command interface and `min`/`max`/`max_velocity` limit params the kinematic backend enforces.
- `piper.urdf` and `piper_with_gripper.xacro` — flat (non-xacro-macro) variants kept alongside the macro versions.

`display.launch.py` uses `OpaqueFunction` to defer xacro processing so launch args can be passed into the `xacro` command substitution. It selects `joint_state_publisher_gui` vs `joint_state_publisher` via mutually exclusive `IfCondition`/`UnlessCondition` on the `gui` arg.

## docs/ — 设计文档与课程笔记（中文）

Not build input, but two items are normative for code:

- `cpp_style_guide.md` — **repo-wide C++ style, binding**（全工程适用）; the Conventions section below defers to it.
- `hardware_framework_design.md` — master working doc for the real-hardware layer (`unistackbot_hardware` + RT core): 分层架构、线程模型（双线程主形态 B）、协议后端契约（master.hpp 五要素、形态盲对象模型）、v3 语义状态机、形态与算法边界（§14：命令模式与形态正交、算法双通道接入）、与 ros2_control 生态的对照审核（§15：痛点规避表、RealtimePublisher、对外接口预留）. Status: 讨论中、未收口 (2026-09) — real-driver design lands here first, implementation follows.
- `socketcan_master_design.md` / `ethercat_master_design.md` — full designs of the two bus-master backends (SocketCAN CAN FD; IgH ecrt, 1kHz + DC), structurally parallel, both written against the master.hpp contract in the framework doc.
- `linux_rt_guide.md` / `rt_software_architecture.md` — RT system tuning (PREEMPT_RT, isolcpus, IRQ affinity, cyclictest/抖动排查) and the RT software architecture manifesto（七支柱）. Normative for how RT threads and cross-thread data paths are written.
- `control_course/` — control-theory course notes（传递函数 → 三环级联、PM/带宽账）. Background for the numbers cited in the design docs; not code documentation.

## Conventions to preserve

- C++ style is governed by `docs/cpp_style_guide.md`. Non-negotiables: Allman braces on their own line everywhere; **Tab indentation** (display width 4), never spaces; filename = snake_case of the main class (`sim_control_hardware.hpp` ↔ `SimControlHardware`); `PKG__FILE_HPP_` include guards; `k`-prefixed constexpr constants, `snake_case_` members; **explicit error flow** — our code never throws (errors via return value + message, `[[nodiscard]]` on error-returning functions); system calls that are documented to throw (`std::stod` etc.) are caught at the call boundary and translated into return values — exceptions never enter the control chain (style guide rule 27). All C++ packages compile with `-Wall -Wextra -Wpedantic` and `cxx_std_17`, zero warnings. RT hot paths: lock-free structures/atomics only — no mutex, malloc, printf, or IO in the control loop (RT code discipline lives in `docs/linux_rt_guide.md` part 2, not the style guide).
- Each package uses `ament_lint_auto` with `ament_lint_common` for tests — match this when adding new packages.
- `bringup`'s `package.xml` lists the packages it orchestrates as `exec_depend` (including `unistackbot_sim_control`); `unistackbot_gazebo` similarly declares its runtime deps. Add new runtime-consumed packages there.
- Keep `.gitkeep` files in empty `launch/`, `config/`, `include/` directories so the package structure survives in git.

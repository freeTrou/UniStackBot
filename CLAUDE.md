# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UniStackBot is a ROS 2 Humble workspace for a general-purpose, multi-morphology real-time robot control framework. It targets fixed-base arms, wheeled bases, quadrupeds, wheeled-arm humanoids, and bipedal humanoids from a single layered architecture. The first concrete robot is the **Piper arm**: URDF/Xacro description + mock `hardware_interface` plugin + Gazebo integration. `unistackbot_controller` and `unistackbot_hardware` (real driver) are still skeletons (empty `placeholder.cpp`).

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

Three launch entry points, all currently Piper-specific:

```bash
# 1. Visualization only (RViz + joint_state_publisher_gui; ros2_control off by default)
ros2 launch unistackbot_description display.launch.py
#    args: model:=<xacro> use_gripper use_ros2_control use_world gui rviz

# 2. Mock control chain: standalone controller_manager + MockPiperHardware, no simulator
ros2 launch unistackbot_bringup piper_control.launch.py     # use_rviz:=true|false

# 3. Gazebo Sim / Fortress (current chain): controller manager inside gz_ros2_control
ros2 launch unistackbot_gazebo piper_ign.launch.py          # gui:=true|false

# 4. Gazebo Classic (EOL, kept for reference): controller manager inside gazebo_ros2_control
ros2 launch unistackbot_gazebo gazebo.launch.py             # gui:=true|false
```

Command the arm via the JTC action (`/joint_trajectory_controller/follow_joint_trajectory`) over joints `joint1`–`joint6` + `gripper`. The README's run example (`unistackbot.launch.py`) does not exist yet.

## Architecture

Three layers orchestrated by `unistackbot_bringup`. `unistackbot_description` is横切层 (cross-cutting), consumed by all layers via URDF/TF:

```
unistackbot_bringup            → 顶层启动 / 参数编排
unistackbot_controller         → 运动控制 / 运动学解算  (rclcpp, geometry_msgs, nav_msgs, tf2)
unistackbot_hardware           → 硬件抽象 / 驱动通信     (hardware_interface, pluginlib, serial)
  ├─ unistackbot_hardware_mock → 仿真/mock 硬件插件（当前可用）
  └─ unistackbot_gazebo        → Gazebo 集成（world + launch）
物理硬件 / 仿真器
```

Morphology differences are isolated to: `description` model files, `hardware`/`hardware_mock` driver plugins, and `controller` kinematics/balance algorithms. Upper-layer logic and the bringup framework stay shared.

### Package responsibilities

- **unistackbot_bringup** — top-level launch + controller config. `piper_control.launch.py` starts `robot_state_publisher`, a standalone `ros2_control_node`, and spawners for `joint_state_broadcaster` + `joint_trajectory_controller`. `config/piper_controllers.yaml` is the single controller config (update_rate 500 Hz; JTC with position command interface over the 7 joints) and is shared by both the standalone node and Gazebo mode. `scripts/piper_demo_motion.py` (`ros2 run`) sends a canned round-trip trajectory — goals must list ALL 7 joints (JTC rejects subsets unless `allow_partial_joints_goal`).
- **unistackbot_controller** — placeholder only; empty `placeholder.cpp` built as a SHARED library. Intended: kinematics/dynamics solve + control laws at a fixed control cycle.
- **unistackbot_description** — URDF/Xacro, RViz config, `display.launch.py`.
- **unistackbot_hardware** — real-driver plugin skeleton (empty `placeholder.cpp`); deps (`hardware_interface`, `pluginlib`) and the `device`/`baudrate`/`loop_rate` params in `piper_ros2_control.xacro` anticipate the real Piper CAN driver.
- **unistackbot_hardware_mock** — `hardware_interface::SystemInterface` plugin `unistackbot_hardware_mock/MockPiperHardware`. Its `read()` steps each joint's state toward the commanded position at a per-joint velocity cap (5 rad/s; gripper 3 rad/s), emulating actuator speed limits. Joint limits come from the `min`/`max` `<param>`s in the `<ros2_control>` block.
- **unistackbot_gazebo** — Gazebo integration, two chains: `piper_ign.launch.py` (Gazebo Sim/Fortress via `ros_gz_sim` + `empty_ign.world` + `/clock` bridge — the current chain) and `gazebo.launch.py` (Gazebo Classic, EOL, kept for reference). In both, the controller manager lives inside the sim's ros2_control plugin — no standalone `ros2_control_node`. Constraints baked into the launches, each fixes a hard failure observed on dev machines:
  - URDF is re-serialized to a **single line** before use: the plugin forwards it to the CM as a `--param robot_description:=<urdf>` rule and rcl's parser rejects newlines → CM never starts.
  - The sim gets the URDF via a temp file (`-file` for spawn_entity / `create`), not the `/robot_description` topic: TRANSIENT_LOCAL latched re-delivery is unreliable under iceoryx/SHM CycloneDDS configs.
  - DDS comes from the machine-wide `~/cyclonedds.xml` (`CYCLONEDDS_URI` in `.bashrc`): binds `lo` + unicast `Peers 127.0.0.1`. This machine's `lo` lacks the MULTICAST flag, so unicast-only discovery intermittently dropped late joiners (services visible at startup, gone minutes later); the Peers bootstrap fixed it. The launches deliberately do NOT override `CYCLONEDDS_URI`. Run at most ONE launch stack at a time — leftover same-name nodes (robot_state_publisher / controller_manager) poison new runs ('Controller already loaded', stale robot_description).
  - The ign chain prepends the description package's ament share root to `IGN_GAZEBO_RESOURCE_PATH`: URDF→SDF conversion rewrites `package://` to `model://`, and Fortress resolves those only via that env var — without it the GUI/server can't find meshes (Classic's gazebo_ros did this automatically).
  - `scripts/gz_clean.sh` (`ros2 run unistackbot_gazebo gz_clean.sh`) kills the whole launch process family — ign servers routinely survive launch shutdown and poison the next run; run it before every launch.

## ros2_control wiring (cross-file contract)

`piper.urdf.xacro` declares four args: `use_gripper`, `use_ros2_control`, `use_world`, `use_gazebo`. `use_gazebo` is three-way (xacro quirk: `$(arg …)` turns `true` into Python `True`, so comparisons must match both):

- `use_gazebo:=false` → `unistackbot_hardware_mock/MockPiperHardware` (bringup's standalone `ros2_control_node`)
- `use_gazebo:=true`/`classic` → Gazebo Classic `gazebo_ros2_control/GazeboSystem` + `libgazebo_ros2_control.so`
- `use_gazebo:=ign` → Gazebo Sim (Fortress) `gz_ros2_control/GazeboSimSystem` + `gz_ros2_control-system` — the current sim chain

All `<gazebo>` plugin blocks live in `piper_ros2_control.xacro` and point at `$(find unistackbot_bringup)/config/piper_controllers.yaml` — the *description* package depends on bringup's installed config in both sim modes.

Contracts to keep in sync when changing joints or interfaces:

- `MockPiperHardware::on_init` requires exactly 7 joints (`joint1`–`joint6` + `gripper`), each with exactly one `position` command interface; joints 1–6 declare 3 state interfaces (position/velocity/effort), the gripper only 2 (no effort). Max velocities are hard-coded by joint name in the plugin.
- `gripper_joint1`/`gripper_joint2` follow `gripper` via URDF `<mimic>` tags **plus** `<param name="mimic">`/`multiplier` entries in the `<ros2_control>` block (both simulators' ros2_control plugins require the block entries); the mock plugin derives their state in `read()`.
- Gotcha: inside `piper_ros2_control.xacro` the gripper block tests `$(arg use_gripper)` — the *global* xacro arg, not the macro's `use_gripper` param, which is therefore dead there. It only works because `piper.urdf.xacro` passes the arg through unchanged.

## Description package conventions

Robot models live under `unistackbot_description/arms/<robot>/urdf/` with mesh assets under `arms/<robot>/meshes/{visual,collision}/`. Mesh paths in URDF use the `package://unistackbot_description/...` URI — keep this prefix when adding new robots so resolved paths work after `--symlink-install`.

The **Piper arm** is the reference model and the pattern to follow for new robots:
- `piper.urdf.xacro` — top-level entry; declares the xacro args above and conditionally includes the macros below.
- `piper_macro.xacro` — `piper_arm` macro defining `base_link` → `link6` with a `parent` param. With `use_world:=true` a `world` link is added and the arm is fixed to it; otherwise `base_link` is the root (what Gazebo spawns).
- `piper_gripper_macro.xacro` — `piper_gripper` macro (flange + gripper_base + prismatic `gripper` joint + two mimic fingers). Keep an `<inertial>` on the otherwise empty `gripper_link` — Gazebo drops inertia-less links, which would remove the `gripper` joint the JTC config depends on.
- `piper_ros2_control.xacro` — `<ros2_control>` block with the plugin switch described above; joints declare a `position` command interface and `min`/`max` limit params the mock plugin enforces.
- `piper.urdf` and `piper_with_gripper.xacro` — flat (non-xacro-macro) variants kept alongside the macro versions.

`display.launch.py` uses `OpaqueFunction` to defer xacro processing so launch args can be passed into the `xacro` command substitution. It selects `joint_state_publisher_gui` vs `joint_state_publisher` via mutually exclusive `IfCondition`/`UnlessCondition` on the `gui` arg.

## Conventions to preserve

- C++ packages compile with `-Wall -Wextra -Wpedantic` and `cxx_std_17`.
- Each package uses `ament_lint_auto` with `ament_lint_common` for tests — match this when adding new packages.
- `bringup`'s `package.xml` lists the packages it orchestrates as `exec_depend` (including `unistackbot_hardware_mock`); `unistackbot_gazebo` similarly declares its runtime deps. Add new runtime-consumed packages there.
- Keep `.gitkeep` files in empty `launch/`, `config/`, `include/` directories so the package structure survives in git.

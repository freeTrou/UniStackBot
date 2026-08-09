# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UniStackBot is a ROS 2 Humble workspace for a general-purpose, multi-morphology real-time robot control framework. It targets fixed-base arms, wheeled bases, quadrupeds, wheeled-arm humanoids, and bipedal humanoids from a single layered architecture. The framework is in an early skeleton state: most packages contain only `placeholder.cpp` and `.gitkeep` files; the first concrete asset is the **Piper arm** URDF/Xacro description.

## Build & Run

Build from the **workspace root** (`UniStackBot_ws/`), not from this repo directory:

```bash
cd <path-to>/UniStackBot_ws
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package: `colcon build --symlink-install --packages-select unistackbot_description`

Lint/tests (per package, via `ament_lint_auto`):
```bash
colcon test --packages-select <package>
colcon test-result --verbose
```

Run the description visualization (the only functional launch today):
```bash
ros2 launch unistackbot_description display.launch.py
# Common overrides:
#   model:=<path-to-xacro>  use_gripper:=true|false
#   use_ros2_control:=true|false  use_world:=true|false
#   gui:=true|false  rviz:=true|false
```

The README references `ros2 launch unistackbot_bringup unistackbot.launch.py`, but that launch file does not exist yet — `unistackbot_bringup` is a placeholder package.

## Architecture

Three-layer stack orchestrated by `unistackbot_bringup`. `unistackbot_description` is横切层 (cross-cutting), consumed by all layers via URDF/TF:

```
unistackbot_bringup       → 顶层启动 / 参数编排
unistackbot_controller    → 运动控制 / 运动学解算  (rclcpp, geometry_msgs, nav_msgs, tf2)
unistackbot_hardware      → 硬件抽象 / 驱动通信     (hardware_interface, pluginlib, serial)
物理硬件 / 仿真器
```

Morphology differences are isolated to: `description` model files, `hardware` driver plugins, and `controller` kinematics/balance algorithms. Upper-layer logic and the bringup framework stay shared.

### Package responsibilities

- **unistackbot_bringup** — top-level launch + parameter orchestration; switches between real-robot and simulation modes. Currently empty (only `.gitkeep`).
- **unistackbot_controller** — kinematics/dynamics solve and control-law computation at a fixed control cycle; translates high-level commands into joint commands. `CMakeLists` builds `src/placeholder.cpp` as a SHARED library; deps declare intent (`geometry_msgs`, `nav_msgs`, `tf2`) but no implementation exists.
- **unistackbot_description** — URDF/Xacro, RViz config, `robot_state_publisher`/`joint_state_publisher` launch. The only package with real content today.
- **unistackbot_hardware** — `hardware_interface` plugin(s) + driver comms (CAN/serial via `serial` dep). Declares `pluginlib` for plugin export. No plugins implemented yet; `src/placeholder.cpp` is a stub.

## Description package conventions

Robot models live under `unistackbot_description/arms/<robot>/urdf/` with mesh assets under `arms/<robot>/meshes/{visual,collision}/`. Mesh paths in URDF use the `package://unistackbot_description/...` URI — keep this prefix when adding new robots so resolved paths work after `--symlink-install`.

The **Piper arm** is the reference model and the pattern to follow for new robots:
- `piper.urdf.xacro` — top-level entry; declares xacro args (`use_gripper`, `use_ros2_control`, `use_world`) and conditionally includes macros.
- `piper_macro.xacro` — `piper_arm` macro defining `base_link` → `link6` with a `parent` param. When `use_world:=true`, a `world` link is added and the arm is fixed to it.
- `piper_gripper_macro.xacro` — `piper_gripper` macro (flange + gripper_base + two mimic prismatic joints).
- `piper_ros2_control.xacro` — `<ros2_control>` block. References `unistackbot_hardware/PiperSystem` plugin (not yet implemented). Joints declare `position` command interface and `position/velocity/effort` state interfaces; the gripper uses `<param name="mimic">` for coupled joints.
- `piper.urdf` and `piper_with_gripper.xacro` — flat (non-xacro-macro) variants kept alongside the macro versions.

`display.launch.py` uses `OpaqueFunction` to defer xacro processing so launch args can be passed into the `xacro` command substitution. It selects `joint_state_publisher_gui` vs `joint_state_publisher` via mutually exclusive `IfCondition`/`UnlessCondition` on the `gui` arg.

## Conventions to preserve

- C++ packages compile with `-Wall -Wextra -Wpedantic` and `cxx_std_17`.
- Each package uses `ament_lint_auto` with `ament_lint_common` for tests — match this when adding new packages.
- `bringup`'s `package.xml` lists the three other packages as `exec_depend`; new packages consumed at runtime should be added there.
- Keep `.gitkeep` files in empty `launch/`, `config/`, `include/` directories so the package structure survives in git.

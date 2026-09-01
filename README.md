# UniStackBot

一个面向通用机器人的实时控制框架。基于 ROS 2 Humble 构建，采用分层架构将硬件抽象、运动控制、模型描述与系统启动解耦，覆盖从固定基座机械臂到双足人形在内的多种机器人本体形态，并在保证实时性的前提下提供可扩展的软件栈。当前参考本体为 **Piper 机械臂**（6 关节 + 夹爪）。

## 设计目标

- **通用性**：硬件层与控制层解耦，通过配置与插件适配不同本体，无需改动上层逻辑。
- **实时性**：控制环路与硬件通信走低延迟路径，避免非确定性调度，满足实时控制需求。
- **模块化**：各功能包职责清晰、可独立构建与替换，便于按需裁剪或扩展。
- **可复现**：统一的启动入口与参数管理，便于在仿真与真机之间无缝切换。

## 支持的机器人形态

框架不绑定单一本体，通过 URDF 模型 + 硬件插件 + 控制器配置的组合即可适配不同形态的机器人。当前面向以下本体类型设计：

| 形态 | 说明 | 控制侧重 |
| --- | --- | --- |
| 单臂 / 双臂机械臂 | 固定基座机械臂 | 关节空间与笛卡尔空间轨迹规划、运动学/动力学求解 |
| 轮式机器人 | 差速、麦克纳姆、全向轮等 | 底盘运动学解算、里程计、速度平滑 |
| 四足机器人 | 腿式移动平台 | 步态生成、足端轨迹、平衡控制 |
| 轮臂人形机器人 | 轮式底盘 + 上肢机械臂 | 底盘与臂的协同、整体平衡 |
| 双足人形机器人 | 双腿站立人形 | 质心轨迹、ZMP/平衡控制、全身运动 |

不同形态之间共享同一套硬件抽象与启动框架，差异主要集中在 `description` 的模型文件、`hardware` 的驱动插件，以及 `controller` 的运动学/平衡算法上。

## 目录结构

```
unistackbot/
├── unistackbot_bringup/        # 顶层启动包
├── unistackbot_controller/     # 运动控制层
├── unistackbot_description/    # 机器人模型描述
├── unistackbot_hardware/       # 硬件抽象层
├── unistackbot_hardware_mock/  # 仿真硬件插件
├── unistackbot_gazebo/         # Gazebo 仿真集成
├── README.md
├── LICENSE
└── .gitignore
```

## 架构

框架自底向上分为三层，由 bringup 统一编排启动：

```
┌─────────────────────────────────────────────┐
│           unistackbot_bringup               │  顶层启动 / 参数编排
├─────────────────────────────────────────────┤
│         unistackbot_controller              │  运动控制 / 运动学解算
├─────────────────────────────────────────────┤
│          unistackbot_hardware               │  硬件抽象 / 驱动通信
│   ├─ unistackbot_hardware_mock              │  仿真/mock 硬件插件
│   └─ unistackbot_gazebo                     │  Gazebo 仿真集成
├─────────────────────────────────────────────┤
│             物理硬件 / 仿真器                │
└─────────────────────────────────────────────┘
        ▲
        │  URDF/Xacro + TF
        │
   unistackbot_description（模型描述，贯穿各层）
```

## 功能包说明

### unistackbot_bringup

顶层启动包，负责拉起整机各节点并加载运行参数。提供统一入口的 launch 文件和默认配置（控制器配置位于 `config/`），管理真机、mock 与仿真三种运行模式之间的切换，内置演示轨迹脚本。

### unistackbot_controller

运动控制层。在固定控制周期内完成本体相关的运动学/动力学解算与控制律计算，将上层指令转换为底层关节命令。针对不同形态提供差异化策略：机械臂的关节空间与笛卡尔空间轨迹跟踪，轮式底盘的运动学解算与速度平滑，四足/人形的步态生成、质心轨迹与平衡控制（ZMP/全身控制）。向上对接导航、规划与遥操作，向下通过硬件接口读写执行器状态。

### unistackbot_description

机器人模型描述包。存放 URDF/Xacro 文件、可视化配置（RViz）以及 `robot_state_publisher` / `joint_state_publisher` 的启动文件，用于描述连杆、关节、惯量与传感器外参，为仿真、可视化、TF 树与运动学解算提供统一的模型来源。

### unistackbot_hardware

硬件抽象层。负责与底层驱动（底盘电机、关节电机、编码器、IMU、力矩传感器等）通信，向上以 `hardware_interface` 插件或统一话题/服务形式暴露执行器与传感器接口，屏蔽具体硬件差异，使上层控制器与设备解耦，便于跨平台与跨本体复用。

### unistackbot_hardware_mock

仿真硬件层。提供与 `unistackbot_hardware` 接口一致的 `hardware_interface` 插件（`MockPiperHardware`），在无真实硬件时按关节限位与最大角速度模拟执行器响应，用于打通控制链路验证。

### unistackbot_gazebo

Gazebo 仿真集成。主链路基于 **Gazebo Sim (Fortress)**：`ros_gz_sim` 启动仿真、`gz_ros2_control` 在仿真器内运行控制器、`ros_gz_bridge` 桥接仿真时钟；另保留 Gazebo Classic 旧链路作对照。与 mock 链路共用同一套控制器配置，并内置残留进程清理脚本。

## 构建

```bash
# 在 ROS 2 工作空间根目录
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 1. 启动仿真

```bash
# 先清理残留进程（仿真进程可能在上次退出后残留，污染下次启动）
ros2 run unistackbot_gazebo gz_clean.sh

# 启动 Gazebo 仿真（Gazebo Sim / Fortress）
ros2 launch unistackbot_gazebo piper_ign.launch.py
```

其他入口：

```bash
ros2 launch unistackbot_description display.launch.py    # 模型可视化（RViz）
ros2 launch unistackbot_bringup piper_control.launch.py  # mock 控制链路（无需仿真器）
ros2 launch unistackbot_gazebo gazebo.launch.py          # Gazebo Classic 旧链路（对照用）
```

### 2. 运行机械臂

等日志出现两行 `Configured and activated`，然后执行演示轨迹（抬臂 → 转向 → 夹爪开合 → 回零，约 12 秒）：

```bash
ros2 run unistackbot_bringup piper_demo_motion.py
```

## 常见问题

- **报 `Controller already loaded` / 机械臂不动**：多为残留进程冲突，执行 `ros2 run unistackbot_gazebo gz_clean.sh` 后重新启动。
- **DDS 配置**：本机统一使用 `~/cyclonedds.xml`（在 `.bashrc` 中 `export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml`），所有终端共享同一 DDS 域，无需额外前缀。

## 许可证

见 [LICENSE](LICENSE)。

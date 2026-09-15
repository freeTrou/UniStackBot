# UniStackBot

一个面向通用机器人的实时控制框架。基于 ROS 2 Humble 构建，采用分层架构将硬件抽象、运动控制、模型描述与系统启动解耦，覆盖从固定基座机械臂到双足人形在内的多种机器人本体形态，并在保证实时性的前提下提供可扩展的软件栈。当前参考本体为 **Piper 机械臂**（6 关节 + 夹爪）。

**一句话定位**：对上是 VLA / RL / 传统控制等算法的**可靠接入底座**（不可信命令源经校验+OTG 进门）；对下是**总线与电机协议的实时主站**（CAN / EtherCAT，驱动器自带看门狗为安全终点）；**机器人描述（URDF+契约）是贯穿全框架的配置轴**；框架核心对形态与算法保持"形态盲、流派盲"——扩展只发生在插件与配置，核心零改动（验收 = 每次接入新机器人后框架包 git diff 为零）。

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

> 形态与算法的完整边界分析（含命令模式正交、五形态控制栈对比、扩展性三机制）见 `docs/hardware_framework_design.md` §14。

## 架构

框架自底向上分为三层，由 bringup 统一编排启动：

```
┌─────────────────────────────────────────────┐
│           unistackbot_bringup               │  顶层启动 / 参数编排
├─────────────────────────────────────────────┤
│         unistackbot_controller              │  运动控制 / 运动学解算
├─────────────────────────────────────────────┤
│          unistackbot_hardware               │  硬件抽象 / 驱动通信
│   ├─ unistackbot_sim_control                │  统一仿真控制层
│   └─ unistackbot_gazebo                     │  Gazebo 仿真集成
├─────────────────────────────────────────────┤
│             物理硬件 / 仿真器                │
└─────────────────────────────────────────────┘
        ▲
        │  URDF/Xacro + TF
        │
   unistackbot_description（模型描述，贯穿各层）
```

## 功能包一览

| 包 | 职责 | 说明 |
| --- | --- | --- |
| `unistackbot_bringup` | 顶层启动 / 参数编排 | [README](unistackbot_bringup/README.md) |
| `unistackbot_controller` | 运动控制 / 运动学解算（空骨架，范围已裁决） | [README](unistackbot_controller/README.md) |
| `unistackbot_description` | URDF/Xacro 模型 + mesh + RViz（横切层） | [README](unistackbot_description/README.md) |
| `unistackbot_hardware` | 真机驱动 / 总线主站（空骨架，设计先行） | [README](unistackbot_hardware/README.md) |
| `unistackbot_sim_control` | 统一仿真控制层（插件 + 后端 + `/sim_control/*`） | [README](unistackbot_sim_control/README.md) |
| `unistackbot_gazebo` | Gazebo 集成（Fortress 主链 + Classic 对照） | [README](unistackbot_gazebo/README.md) |
| `unistackbot_interface` | 公共接口定义，跨仓库单一事实源（空骨架） | [README](unistackbot_interface/README.md) |
| `unistackbot_common` | 组件库，**非 ROS 包**（交换原语 + ulog 日志） | [README](unistackbot_common/README.md) |

细节看各包 README；自研组件的交付状态与评审记录见 `unistackbot_common/README.md`。

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

需要可视化时手动启动 RViz（Fixed Frame 设为 `world`，Add → RobotModel）：

```bash
rviz2
```

### 3. 检查与清理

```bash
ros2 topic hz /joint_states         # ≈500 Hz = 链路健康
ros2 control list_controllers       # 控制器应为 active

# 报 Controller already loaded / spawner 失败时清残留（mock 链）
pkill -9 -f ros2_control_node; pkill -9 -f robot_state_publisher
# Gazebo 链用: ros2 run unistackbot_gazebo gz_clean.sh
```

## mock / Gazebo 切换

两个环境共享同一套模型、控制器配置与上层工具（JTC、演示脚本、`/sim_control/*` 服务），**切换只需换一条 launch 命令**：

| 环境 | 启动命令 | 特点 |
| --- | --- | --- |
| mock（运动学） | `ros2 launch unistackbot_bringup piper_control.launch.py` | 启动快、完全确定、支持 `set_joint_state` 瞬移，适合算法开发与回归测试 |
| Gazebo（动力学） | `ros2 launch unistackbot_gazebo piper_ign.launch.py` | 真实物理（重力/碰撞/惯性），只支持 pause/resume/step，适合动力学验证 |

切换纪律：**一次只跑一套，切换前先执行 `gz_clean.sh` 清残留**——两套环境的控制器同名（`/controller_manager`），残留进程会污染下一次启动（报 `Controller already loaded`）。

推荐工作流：算法在 mock 上快速迭代，每个版本在 Gazebo 上回归一次，两边都绿才算通过。

## 常见问题

- **报 `Controller already loaded` / 机械臂不动**：多为残留进程冲突，执行 `ros2 run unistackbot_gazebo gz_clean.sh` 后重新启动。
- **DDS 配置**：本机统一使用 `~/cyclonedds.xml`（在 `.bashrc` 中 `export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml`），所有终端共享同一 DDS 域，无需额外前缀。

## 许可证

见 [LICENSE](LICENSE)。

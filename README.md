# UniStackBot

一个面向通用机器人的实时控制框架。基于 ROS 2 Humble 构建，采用分层架构将硬件抽象、运动控制、模型描述与系统启动解耦，覆盖从固定基座机械臂到双足人形在内的多种机器人本体形态。当前参考本体：**Piper**（6 关节 + 夹爪）与 **xArm7**（7 关节，7 轴业务与 IK 路线的参考机）。

**一句话定位**：对上是 VLA / RL / 传统控制等算法的**可靠接入底座**（不可信命令源经校验+OTG 进门）；对下是**总线与电机协议的实时主站**（CAN / EtherCAT，驱动器自带看门狗为安全终点）；**机器人描述（URDF+契约）是贯穿全框架的配置轴**；框架核心对形态与算法保持"形态盲、流派盲"——扩展只发生在插件与配置，核心零改动（验收 = 每次接入新机器人后框架包 git diff 为零）。

## 设计目标

- **通用性**：硬件层与控制层解耦，通过配置与插件适配不同本体，无需改动上层逻辑。
- **实时性**：控制环路与硬件通信走低延迟路径，避免非确定性调度。
- **模块化**：各功能包职责清晰、可独立构建与替换。
- **可复现**：统一启动入口与参数管理，仿真与真机无缝切换；链路级验收一条命令（`test/verify_robot.sh`）。

## 支持的机器人形态

框架不绑定单一本体，通过 URDF 模型 + 硬件插件 + 控制器配置的组合即可适配：单臂/双臂机械臂、轮式、四足、轮臂人形、双足人形。差异集中在 `description` 模型、`hardware`/`sim_control` 后端插件、`controller` 算法；上层逻辑与启动框架共享。

> 形态与算法的完整边界分析见 `docs/hardware_framework_design.md` §14。

## 架构

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
        ▲ URDF/Xacro + TF        ▲ /sim_control 等契约
   unistackbot_description    unistackbot_interface（跨包类型单一事实源）
```

## 功能包一览

| 包 | 职责 | 说明 |
| --- | --- | --- |
| `unistackbot_bringup` | 顶层启动 / 按机型控制器配置 | [README](unistackbot_bringup/README.md) |
| `unistackbot_controller` | 运动控制 / 运动学解算（空骨架，范围已裁决） | [README](unistackbot_controller/README.md) |
| `unistackbot_description` | URDF/Xacro 模型 + mesh + RViz（横切层） | [README](unistackbot_description/README.md) |
| `unistackbot_hardware` | 真机驱动 / 总线主站（空骨架，设计先行） | [README](unistackbot_hardware/README.md) |
| `unistackbot_sim_control` | 统一仿真控制层（插件 + 后端 + `/sim_control/*`） | [README](unistackbot_sim_control/README.md) |
| `unistackbot_gazebo` | Gazebo 集成（Fortress 链） | [README](unistackbot_gazebo/README.md) |
| `unistackbot_interface` | 公共接口定义，跨仓库单一事实源（`/sim_control` 契约已落地） | [README](unistackbot_interface/README.md) |
| `unistackbot_common` | 组件库，**非 ROS 包**（交换原语 + ulog 日志） | [README](unistackbot_common/README.md) |

细节看各包 README。

## 构建

```bash
# 在 ROS 2 工作空间根目录
colcon build --symlink-install
source install/setup.bash
```

## 运行

所有控制/仿真 launch 的 `robot:=<机型>` **必填**（现有机型：`piper`、`xarm7`；未知机型显式报错并列出可用项）。

```bash
# 1) 可视化（RViz + 滑块，无需仿真器）
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro

# 2) mock 控制链（standalone CM + SimControlHardware，无仿真器，算法迭代用）
ros2 launch unistackbot_bringup control.launch.py robot:=xarm7 use_rviz:=true

# 3) Gazebo Sim / Fortress（物理引擎链，CM 在 gz_ros2_control 内）
ros2 run unistackbot_gazebo gz_clean.sh        # 单独执行！清残留
ros2 launch unistackbot_gazebo ign.launch.py robot:=xarm7 gui:=false   # use_rviz:=true 可选
```

发一段演示轨迹（机型无关：关节表读自控制器，路径点按限位推算）：

```bash
ros2 run unistackbot_bringup demo_motion.py
```

## 测试

```bash
# 链路级验收（静态 URDF + launch 错误路径 + mock/gz 链 JTC 到位 + RTF 监控）
bash test/verify_robot.sh xarm7 --with-gazebo

# /sim_control 服务语义深测（pause 下瞬移/mimic/超限拒绝/reset）
bash test/smoke_sim_control.sh

# 组件库（不进 colcon，g++ 直编）
cd unistackbot_common/ulog && g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_ulog.cpp -o test_ulog && ./test_ulog
```

## mock / Gazebo 切换

两套环境共享模型、控制器配置与上层工具（JTC、demo、`/sim_control/*`），**切换只换一条 launch 命令**：

| 环境 | 启动命令 | 特点 |
| --- | --- | --- |
| mock（运动学） | `control.launch.py robot:=<机型>` | 启动快、完全确定、支持 `set_joint_state` 瞬移，算法迭代与回归用 |
| Gazebo（物理引擎） | `ign.launch.py robot:=<机型>` | 碰撞检测 + `/stats` RTF 监控；`reset`/`set_joint_state` 在 Fortress 无原生等价（拒绝） |

纪律：**一次只跑一套；`gz_clean.sh` 永远单独执行**（它的 pkill 模式会击杀同一命令行里含 "ros2 launch" 字样的宿主）；换链前先清残留，否则同名节点互相污染（`Controller already loaded`）。推荐工作流：mock 迭代 → 每版本 Gazebo 回归 → `verify_robot.sh` 全绿才通过。

## 常见问题

- **`Controller already loaded` / 臂不动**：残留进程冲突，`gz_clean.sh` 后重启。
- **`Failed to find a free participant index`**：残留 ROS 进程占满 DDS 参与者，`gz_clean.sh` 清零（含 sim_control_gz_node 模式）。
- **DDS**：本机统一 `~/cyclonedds.xml`（`.bashrc` 已 export），所有终端同域。

## 许可证

见 [LICENSE](LICENSE)。

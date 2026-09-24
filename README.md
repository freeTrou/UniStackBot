# UniStackBot

面向通用机器人的实时控制框架（ROS 2 Humble）。分层架构：硬件抽象、运动控制、模型描述、系统启动解耦。当前参考本体：**Piper**（6 关节 + 夹爪）与 **xArm7**（7 关节，7 轴业务参考机）。

**定位**：对上是 VLA/RL/传统控制的**可靠接入底座**；对下是总线与电机协议的**实时主站**（CAN/EtherCAT）；URDF+契约是贯穿框架的配置轴；核心"形态盲、流派盲"——扩展只发生在插件与配置，验收 = 接入新机器人后框架包 git diff 为零。

## 架构

```
┌─────────────────────────────────────────────┐
│           unistackbot_bringup               │  顶层启动 / 参数编排 / supervisor 诊断
├─────────────────────────────────────────────┤
│         unistackbot_controller              │  CM 笛卡尔流式 + JS 关节点流式 + EE 反馈
├─────────────────────────────────────────────┤
│          unistackbot_algorithm              │  纯算法库 (FK/IK/求解器接口/种子库)
├─────────────────────────────────────────────┤
│          unistackbot_hardware               │  真机驱动 / 总线主站 (骨架, 设计先行)
│   └─ unistackbot_sim_control/               │  仿真集成组 (容器, 三独立包)
│      core / gazebo / mujoco                 │    统一仿真层 + Gazebo + MuJoCo
├─────────────────────────────────────────────┤
│             物理硬件 / 仿真器                │
└─────────────────────────────────────────────┘
        ▲ URDF/Xacro + TF        ▲ /sim_control 等契约
   unistackbot_description    unistackbot_interface（跨包类型单一事实源）
```

## 功能包

| 包 | 职责 | 说明 |
| --- | --- | --- |
| `unistackbot_bringup` | 顶层启动 / 按机型控制器配置 / supervisor | [README](unistackbot_bringup/README.md) |
| `unistackbot_controller` | CM（笛卡尔流式）+ JS（关节点流式）+ EE 反馈 + 工具节点 | [README](unistackbot_controller/README.md) |
| `unistackbot_algorithm` | urdf_fk / dls_ik / IkSolver 接口 / 种子库 | [README](unistackbot_algorithm/README.md) |
| `unistackbot_description` | URDF/Xacro + mesh + MJCF + RViz（横切层） | [README](unistackbot_description/README.md) |
| `unistackbot_hardware` | 真机驱动 / 总线主站（空骨架） | [README](unistackbot_hardware/README.md) |
| `unistackbot_sim_control/` | 仿真集成组：core + gazebo + mujoco | [README](unistackbot_sim_control/README.md) |
| `unistackbot_interface` | 跨包/跨仓库共享类型单一事实源 | [README](unistackbot_interface/README.md) |
| `unistackbot_common` | 组件库，非 ROS 包（无锁原语/日志/RT 调优） | [README](unistackbot_common/README.md) |

## 构建

```bash
cd <path-to>/UniStackBot_ws
colcon build --symlink-install
source install/setup.bash
```

## 运行

`robot:=<机型>` 必填（`piper` | `xarm7`，未知机型显式报错）。

```bash
# 统一入口：chain 一参切三链 (mock|gz|mujoco)
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=piper

# 或分别起链
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro  # 可视化（换机型改 model 路径）
ros2 launch unistackbot_bringup control.launch.py robot:=piper      # mock（算法迭代）
ros2 run unistackbot_gazebo gz_clean.sh                             # gz 前清残留，永远单独执行
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper gui:=false  # Gazebo Fortress
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper          # MuJoCo（真实动力学）

ros2 run unistackbot_demo demo_motion.py      # 关节空间演示（前置: 任一链已启动）
ros2 run unistackbot_demo demo_cartesian.py   # 笛卡尔空间演示（切 CM 往返, 机型无关）
```

命令通道：关节空间 `/joint_stream_controller/command`（`unistackbot_interface/JointCommand` 点流）；笛卡尔空间 `/cartesian_motion_controller/target`（PoseStamped, base 系）。双控制器经 `ros2 control switch_controllers` 互切。

## 测试

```bash
bash test/verify_robot.sh piper --with-gazebo --with-mujoco   # 链路级验收（一条命令）
bash test/fault_injection.sh --robot piper --chain mujoco     # 故障注入（F1-F7; --robot 选机型）
bash test/smoke_sim_control.sh                                # /sim_control 服务语义
sudo bash test/rt_tune_boot.sh                                # 开机 RT 调优（先于一切基准）
bash test/rt_chain_bench.sh <标签> --robot piper [--chain mock|mujoco]  # RT 基准套件
```

组件库不进 colcon，g++ 直编（规范命令见 `unistackbot_common/ulog/README.md`）。

## 常见问题

- **臂不动 / `Controller already loaded`**：残留进程冲突，`gz_clean.sh` 后重启；一次只跑一套栈。
- **`Failed to find a free participant index`**：杀链后立即起链所致——清进程 + 等租约再起；确认 `lo` 有 MULTICAST 标志（丢了 `sudo ip link set lo multicast on`，重启会丢）。
- DDS 统一 `~/cyclonedds.xml`（`.bashrc` 已 export）。

## 许可证

见 [LICENSE](LICENSE)。

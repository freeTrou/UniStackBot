# UniStackBot

面向通用机器人的实时控制框架（ROS 2 Humble）。分层架构：硬件抽象、运动控制、模型描述、系统启动解耦。当前参考本体：**Piper**（6 关节 + 夹爪）、**xArm7**（7 关节，7 轴业务参考机）、**StationLite**（双臂，Lumos）与 **R1-7A**（宇树七轴冗余臂）。

**定位**：对上是 VLA/RL/传统控制的**可靠接入底座**；对下是总线与电机协议的**实时主站**（CAN/EtherCAT/串口 RS-485，驱动器自带看门狗为安全终点）；URDF+契约是贯穿框架的配置轴；核心"形态盲、流派盲"——扩展只发生在插件与配置，验收 = 接入新机器人后框架包 git diff 为零。

## 架构

```
      ┌────────────────────────────────────────────┐
      │   unistackbot_demo —— 外部命令源样例         │  控制 demo 各命令域 = 链路契约的独立
      └───────────────────┬────────────────────────┘  外部消费者; VLA/RL/传统算法接入同此位
                          │ /joint_stream_controller/command (JointCommand 点流)
                          │ /cartesian_motion_controller/command (PoseStamped, base 系)
┌─────────────────────────┴──────────────────────────────────────┐
│                       unistackbot_bringup                       │ 顶层启动 / 参数编排 / supervisor 诊断
├────────────────────────────────────────────────────────────────┤
│                    unistackbot_controller                       │ CM 笛卡尔流式 + JS 关节点流式 + EE 反馈
├────────────────────────────────────────────────────────────────┤
│                    unistackbot_algorithm                        │ 纯算法库 (FK/IK/求解器接口/种子库)
├───────────────────────────────┬────────────────────────────────┤
│     unistackbot_hardware      │     unistackbot_sim_control/   │
│     真机域容器                 │     仿真域容器 (三独立包)        │
│  protocol · statemachine ·    │     core / gazebo / mujoco     │
│  bus · serial 四轴包          │     → mock | gz | mujoco 三链   │
│  + 三主站骨架目录             │                                 │
├───────────────────────────────┼────────────────────────────────┤
│      物理总线与电机            │      仿真器                     │
│  RS-485 / CAN FD / EtherCAT   │  kinematic 理想 / Gazebo / MuJoCo │
└───────────────────────────────┴────────────────────────────────┘
      ▲ URDF/Xacro + TF                    ▲ 跨包契约类型单一事实源
 unistackbot_description                 unistackbot_interface
 unistackbot_common（非 ROS 组件库: 无锁原语/异步日志/OTG 整形/RT 调优, 横切全栈）
```

## 功能包

| 包 | 职责 | 说明 |
| --- | --- | --- |
| `unistackbot_bringup` | 顶层启动 / 按机型控制器配置 / supervisor | [README](unistackbot_bringup/README.md) |
| `unistackbot_demo` | 控制 demo：末端位姿流 / 关节慢流 / 关节满速流 / 键盘遥操（笛卡尔+关节域）/ 点位序列回放 | [README](unistackbot_demo/README.md) |
| `unistackbot_controller` | CM（笛卡尔流式）+ JS（关节点流式）+ EE 反馈 + 工具节点 | [README](unistackbot_controller/README.md) |
| `unistackbot_algorithm` | urdf_fk / dls_ik / analytic_piper / IkSolver 接口 / 种子库 | [README](unistackbot_algorithm/README.md) |
| `unistackbot_description` | URDF/Xacro + mesh + MJCF + RViz（横切层） | [README](unistackbot_description/README.md) |
| `unistackbot_hardware` | 真机域容器（protocol codec 轴 + statemachine 翻译轴 + bus 交换轴 + serial 串口主站 + 三骨架目录） | [README](unistackbot_hardware/README.md) |
| `unistackbot_sim_control/` | 仿真集成组：core + gazebo + mujoco | [README](unistackbot_sim_control/README.md) |
| `unistackbot_moveit_config` | MoveIt2 接入配置（任务级规划，执行走链上 JTC；piper 试点 M1） | [README](unistackbot_moveit_config/README.md) |
| `unistackbot_tool` | 工具程序容器（数据录制规范 record_verify / 点位资产转换） | [README](unistackbot_tool/README.md) |
| `unistackbot_interface` | 跨包/跨仓库共享类型单一事实源 | [README](unistackbot_interface/README.md) |
| `unistackbot_common` | 组件库，非 ROS 包（无锁原语/日志/RT 调优/OTG 整形） | [README](unistackbot_common/README.md) |

## 环境安装（Ubuntu 22.04 + ROS 2 Humble）

### 1. ROS 2 Humble 基础（小鱼一键安装）

```bash
wget http://fishros.com/install -O fishros && bash fishros
# 菜单 [1] 一键安装ROS → 选 humble + desktop 版 (含 rviz2/robot_state_publisher/xacro)
# 再跑一次 fishros → [3] 一键配置 rosdep (国内源, 免手动 rosdep init/update)
sudo apt install python3-colcon-common-extensions   # colcon 构建
```

### 2. 工作区依赖（rosdep 一把梭，**必跑，不可跳**）

```bash
cd <path-to>/UniStackBot_ws/src/UniStackBot
rosdep install --from-paths . --ignore-src -y -r
```

覆盖 ros2_control 套件（controller_manager/joint_state_broadcaster 等）、
tf2、realtime_tools、rosbag2+mcap（录制）、tinyxml2_vendor、interactive_markers、
xacro 等全部 ROS 依赖。

> **部署实锤警告（2026-10-13 首台部署机三连坑）**：跳过此步 = 大量**运行时依赖**
> 潜伏到 launch/spawner 才炸——编译不需要它们，colcon 全绿不代表环境齐：
> 缺 `xacro` → launch 报 "executable not found on PATH"；
> 缺 `joint_state_broadcaster` → spawner 报 "Loader for controller not found"。
> 跑完检查输出：**无 ERROR、且看到它装了东西（或输出空=已齐）**才算过。

### 3. rosdep 覆盖外的系统包（手动）

| 包 | 验收版本（2026-10-12 实测） | 说明 |
|---|---|---|
| `ros-humble-mujoco-ros2-control` | 0.1.2 | MuJoCo 链宿主，内嵌 MuJoCo 3.12.0；headless/sim_speed_factor 走 URDF 硬件参数。上游主循环节拍有已知特性（见 mujoco README），**升级前先对账** |
| `ros-humble-gz-ros2-control` | 0.7.21 | gz 链 ros2_control 插件（mimic 接口读 ros2_control 块参数） |
| `ros-humble-ros-gz-bridge` / `-interfaces` / `-sim` | — | gz 链的桥/消息/启停全家（**unistackbot_gazebo 构建必需**；缺 `-interfaces` 时 CMake 在 find_package 直接失败——首台部署机实锤 2026-10-13） |
| `ignition-fortress` | 1.0.3 | Gazebo Sim 发行版（launch/世界 API 按 Fortress 写，Garden+ 不兼容） |
| `libnlopt-cxx-dev` | 2.7.1 | TRAC-IK 的外部依赖（求解器本体已 **vendored 随仓**：`unistackbot_algorithm/trac_ik_solver/vendor/`，无需安装上游包；构建开关 `UNISTACKBOT_BUILD_TRAC_IK` 默认 ON，OFF 时勿编 controller） |
| `libeigen3-dev` / `liborocos-kdl-dev` | — | Eigen / KDL（算法库直引系统头） |
| `ros-humble-rmw-cyclonedds-cpp` | 1.3.5 | CycloneDDS RMW（desktop 默认 Fast DDS，不带此包） |
| `ros-humble-ros2controlcli` | — | `ros2 control` CLI（**独立 deb，rosdep 不装**；缺它则一切控制器切换/demo 报 invalid choice: control——部署机实锤 2026-10-13） |

```bash
sudo apt install ros-humble-mujoco-ros2-control ros-humble-gz-ros2-control \
    ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces ros-humble-ros-gz-sim \
    ignition-fortress libnlopt-cxx-dev libeigen3-dev liborocos-kdl-dev \
    ros-humble-rmw-cyclonedds-cpp ros-humble-ros2controlcli
```

> 版本列是验收时点实测值（apt 源会随时间前移 minor），命令**不锁 apt 版本串**
> （`pkg=0.1.2` 短串匹配会直接失败——候选是 `0.1.2-1jammy.xxx` 全串）。
> 验证基线 = 上表；换源/换镜像后若行为异常，先 `apt-cache policy <包>` 对照此表。

### 4. 可选组件

- **MoveIt 2**（任务级规划, `unistackbot_moveit_config`）: `sudo apt install ros-humble-moveit`
- **joint_state_publisher_gui**（display 链拖动滑条）: `sudo apt install ros-humble-joint-state-publisher-gui`

### 5. DDS：切 CycloneDDS 并配置（全链验证过 RMW, 勿用默认 Fast DDS 混跑）

```bash
# ① 切 RMW + 指定配置 (写进 ~/.bashrc):
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# ② ~/cyclonedds.xml (绑定 lo + 单播 Peers 兜底——设计要点见文件头注释; 样例=工程在用实件):
cat > ~/cyclonedds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# ③ lo 组播标志开机即丢, 补位 (已并入 test/rt_tune_boot.sh):
sudo ip link set lo multicast on
```

## 构建

```bash
cd <path-to>/UniStackBot_ws
colcon build --symlink-install
source install/setup.bash
```

## 运行

`robot:=<机型>` 必填（`piper` | `xarm7` | `stationlite` | `r1_7a`，未知机型显式报错）。

```bash
# 统一入口：chain 一参切三链 (mock|gz|mujoco)
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=piper

# 起链前清场（三链通用，exit 0=干净；永远单独执行，勿与 launch 同命令行）
ros2 run unistackbot_bringup chain_clean.sh

# 或分别起链
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro  # 可视化（换机型改 model 路径）
ros2 launch unistackbot_bringup control.launch.py robot:=piper      # mock（算法迭代）
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper gui:=false  # Gazebo Fortress
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper          # MuJoCo（真实动力学）

# 控制 demo（前置: 任一链已启动）
ros2 run unistackbot_demo demo_cartesian.py       # 末端位姿流 (up/axes/sweep/reject 四 pattern)
ros2 run unistackbot_demo demo_motion.py --hz 50  # 关节慢流 (JS ruckig 档填充)
ros2 run unistackbot_demo demo_joint_fullrate     # 关节满速流 (C++, =控制频率透传)
ros2 run unistackbot_demo demo_teleop_key.py      # 键盘遥操, 笛卡尔速度域 (关节域 = demo_teleop_joint.py)
ros2 run unistackbot_demo demo_points_playback.py --points-file <json> --seq @<序列名>  # 点位序列回放
```

命令通道：关节空间 `/joint_stream_controller/command`（`unistackbot_interface/JointCommand` 点流）；笛卡尔空间 `/cartesian_motion_controller/command`（PoseStamped, base 系）。双控制器经 `ros2 control switch_controllers` 互切。

## 测试

```bash
bash test/verify_robot.sh piper --with-gazebo --with-mujoco   # 链路级验收（一条命令）
bash test/fault_injection.sh --robot piper --chain mujoco     # 故障注入（F1-F7; --robot 选机型）
bash test/smoke_sim_control.sh                                # /sim_control 服务语义
sudo bash test/rt_tune_boot.sh                                # 开机必做（RT 调优 + lo 组播补位，先于一切基准）
bash test/rt_chain_bench.sh <标签> --robot piper [--chain mock|mujoco]  # RT 基准套件
bash test/rate_matrix_bench.sh <标签> --robot piper --chain mock --bus-hz 500 --cmd-hz 50  # 频率矩阵
```

组件库不进 colcon，g++ 直编（规范命令见 `unistackbot_common/ulog/README.md`）；hardware 域四子包（protocol/statemachine/bus/serial）是 colcon 包但单元测试同零 ROS、g++ 直编（范本 `unistackbot_hardware/serial_master/README.md`）。

## 常见问题

- **臂不动 / `Controller already loaded`**：残留进程冲突，`ros2 run unistackbot_bringup chain_clean.sh` 清场（三链通用, 单独执行）后重启；一次只跑一套栈。
- **`Failed to find a free participant index`**：杀链后立即起链所致——chain_clean 清进程 + 等租约再起；确认 `lo` 有 MULTICAST 标志（重启即丢，`sudo ip link set lo multicast on`——已并入 `test/rt_tune_boot.sh` 开机一条收拢）。
- DDS 统一 `~/cyclonedds.xml`（`.bashrc` 已 export）。

## 许可证

见 [LICENSE](LICENSE)。

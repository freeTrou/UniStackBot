# unistackbot_gazebo

Gazebo 集成层：world + launch + `/sim_control` gz 适配器。单链路（Fortress），controller_manager 活在仿真器的 ros2_control 插件里（无独立 ros2_control_node）。Classic 链已删（2026-09-17，EOL 死角，git 历史可回溯）。

| 链路 | 入口 | 状态 |
|---|---|---|
| Gazebo Sim (Fortress) | `ros2 launch unistackbot_gazebo ign.launch.py robot:=<机型>`（`gui:=true\|false`，`use_rviz:=true` 可选） | **现行** |

`robot` 为必填机型名（对应 `unistackbot_description/arms/<robot>/`），launch 内不写死任何型号。

## 启动示例

```bash
# 终端 1 —— 启动（gz_clean 必须单独一条执行，与 launch 绝不同行！）
cd <工作区根> && source install/setup.bash
ros2 run unistackbot_gazebo gz_clean.sh
ros2 launch unistackbot_gazebo ign.launch.py robot:=xarm7 gui:=false

# 终端 2 —— 让机械臂动（等终端 1 出现两行 "Configured and activated"）
source install/setup.bash
ros2 run unistackbot_bringup demo_motion.py
```

常用变体（只换参数）：`gui:=true` 开 Gazebo 图形界面；`use_rviz:=true` 无头仿真加 RViz；`robot:=piper` 换机型。

```bash
# 健康检查（可选）
ros2 control list_controllers   # 两个控制器都应 active
ros2 topic hz /joint_states     # ≈500 Hz = 链路健康

# 收工：终端 1 Ctrl+C 后，再单独跑一次 gz_clean.sh（ign server 常在 launch 退出后残留）
```

## 用法纪律

ign server 常在 launch 关闭后存活并毒化下一次启动；**一次只跑一套栈**（同名 controller_manager / robot_state_publisher 会互相污染）；`gz_clean.sh` 的 pkill 模式会击杀同一命令行里含 "ros2 launch" 字样的宿主，因此永远单独执行。

## 内置的坑规避（launch 里已写死，改动前先懂为什么）

- URDF 先压成**单行**再传：插件以 `--param robot_description:=<urdf>` 转发，rcl 解析器拒绝换行 → CM 起不来。
- 仿真器经**临时文件**（`-file`）拿 URDF，不走 `/robot_description` topic：iceoryx/SHM 下 TRANSIENT_LOCAL 锁存重发不可靠。
- **不覆盖** `CYCLONEDDS_URI`：本机 `~/cyclonedds.xml`（lo 绑定 + 单播 Peers）是 DDS 随机失联的修复配置。
- ign 链把 description 包的 share 根前置进 `IGN_GAZEBO_RESOURCE_PATH`：URDF→SDF 会把 `package://` 改写成 `model://`，Fortress 只认这个环境变量。

## Gazebo 对外接口速查（2026-09-16 在本机实测枚举）

- **ign-transport 原生**：话题 `/stats`（RTF/暂停态，已桥接为 `ros_gz_interfaces/WorldStatistics`）、`pose/info`、`scene/info` 等；服务 `control`（pause/resume/step，gz 适配器的落点）、`create`/`remove`/`set_pose`、`set_physics`、`enable/disable_collision`、`scene/graph` 等，全挂 `/world/unistack_world/` 下
- **ROS 桥**（`parameter_bridge`）：`/clock`、`/stats`、`/world/unistack_world/control`（世界控制服务，gz 适配器的通道）；传感器上线后按需加映射
- **进程内 System 插件**：`gz_ros2_control-system`（CM 宿主，主乘骑）；world 现挂 Physics/UserCommands/SceneBroadcaster——`Contact`（碰撞接触）与 `ForceTorque`（腕 FT）留待协作二期声明加载
- **reset 实测有毒（2026-09-16）**：Fortress 的 `WorldControl.reset{all}` 返回 success=true 但世界随即进入 negative-timestep 错误态、仿真停摆——适配器拒绝并引用此实证。`remove`+`create` 合成 reset 同样否决（CM 活在模型实体插件里，remove 即杀 CM）。Fortress 时代关节复位 = JTC 归零轨迹；Garden+ 才有可用原生 reset

## 关键文件

`src/sim_control_gz_node.cpp`（`/sim_control/*` 的 gz 适配器，**纯 ROS 构建无 ign 编译依赖**——pause/resume/step 经 launch 里 parameter_bridge 桥接的 `ControlWorld` 服务下发，reset/set_joint_state 拒绝并说明）、`worlds/empty_ign.world`（`<world name>` 为 `unistack_world`——共享 world 的名字，与机型无关，须与 `sim_control_gz_node` 的 `world` 参数一致）、`scripts/gz_clean.sh`。

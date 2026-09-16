# unistackbot_gazebo

Gazebo 集成层：world + launch。两条链路，controller_manager 都活在仿真器的 ros2_control 插件里（无独立 ros2_control_node）。

| 链路 | 入口 | 状态 |
|---|---|---|
| Gazebo Sim (Fortress) | `ros2 launch unistackbot_gazebo ign.launch.py robot:=<机型>`（`gui:=true\|false`，`use_rviz:=true` 可选） | **现行** |
| Gazebo Classic | `ros2 launch unistackbot_gazebo gazebo.launch.py robot:=<机型>` | EOL，留作参考 |

`robot` 为必填机型名（对应 `unistackbot_description/arms/<robot>/`），launch 内不写死任何型号。

## 用法纪律

```bash
ros2 run unistackbot_gazebo gz_clean.sh   # 每次启动前必跑：清残留进程
```

ign server 常在 launch 关闭后存活并毒化下一次启动；且**一次只跑一套栈**（同名 controller_manager / robot_state_publisher 会互相污染）。

## 内置的坑规避（launch 里已写死，改动前先懂为什么）

- URDF 先压成**单行**再传：插件以 `--param robot_description:=<urdf>` 转发，rcl 解析器拒绝换行 → CM 起不来。
- 仿真器经**临时文件**（`-file`）拿 URDF，不走 `/robot_description` topic：iceoryx/SHM 下 TRANSIENT_LOCAL 锁存重发不可靠。
- **不覆盖** `CYCLONEDDS_URI`：本机 `~/cyclonedds.xml`（lo 绑定 + 单播 Peers）是 DDS 随机失联的修复配置。
- ign 链把 description 包的 share 根前置进 `IGN_GAZEBO_RESOURCE_PATH`：URDF→SDF 会把 `package://` 改写成 `model://`，Fortress 只认这个环境变量。

## Gazebo 对外接口速查（2026-09-16 在本机实测枚举）

- **ign-transport 原生**：话题 `/stats`（RTF/暂停态，已桥接为 `ros_gz_interfaces/WorldStatistics`）、`pose/info`、`scene/info` 等；服务 `control`（pause/resume/step，gz 适配器的落点）、`create`/`remove`/`set_pose`、`set_physics`、`enable/disable_collision`、`scene/graph` 等，全挂 `/world/unistack_world/` 下
- **ROS 桥**（`parameter_bridge`）：`/clock`、`/stats`；传感器上线后按需加映射
- **进程内 System 插件**：`gz_ros2_control-system`（CM 宿主，主乘骑）；world 现挂 Physics/UserCommands/SceneBroadcaster——`Contact`（碰撞接触）与 `ForceTorque`（腕 FT）留待协作二期声明加载
- **已评估否决**：用 `remove`+`create` 合成 reset——CM 活在模型实体的插件里，remove 即杀 CM，重生后控制器全死。Fortress 关节级瞬移维持"拒绝"（Garden+ 原生支持）

## 关键文件

`src/sim_control_gz_node.cpp`（`/sim_control/*` 的 gz 适配器，编译进本包——pause/resume/step 翻译成 ign 世界服务，reset/set_joint_state 在 Fortress 无原生等价直接拒绝；可选编译，CMake QUIET 探测 ign 库）、`worlds/empty_ign.world`（`<world name>` 为 `unistack_world`——共享 world 的名字，与机型无关，须与 `sim_control_gz_node` 的 `world` 参数一致）、`worlds/empty.world`（Classic）、`scripts/gz_clean.sh`。

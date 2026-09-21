# unistackbot_sim_control/ — 仿真集成组 (容器目录)

本目录收纳全部三种仿真的集成, **三个独立 colcon 包** (维持三包: 依赖各自声明、
可选择性构建; 与未来真机总线后端包 SocketCAN/EtherCAT master 同一"框架核心 +
平行集成包"模式)。统一性不靠包结构: `/sim_control` 契约 + 共享
`unistackbot_bringup/config/<robot>_controllers.yaml` + 三个 launch 同款 `robot:=` 人机工学。

| 子目录 | 包名 | 角色 |
|---|---|---|
| `core/` | `unistackbot_sim_control` | 仿真核心: SimControlHardware 插件 (kinematic 后端) + `/sim_control` 契约 + write()/read() 防线 (真机驱动同型照抄) |
| `gazebo/` | `unistackbot_gazebo` | Gazebo Fortress 链: `ign.launch.py` + world + gz 适配器 (`sim_control_gz_node`) + `gz_clean.sh` |
| `mujoco/` | `unistackbot_mujoco` | MuJoCo 链 (mujoco_ros2_control 0.1.2): `mujoco.launch.py` |

起链 (三选一, 详见各包 README):

```bash
ros2 launch unistackbot_bringup control.launch.py robot:=piper    # mock (kinematic)
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper         # Gazebo Fortress
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper      # MuJoCo
```

注: mock 链的 launch 住 bringup (顶层启动编排); 目录名 != 包名 (`core/` 的包名是
`unistackbot_sim_control`, 以 `package.xml` 为准)。

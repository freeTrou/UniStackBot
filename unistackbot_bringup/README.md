# unistackbot_bringup

顶层启动包：负责拉起整机各节点、加载控制器配置，是三层架构的**指挥层**。

## 用法

```bash
# mock 控制链路（无仿真器）：standalone controller_manager + SimControlHardware
ros2 launch unistackbot_bringup piper_control.launch.py        # use_rviz:=true|false

# 发一段演示往返轨迹（JTC action）
ros2 run unistackbot_bringup piper_demo_motion.py
```

Gazebo 环境不用本包的 launch，改用 `unistackbot_gazebo` 的入口（见其 README）。

## 关键文件

| 文件 | 说明 |
|---|---|
| `launch/piper_control.launch.py` | mock 链路入口：robot_state_publisher + ros2_control_node + 两个 spawner |
| `config/piper_controllers.yaml` | **全工程唯一**控制器配置（500 Hz；JTC position 接口，7 关节），mock 与 Gazebo 共用 |
| `scripts/piper_demo_motion.py` | 演示轨迹脚本（`ros2 run`） |

## 注意事项

- JTC 的 goal 必须列出**全部 7 个关节**，子集会被拒绝（除非开 `allow_partial_joints_goal`）。
- 一次只跑一套 launch 栈；切换环境前先 `ros2 run unistackbot_gazebo gz_clean.sh` 清残留，否则同名节点会污染新启动。
- 本包在 `package.xml` 里把所有被编排的包声明为 `exec_depend`，新增运行时依赖的包记得同步登记。

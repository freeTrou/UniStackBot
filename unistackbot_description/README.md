# unistackbot_description

机器人描述横切层：URDF/Xacro 模型 + mesh 资源 + RViz 配置。被所有层经 URDF/TF 消费，不依赖任何其他功能包。

## 目录约定

```
arms/<robot>/urdf/     模型文件
arms/<robot>/meshes/   visual/ + collision/ 两套网格
```

mesh 路径一律用 `package://unistackbot_description/...` 前缀，保证 `--symlink-install` 后可解析。**新机器人照 Piper 的模式抄。**

## Piper（参考模型）

| 文件 | 职责 |
|---|---|
| `piper.urdf.xacro` | 顶层入口，声明 `use_gripper` / `use_ros2_control` / `use_world` / `use_gazebo` 四个 arg |
| `piper_macro.xacro` | `piper_arm` 宏（base_link → link6）；`use_world:=true` 时加 world 并固定 |
| `piper_gripper_macro.xacro` | 夹爪宏：prismatic `gripper` 关节 + 两个 mimic 手指 |
| `piper_ros2_control.xacro` | `<ros2_control>` 块，插件随 `use_gazebo` 三态切换 |
| `piper.urdf`、`piper_with_gripper.xacro` | 平坦（非宏）变体，备用 |

## 用法

```bash
# 可视化（默认 ros2_control 关闭，joint_state_publisher_gui 拖关节）
ros2 launch unistackbot_description display.launch.py
# 参数: model:=<xacro> use_gripper use_ros2_control use_world gui rviz
```

## 注意事项

- `use_gazebo` 是三态：`false` → SimControlHardware；`true`/`classic` → Gazebo Classic；`ign` → Gazebo Sim (Fortress)。
- `gripper_link` 虽近乎空载也必须保留 `<inertial>` —— Gazebo 会丢弃无惯量 link，连带删掉 JTC 依赖的 `gripper` 关节。
- 关节 / 接口改动时，同步 `piper_ros2_control.xacro`（限位、`max_velocity`、mimic 参数）与 bringup 的 `piper_controllers.yaml`，三处是同一契约。

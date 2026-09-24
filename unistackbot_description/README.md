# unistackbot_description

机器人描述横切层：URDF/Xacro 模型 + mesh 资源 + RViz 配置。被所有层经 URDF/TF 消费，不依赖任何其他功能包。

## 目录约定

```
arms/<robot>/urdf/     模型文件
arms/<robot>/meshes/   visual/ + collision/ 两套网格
arms/<robot>/mujoco/   MJCF 资产（MuJoCo 链；URDF 惯量/结构改动后须再生成，见其 README）
arms/<robot>/ik/       IK 种子库等生成资产（生成器在 unistackbot_algorithm/dls_ik/test/）
arms/<robot>/original/ 原始 URDF 留档（死档案 diff 基线，永不修改，见 arms/original_README.md）
```

mesh 路径一律用 `package://unistackbot_description/...` 前缀，保证 `--symlink-install` 后可解析。**新机器人照 Piper 的模式抄**；外部参考机型的 vendor 模式见 `arms/xarm7/`（上游来源/改动/许可记录在其 README）。

## Piper（参考模型）

| 文件 | 职责 |
|---|---|
| `piper.urdf.xacro` | 顶层入口，声明五个 arg：`use_gripper` / `use_ros2_control` / `use_world` / `use_gazebo` / `headless`（mujoco 链专用） |
| `piper_macro.xacro` | `piper_arm` 宏（base_link → link6）；`use_world:=true` 时加 world 并固定 |
| `piper_gripper_macro.xacro` | 夹爪宏：prismatic `gripper` 关节 + 两个 mimic 手指 |
| `piper_ros2_control.xacro` | `<ros2_control>` 块，插件随 `use_gazebo` 三态切换 |
| `piper.urdf`、`piper_with_gripper.xacro` | 平坦（非宏）变体，备用 |

## 用法

```bash
# 可视化（默认 ros2_control 关闭，joint_state_publisher_gui 拖关节）
# model 必填（机型无默认值），指向对应机型的 xacro 入口
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro
# 参数: model:=<xacro>(必填) use_gripper use_ros2_control use_world gui rviz
```

## 注意事项

- `use_gazebo` 是四态：`false` → SimControlHardware（mock）；`true`/`classic` → Gazebo Classic（链已删，仅惰性保留）；`ign` → Gazebo Sim (Fortress)；`mujoco` → mujoco_ros2_control（MJCF 资产路径经 `mujoco_model` 参数）。
- `gripper_link` 虽近乎空载也必须保留 `<inertial>` —— Gazebo 会丢弃无惯量 link，连带删掉控制器配置依赖的 `gripper` 关节。
- 关节 / 接口改动时，同步 `<robot>_ros2_control.xacro`（限位、`max_velocity`、mimic 参数）与 bringup 的 `<robot>_controllers.yaml`，三处是同一契约；改了惯量/结构还要再生成 MJCF（`arms/<robot>/mujoco/README.md` 有命令）。
- gz 链注意：关节静置压在限位上会被 ODE 限位约束咬死（上游 gz_ros2_control #165）——ign 分支易冻关节的 state_interface 已加 `initial_value` 略离限位线；运行时仍避免命令关节精确停限位静置。
- 现有机型：`piper`（6 关节+夹爪，参考模型，IK 裁决见 `arms/piper/ik_decision_card.md`）、`xarm7`（7 关节，构型判定与 IK 裁决见 `arms/xarm7/ik_decision_card.md`）。

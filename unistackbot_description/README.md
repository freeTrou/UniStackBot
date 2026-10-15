# unistackbot_description

机器人描述横切层：URDF/Xacro 模型 + mesh 资源 + RViz 配置。被所有层经 URDF/TF 消费，不依赖任何其他功能包。

## 目录约定（2026-10-14 起按本体形态分类）

```
arms/<robot>/                  固定基座机械臂
mobile_manipulators/<robot>/   带移动底盘的操作机器人
humanoids/<robot>/             人形机器人（双足; 暂空）

<任一类别>/<robot>/urdf/       模型文件
<任一类别>/<robot>/meshes/     visual/ + collision/ 两套网格
<任一类别>/<robot>/mujoco/     MJCF 资产（MuJoCo 链; URDF 惯量/结构改动后须再生成, 见其 README）
<任一类别>/<robot>/ik/         IK 种子库等生成资产（生成器在 unistackbot_algorithm/dls_ik/test/）
<任一类别>/<robot>/original/   原始 URDF 留档（死档案 diff 基线, 永不修改, 见 arms/original_README.md）
```

机型发现 = 各类别目录下 glob（launch/verify 已按 `CATEGORIES = (arms, mobile_manipulators,
humanoids)` 搜索, 无 robot→类别映射表）。mesh 路径一律用 `package://unistackbot_description/...`
前缀。**新机器人照 Piper 的模式抄**。

## 机型登记表（当前模型 + 开来源; 详录在各机型目录 README/original/）

| 机型 | 类别 | 形态 | 开源来源 | 导入日期 | License |
|---|---|---|---|---|---|
| **piper** | arms/ | 6-DoF 球腕臂+夹爪, 参考模型 | [Piper 社区 ROS 包](https://github.com/Piper-Lang)（社区发起方 AgileX 系; 提取提交 `94fea82`） | 2026-09 早期 | 见 `original/` |
| **xarm7** | arms/ | 7-DoF 臂, UFACTORY | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)（`xarm_description` 包; 提取提交 `e7e7201`） | 早期 | BSD（`LICENSE.xarm_ros2`） |
| **r1_7a** | arms/ | 宇树 R1-7A 七轴冗余臂 | [宇树官方支持页 R1 Robotic Arm](https://support.unitree.com/home/zh/R1-7a/R1%20Robotic%20Arm)（R1_7A SDK 内置 URDF V1.1-0915; 本地资料包 `/home/work/R1_7A_DOC`） | 2026-09-29 | 宇树 SDK 条款 |
| **r1_pro** | mobile_manipulators/ | Galaxea R1 Pro: 三轮矢量底盘+4-DoF 旋转腰+双 7-DoF 臂+g1z 双指爪 | Galaxea 官方 `urdf_r1pro_g1z_2026` 资产包（`original/` 留档, 包名 `r1pro_urdf` v1.0.0; 官方仓库地址待补） | 2026-10-14 | BSD（`original/package.xml` 声明; 包内无 LICENSE 文件） |
| **galbot_s1** | mobile_manipulators/ | Galbot S1: 四轮独立转向底盘+升降柱+双 7-DoF 臂 | [GalaxyGeneralRobotics/galbot_s1_description](https://github.com/GalaxyGeneralRobotics/galbot_s1_description)（官方描述包; 含官方 MJCF） | 2026-10-14 | Apache-2.0 |

> 新机型接入时**本表必须同步登记**（来源 URL + 提取提交/日期 + License 三要素）——
> 这是开源合规与回溯的单一入口；死档案与提取方法见 `arms/original_README.md`。
> mobile_manipulators 两款（2026-10-14）尚在 original 留档阶段, xacro/yaml/MJCF 待接入。

## Piper（参考模型, 新机型照此抄）

| 文件 | 职责 |
|---|---|
| `piper.urdf.xacro` | 顶层入口，声明五个 arg：`use_gripper` / `use_ros2_control` / `use_world` / `use_gazebo` / `headless`（mujoco 链专用） |
| `piper_macro.xacro` | `piper_arm` 宏（base_link → link6）；`use_world:=true` 时加 world 并固定 |
| `piper_gripper_macro.xacro` | 夹爪宏：prismatic `gripper` 关节 + 两个 mimic 手指 |
| `piper_ros2_control.xacro` | `<ros2_control>` 块，插件随 `use_gazebo` 四态切换 |
| `piper.urdf`、`piper_with_gripper.xacro` | 平坦（非宏）变体，备用 |

## 用法

```bash
# 可视化（默认 ros2_control 关闭, joint_state_publisher_gui 拖关节）
# model 必填（机型无默认值）, 指向对应机型的 xacro 入口
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro
# 参数: model:=<xacro>(必填) use_gripper use_ros2_control use_world gui rviz
```

## 注意事项

- `use_gazebo` 是四态：`false` → SimControlHardware（mock）；`true`/`classic` → Gazebo Classic（链已删，仅惰性保留）；`ign` → Gazebo Sim (Fortress)；`mujoco` → mujoco_ros2_control（MJCF 资产路径经 `mujoco_model` 参数）。
- `gripper_link` 虽近乎空载也必须保留 `<inertial>` —— Gazebo 会丢弃无惯量 link，连带删掉控制器配置依赖的 `gripper` 关节。
- 关节 / 接口改动时，同步 `<robot>_ros2_control.xacro`（限位、`max_velocity`、mimic 参数）与 bringup 的 `<robot>_controllers.yaml`，三处是同一契约；改了惯量/结构还要再生成 MJCF（机型目录 `mujoco/README.md` 有命令）。
- gz 链注意：关节静置压在限位上会被 ODE 限位约束咬死（上游 gz_ros2_control #165）——ign 分支易冻关节的 state_interface 已加 `initial_value` 略离限位线；运行时仍避免命令关节精确停限位静置。
- IK 裁决卡：piper/xarm7/stationlite 各有 `ik_decision_card.md`（机型目录内）。

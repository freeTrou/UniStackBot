# mos_p20_description —— Lumos MOS P20 机型资产（产品域）

**状态（2026-11-01, Phase 1 装配完成 + Phase 4 验收全绿）**：URDF + meshes + xacro（backend 可切 kinematic|udp_board）+ 控制器配置（js_arms/js_head/js_torso/js_gripper 四件式 + WB/CM/JTC inactive）+ 验收表齐备；mock 链 verify 29 PASS（4 FAIL = ign/mujoco 链指纹挂账，见下）。

| 项 | 值 |
|---|---|
| 来源 | `mos_urdf` @ main（2026-07-23 快照），urdf/mos_p20.urdf + meshes/mos_p20（26 文件 105MB） |
| 清理 | **52 处** `file:///root/work_space/...` 绝对路径 → `package://mos_p20_description/meshes/`（零残留） |
| 关节 | 33 关节（27 运动 + fixed）：臂 14 + 头 2 + 升降 1 + 舵轮 4 + 驱动轮 4 + 夹爪 2 + 固定件 |

## 限位对账差异档案（Phase 0A 实测，处置=保留 URDF 运行口径）

实测修正 ODE 旧账（原记"5 关节缺 limit"）：**position 限位全在且为设计值**（各不相同，非转换器默认）；真缺口是 **effort/velocity 全线为零**（仅夹爪 effort=3）。URDF 值与官方文档表（SDK 手册部位表）存在三处口径差，**保留 URDF（机器在跑的运行真值），差异记档**：

| 关节 | URDF | 官方文档表 | 判定 |
|---|---|---|---|
| 臂关节（14 个） | 粗值（如 shoulder_pitch ±2.75, roll [0,3.14]） | 精值（±2.7751, [-0.2618,3.4034]） | 文档为精值；差 ≤2.4%，待 HAL 标定 yaml 对齐（真机批次） |
| head_yaw | ±1.05 | ±90°（±1.5708） | **URDF 为准**——SDK 示例实证"P20 当前限制 ±1.05"（文档表是机构能力） |
| head_pitch | ±0.524 对称 | [-35°,+30°] 非对称 | URDF 为准（SDK 示例同口径 ±0.524） |
| 舵轮 ×4 | ±3.14 | ±155°（±2.7053） | URDF 偏宽 16%；NSteerKinematics 过零翻转不受影响，收紧待真机批次 |

effort/velocity 零值处置：仿真链不消费（`<ros2_control>` 块自带 max_velocity param，Phase 1 装配）；真机由总线侧/标定管。

## 目录

```
urdf/mos_p20.urdf.xacro   入口 (backend:=kinematic|udp_board, board_ip/board_port/bind_port)
urdf/mos_p20.urdf         清理版 (package:// 路径)
config/mos_p20_controllers.yaml    四件式 JS + WB/CM/JTC 配置
config/mos_p20_acceptance.yaml     验收登记表
meshes/                   26 文件 STL (visual/collision 同源)
original/                 死档案: 原样 URDF 快照 (diff 基线, 不安装)
```

## 后续批次挂账

- MJCF 未生成 / gz 分支未写 → verify 的 ign/mujoco 链指纹 4 FAIL 为挂账（arm7_test/stationlite 同款处置：mock 全绿即收口，链资产补齐后登记表回填）；
- 限位精值回填（等 HAL 标定 yaml 或 Lumos 确认口径）；
- udp_board 底盘 8 轮映射与板状态机（M4，当前 system 接口只认领臂/头/升降 19 关节）。

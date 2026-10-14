# r1_pro 原始档案（死档案, 导入时点快照）

- **来源**: [behavior-robot-suite/brs-ctrl](https://github.com/behavior-robot-suite/brs-ctrl) `assets/robot/r1_pro/`
  （Stanford BEHAVIOR Robot Suite, CoRL 2025 —— Galaxea R1 Pro 的 BRS 接入资产）
- **License**: MIT（见本目录 LICENSE, Copyright 2025 behavior-robot-suite）
- **拉取日期**: 2026-10-14; 本目录 = 原样快照, 禁止修改（diff 基线; git 是真版本管理, 见 `arms/original_README.md`）
- **内容**: `r1_pro.urdf` + `meshes/`（STL, 34MB）

## 结构指纹（拉取时解剖, 供后续 xacro 转换对照）

| 部位 | 关节 | 类型 | 备注 |
|---|---|---|---|
| 底盘 | wheel_joint1-3 + servo_joint1-3 | wheel=continuous(z), servo=revolute | 三轮独立转向矢量底盘 |
| 躯干 | torso_joint1-4 | revolute ×4, 全 z 轴(yaw) | **4-DoF 旋转腰**（扭转冗余, 无升降） |
| 双臂 | left/right_arm_joint1-6 ×2 | revolute ×12 | 双 6-DoF |
| 夹爪 | left/right_gripper_axis1-2 ×2 | prismatic ×4 | 平行爪 |
| 头/相机 | head/zed/t265 等 | 多为 fixed | 传感器挂点（22 个 fixed） |

计 48 joint / 49 link；非轮非腰非臂的关节多为传感器挂点（fixed）。
对应四件式控制器分割预案: 底盘(velocity 生态件) / js_arms(12) / js_torso(4) / CM(主臂链)。

# galbot_s1 原始档案（死档案, 导入时点快照）

- **来源**: [GalaxyGeneralRobotics/galbot_s1_description](https://github.com/GalaxyGeneralRobotics/galbot_s1_description)
  （Galbot 官方开源描述包, Apache-2.0）
- **License**: Apache-2.0（见本目录 LICENSE）
- **拉取日期**: 2026-10-14; 本目录 = 原样快照, 禁止修改（diff 基线; git 是真版本管理, 见 `arms/original_README.md`）
- **内容**: `galbot_s1.urdf` + `meshes/`（visual=**glb**, collision=stl, 17MB）+
  `mjcf_official/`（**官方 MJCF 全套**——含 meshes, MuJoCo 链策展的官方起点）

## 结构指纹（拉取时解剖, 供后续 xacro 转换对照）

| 部位 | 关节 | 类型 | 备注 |
|---|---|---|---|
| 底盘 | wheel_steering/driving_joint1-4 ×2 | steering=revolute, driving=continuous | 四轮独立转向矢量底盘（8 轮关节） |
| 躯干 | torso_lift_joint1 | **prismatic ×1** | **升降柱**（全机唯一 prismatic; 单位米） |
| 双臂 | left/right_arm_joint1-7 ×2 | revolute ×14 | 双 7-DoF |
| 其余 | 相机/挂点 | fixed 22 | 传感器挂点 |

计 类型分布 fixed22/revolute26/continuous4/prismatic1。
对应四件式控制器分割预案: 底盘(velocity 生态件) / js_arms(14) / js_torso(1, 升降) / CM(主臂链)。

## 已知导入事项

1. **visual mesh 是 glb**——本仓 URDF 管线只认 STL/OBJ：转换（gltf-pipeline/assimp）或 visual 直引 collision STL（piper "mesh 直引 collision STL 零复制"先例可参考）
2. **官方 MJCF 在 `mjcf_official/`**——MuJoCo 链可从官方 MJCF 策展（首次有官方 MJCF 可用的机型, 免手写七坑）
3. prismatic 升降柱: 限位/速度单位米（js_torso 域, URDF 参数天然兼容; 消费者勿假设 rad）

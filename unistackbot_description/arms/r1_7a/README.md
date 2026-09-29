# r1_7a — 宇树 R1-7A 七轴机械臂

官方 SDK URDF (V1.1-0915, SolidWorks 导出) 程序化转换导入, 2026-09-29。

- **构型**: base_link → Link7, 厂商关节名 **J1-J7 原样保留** (与真机 SDK/日志对账零翻译); 无夹爪
- **总质量**: ~6.1 kg (8 links); 限位/力矩 (60/60/33/33/33/10/10 N·m) 全套官方值
- **IK**: 7 轴冗余 → **dls** (xarm7 同款主场; 决策对齐 `arms/xarm7/ik_decision_card.md`)。
  **几何验证 (2026-09-29)**: 全交替垂直轴 (相邻轴点积全 0, 无任何平行段) ——
  非 Pieper (7 轴冗余无闭式) 且**非 SRS** (球肩 J3 锚距 J1/J2 轴 108mm 不共点;
  腕 J5/J6/J7 近共点但 J7→J6 轴偏 10mm 真偏置非舍入) —— 臂角半解析解路线不通,
  DLS 数值解是几何事实下的标准答案。冗余零空间价值: 避奇异/避障/肘形偏好
  (DlsIk 冗余偏好 + 契约 RedundancyPreference.psi 预留)。
- **零位压限位线**: J2/J4 下限=0 (stationlite j3 同款) —— mock/mujoco 实测未触发异常
- 资产: `original/` 官方包死档案 · `urdf/` xacro 三件套 (生成器逻辑见提交) ·
  `meshes/visual/` 官方 STL (视觉/碰撞共用, package:// 路径已规范化) ·
  `mujoco/r1_7a.xml` (stationlite 七坑纪律全套内建: joint pos=0+child body+type="mesh"+damping/力矩界+ts 1kHz+低档 kp)
- **验证 (2026-09-29)**: mock + mujoco 双链 4 控制器就位、demo_motion 50Hz、
  mujoco 零发散警告、静止段速度 ~1e-10 (导入一次过, 未踩 stationlite 任一坑)
- **真机**: RS485@6M 宇树 IM 协议 MIT 原生 (protocol 包已实现 256 oracle) —
  资料包 `/home/work/R1_7A_DOC` (SDK/说明 PDF/电机 demo); 重力补偿/拖动示教见真机架构 §6A
- 待办: acceptance.yaml (A-G 验收) · RT 基线 · 与 xarm7 对比的 DLS AB 账

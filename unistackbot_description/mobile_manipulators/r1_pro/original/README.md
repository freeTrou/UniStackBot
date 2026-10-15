# r1_pro 原始资产留档

- **包**: `r1pro_urdf` v1.0.0（Galaxea 官方 `urdf_r1pro_g1z_2026` 资产包, 2026-10-14 导入）
- **License**: BSD——**仅 `package.xml` 内声明, 上游包不带 LICENSE 文本文件**（如后续找到
  官方 LICENSE 原文或仓库地址, 放回本目录并更新 `unistackbot_description/README.md` 登记表）
- **内容**: 官方 URDF + collision/visual meshes + 骨架 package.xml/CMakeLists（上游占位符未改）
- **提取关系**: `../urdf/r1_pro_macro.xacro` 程序化提取自本包 URDF（数字零手抄,
  覆盖控制域 18 关节 = 腰 4 + 双臂 7×2; 轮/夹爪/传感器挂点不在控制域宏内）
- **历史**: 2026-10-14 早先曾以 Stanford BRS `brs-ctrl` assets（MIT, 双 6-DoF 臂版本）入库,
  同日换源为官方 g1z 包（双 7-DoF）——旧档案连同其 MIT LICENSE 一并移除, 勿混用血统

导入方法与目录纪律见 `docs/guides/robot_import_sop.md` 与 `arms/original_README.md`。

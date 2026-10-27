# arm7_test original/ 导入留档（死档案 diff 基线）

| 文件 | 内容 |
|---|---|
| `arm7_test_cad2urdf_原始导出.urdf` | cad2urdf 工具原始产物（`local_mvj4nk13_i7vcsf_urdf_stl.zip` 内 robot.urdf 原样，中文 link/joint 名、占位限位 ±1.57、壳估算惯量）——**导入时点基线，勿改** |
| `arm7_test_导入期工作本.urdf` | 导入期的工作 flat URDF（原始件 + 下列全部手工变换），已被 `urdf/arm7_test_macro.xacro` 取代（2026-10-26 审核修 F7 消双源：macro = 唯一事实源）。留档供变换追溯 |

## 原始件 → 工作本的变换账（全部发生于 2026-10-22）

1. `robot name` generated_robot → arm7_test；
2. link/joint 中文改名：基座→base_link、1关节/2关节/关节3..7→link1..link7；*_joint→joint1..joint7；
3. mesh 相对路径 → `package://unistackbot_description/arms/arm7_test/meshes/visual/`；
4. 限位占位 ±1.57 → 用户提供的角度真值（J1 -130~200 / J2 复核改向 -50~165 / J3 ±150 待确认 / J4 -65~150 / J5 ±150 / J6 ±20 / J7 ±110）；effort/velocity 仍占位 10/5；
5. bench 测试台架 link + bench_mount fixed 关节（identity）加入为根；
6. bench_mount origin 后改 `xyz="0 -0.09 0"`（臂随抱板外移，2026-10-23）。

厂商回修正版模型（真限位表/质量属性/法兰）到位后：diff 基线在原始导出件，新模型走全新导入，不在此链上叠改。

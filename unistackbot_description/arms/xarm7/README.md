# xarm7 — UFACTORY xArm7 参考模型（vendor）

7 轴参考臂：三链冒烟（P0.1）与数值 IK 骨架（P1.4）的目标机型。原厂商品臂，构型/负载定位与公司 7 轴业务预期最接近。

## 来源与许可

- 上游仓库: https://github.com/xArm-Developer/xarm_ros2 （`xarm_description` 包）
- vendored commit: `62936f7ea1846a85f7350de2c4c18f39e6d19715`（2026-08-11）
- 许可: BSD-3-Clause，全文见 [LICENSE.xarm_ros2](LICENSE.xarm_ros2)——**保留版权声明，再分发时不可移除**；mesh 商用复用同样受此约束

## 移植内容与改动

| 本目录文件 | 上游位置 | 改动 |
|---|---|---|
| `urdf/xarm7_macro.xacro` | `urdf/xarm7/xarm7.urdf.xacro` | 改名（让位给本包入口命名约定）；`$(find xarm_description)/config/` → `$(find unistackbot_description)/arms/xarm7/config/`（2 处）；`collision_dir` 由 visual 直用改为 `xarm7/collision` 凸包（2026-09-16，见下） |
| `meshes/xarm7/collision/*.stl`（8 个） | — | **生成**：对上游 visual mesh 逐 link 求凸包（trimesh），面数降 3–5 倍、体积比 1.18–1.77——供 MoveIt 碰撞检查与 gz 物理链，避免高面数 visual mesh 直接做碰撞体 |
| `urdf/xarm7.urdf.xacro` | — | **新写**（入口，模式对齐 `arms/piper/urdf/piper.urdf.xacro`） |
| `urdf/common/common.link.xacro` | 同名 | 原样 |
| `urdf/common/common.material.xacro` | 同名 | 原样 |
| `meshes/xarm7/visual/*.stl`（8 个） | 同名 | 原样（visual 兼作 collision，上游即如此） |
| `meshes/end_tool/collision/end_tool.stl` | 同名 | 原样（link7 默认碰撞体引用） |
| `config/link_inertial/xarm7_type7_HT_BR2.yaml` | 同名 | 原样（上游默认惯性参数） |
| `config/kinematics/default/xarm7_default_kinematics.yaml` | 同名 | 原样（DH 标称参数） |
| `ik/seed_lib_xarm7.txt` | — | **生成**（本仓库资产，非移植）：IK 种子库 v1.1（12000 条 = v1.0 8000 + 姿态感知增量 4000），`unistackbot_controller/test/gen_seed_library.py` 生成 + `test/seed_lib_incremental.py` 增量（指纹头在文件内）；消费方 `DlsIk::loadSeedLibrary`，换臂重生成 |

**未移植**（保持体积与链路干净）：gripper/vacuum_gripper mesh（22MB+）、`xarm7_1305` 新版模型（11MB，`model_num>=1305` 时才用）、`xarm7.ros2_control/transmission/gazebo.xacro`（我们三条链自带插件体系，不引 `uf_robot_hardware`）、`xarm_device_macro.xacro`（全型号入口，由本包入口替代）。

注意：`xarm7_macro.xacro` 里 `model_num>=1305`、`mesh_suffix=='dae'` 等分支在本包不可达（未 vendor 对应 mesh），入口固定 `mesh_suffix=stl`、`model_num` 缺省 0。

## 使用

```bash
xacro $(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro use_world:=true
```

关节: `joint1`–`joint7`（revolute，7 个），根链 `link_base` → `link7`。限位/惯性来自上游 YAML 标称值。

## 状态

- [x] vendor + 路径改写 + xacro 展开 7 关节验证（check_urdf 通过，mesh 全解析）
- [x] RViz 可视化（`display.launch.py model:=` 直指本机型，2026-09-15）
- [x] `xarm7_ros2_control.xacro` 三模式插件切换 + `unistackbot_bringup/config/xarm7_controllers.yaml`（2026-09-15 展开/落盘验证）
- [x] 构型判定与 IK 路线裁决 → **[ik_decision_card.md](ik_decision_card.md)**（含 ssik 实测、oracle 三件套、机械设计输入）
- [x] mock 链接入（`control.launch.py robot:=xarm7`，JTC 7 关节轨迹到位，2026-09-15）
- [x] Fortress 链接入（`ign.launch.py robot:=xarm7 gui:=false`，物理仿真轨迹到位，2026-09-15）
- [x] 机型参数化验收脚本 `test/verify_robot.sh <robot> [--with-gazebo]`（xarm7 13/13、piper 9/9 全过，2026-09-15）
- [x] 碰撞几何凸包化（2026-09-16，MoveIt2 前置地基）
- [ ] `smoke_sim_control.sh` 深度服务断言（mimic/set_joint_state 语义）仍为 piper 关节表，待参数化

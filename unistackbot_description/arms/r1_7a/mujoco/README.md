# r1_7a MuJoCo 资产 (r1_7a.xml)

宇树 R1-7A 七轴臂的 MJCF。URDF→MJCF 转换 + 手工策展，纪律全部来自
`arms/stationlite/mujoco/README.md` 调修档案（手写 MJCF 前必读）。

## 调修档案 (2026-10-12 收口)

### 坑1: 默认 Euler 积分器下位置伺服 kp≥300 即失稳
J4-J7 |v|max 冲 30-40 rad/s 极限环。**修 = `integrator="implicitfast"`**——
Euler 只隐式积分 joint passive damping，执行器 kv（dampratio 生成）不隐式；
implicitfast 使 kv 进隐式求解，高 kp 才稳得住。kp 重定与切积分器是同一件事
的两面，**必须同时**（kp 拉高不切 = 必炸；只切不拉高 = 伺服偏软）。
kv 不写显式值，用 `dampratio="1.0"`（临界阻尼，kv=2ζ√(kp·m) 自动配平——
调参只动 kp 一个旋钮；注意 m 按 qpos0 有效惯量算、不含 joint damping/frictionloss，
若被动阻尼不可忽略 dampratio 可能需 <1，本模型 damping=0.1 影响小未需要）。

### 坑2: 初版 kp=50~500 伺服偏软
CM up 流误差峰值 4.3mm / 稳态垂差 1.876mm（J2 扛整臂重力仅 500），视觉
表现为跟踪滞后晃动。**修 = 按重力力矩/容许角误差重定**（piper 同链 200~24000
参照）：J1-J7 kp 2000/8000/6000/3000/800/300/300。

### 坑3: J2/J4 零位压限位线
限位下限=0，启动位恰在限位上（piper/stationlite 同款已知）。mujoco 链未触发
咬死（gz 链同款坑见 r1_7a_ros2_control.xacro ign 分支 initial_value=0.01）。

### 坑4: 全关节 damping=0.1 + frictionloss=0.1 + actuatorfrcrange
stationlite 档案 #1（QACC 发散防线）标配，导入即带。

## 验收数据 (2026-10-12, kp 终态 2000/8000/6000/3000/800/300/300)

| 项 | 调参前 (50~500) | 调参后 (终态) |
|---|---|---|
| CM up 稳态误差 | 1.876mm | **0.711mm**（改善 2.6×, 进 1mm 线） |
| 大幅 sweep (r1_7a_demo_cartesian) | 不可用（伺服偏软） | **14 腿全收敛 0 拒, err 0.25-0.97mm 全进 1mm** |
| 静态振动 (agile 激励后) | 抖动 | **零** (全关节 \|v\|=0.0000 rad/s) |
| 极限环 | kp≥300 即失稳 | 无 |
| 关节 demo 跟踪 | — | 指令差 ≤0.03 rad, 换构型 J6 带内到位 |

## 决策记录

- **gravcomp 不上**：0.711mm < 1mm 容差线，按 piper 判据（kp 够硬不需要）；
  stationlite 是上 gravcomp 的先例（link2-5 `<body gravcomp="1">`），若未来
  加负载/加链节导致 sag 破线，gravcomp 是第一后备。
- **timestep=0.001**：stationlite 伺服振动案的定版（谱主频钉奈奎斯特教训）。
- 基线包: /tmp r1_7a_mujoco_smooth_baseline_20261010_174620（重启丢失，账在案）。

## 再生

URDF 改动后须再生成 MJCF（`mujoco/README.md` 通用命令），策展差异以本文件
调修档案为准对照。mesh 直引 `../meshes/visual/` STL。

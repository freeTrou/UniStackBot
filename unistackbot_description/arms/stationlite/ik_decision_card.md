# stationlite IK 决策卡（双臂机 · 左臂先验）

## 0. 一句话结论

stationlite 左臂 = **三平行轴 6 轴**（j2/j3/j4 严格平行，Pieper 条件形式二成立，几何实测实锤），
**闭式解析解存在**，且**已上线**——2026-10-21 B 线立项落地：IKFast 生成物
（`analytic_stationlite_ikfast.cpp`，sl_ 前缀隔离）+ 平行轴指纹选拒 + 周期折叠
（wrapToLimits）+ 适配层公共件（analytic_common，与 piper 共享）。现行 yaml
`ik_solver: analytic_stationlite`（2026-10-21 切换，mock 34/1 存量 ign 缺口）；
数值解 trac_ik 为 fallback。历史：DlsIk（2026-09-29 首验）→ trac_ik（2026-10-08
全量换装）→ 解析解上线（2026-10-21）。

## 1. 构型判定（几何实测，2026-09-29）

数据源：MuJoCo 零位 FK 关节轴向/锚点（等价 `urdf_fk::jointAxesAtZero()`）。

- j2/j3/j4 轴向点积 = **1.0000 严格平行**（零位构型，j2/j3/j4 均为 y 向）
- j5(z)/j6(x) 垂直但**异面**：j6 锚点距 j5 轴 25.5mm，j5/j6 交点距 j4 轴 **78.1mm**
- **判定：非球腕**（对照 piper 三轴共点实测 59µm）；**Pieper 形式二（三相邻轴平行）成立
  → 闭式解析解存在**
- 有趣的同一枚硬币：零位 j234 共面 = **内部奇异**（demo_cartesian 落回起点被
  NEAR_SINGULAR 诚实拒的根源）——奇异性质与解析可解性同源，都是这个平行轴构型
- **代码回验（解析解立项时做）**：平行轴指纹 `check_parallel_axes`（j2/j3/j4 轴向点积
  ≈1 + 共面距离 ≤ 容差），照抄 piper 的 `check_spherical_wrist` 模式（init 选拒拒启）；
  测定方法即本节数据脚本

## 2. 双臂现状与边界

- CM 当前**只配左臂**（`base_link: left_armbase → tip_link: left_linkgripper`，yaml 显式）
- 右臂 CM 未配——双臂 = 两个 CM 实例（`cartesian_motion_controller_left/right`）还是
  单 CM 多链，是**未裁决的独立工作项**（对齐真机架构文档"双臂控制形态"节），不随本卡
- 左右臂几何镜像同构 → 解析解对右臂同样成立（IKFast 按链生成，镜像 URDF 各一份）

## 3. 定位裁决（对齐 piper 卡 §2）

| 解 | 定位 |
|---|---|
| **DlsIk（现行）** | stationlite 当前链上默认 + 已验证；全机型通用 fallback |
| 解析解（待立项） | 触发 §5 时机后接入，成为推荐解；实现 `analytic_stationlite` |

## 4. 候选路径

| 路径 | 评估 |
|---|---|
| **IKFast 生成管线（piper 同款复用）** | **定路**：IKFast 符号求解不挑 Pieper 形式，平行轴照出闭式。管线全复用：`arms/piper/ik/analytic_piper/gen/`（podman+OpenRAVE 容器，三坑配方见其 README）、规范化 URDF、vendored 生成物、适配层骨架、回代黄金测试（随机 q→FK→IK→FK）。**差异件**：指纹从球腕共点换平行轴（§1）；解数预期 4-8 组（底座 1-2 × 肘上/下 × 腕 2） |
| 手写圆柱坐标分解 | 排除：违反"数学成熟操作不自研"纪律（CLAUDE.md Conventions），IKFast 已覆盖 |
| 厂商 SDK | 未查证；按 piper 先例（不依赖厂商）默认排除 |

## 5. 触发时机（立项判据，2026-09-29 暂缓裁决）

当前 DlsIk 够用（mujoco 链 circle/部署/收尾全程通过；奇异邻域诚实拒绝正确工作）。
立项触发器（任一命中即开工，预估 ~1 天照抄管线）：

1. **真机接入前**——解析解的确定性（µs 零预算、无 ITERATION_LIMIT 偶发）对真机安全链价值最大
2. **需要全解枚举**——上层规划/避障/工作空间分析要全部分支（DLS 只给单解）
3. 奇异邻域需要**结构信息**（哪些分支在何处退化）而非仅拒绝

注意一个不收益：解析解**不消除零位奇异**——奇异是构型几何性质；解析解在那里同样解退化，
只是能枚举清楚"怎么退化"。真机部署的正解仍是 home 定义在非奇异部署位（demo circle
已演示正确边界：近零位停住）。

## 6. 验证口径（立项时对齐 piper 全套）

playbook 六阶段（`docs/guides/ik_validation_playbook.md`）+ §9 流水线照抄；类专属用例：

- 零位奇异邻域回代（解退化处的行为定义）
- j234 平面内/外目标的双分支（肘上/肘下）枚举正确性
- 限位过滤 + `kLimitEpsRad` 微容差（piper 零位压线同款问题预计在 j3 上限复现——
  零位 j3=0 恰在限位线上）
- 与 DlsIk 平级 AB 对比（同目标集：成功率/耗时/解距种子分布）

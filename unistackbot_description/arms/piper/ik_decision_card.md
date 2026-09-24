# piper IK 决策卡（6 轴参考机）

## 0. 一句话结论

piper = 球腕 6 轴（Pieper 条件成立，几何实测实锤），**闭式解析解存在**。定位裁决（2026-09-22）：
**6 轴用成熟解析解；数值解（DlsIk）定位 7 轴冗余臂 + 通用 fallback**。解析解来源**待选路**
（IKFast 推荐 vs 厂商 SDK 待查证），未立项。

## 1. 构型判定（几何实测，2026-09-22）

数据源：`urdf/piper_macro.xacro` joint4/5/6 的 origin/axis。

- joint5 原点相对 joint4 **零平移** → 轴4 与轴5 交于一点
- joint6 原点偏 0.091 m 但偏移方向在自身轴线方向上，垂直偏差仅 **8.8e-05 m**（建模舍入级）
  → 轴6 也过该交点
- **结论：joint4/5/6 三轴交于一点 = 球腕，Pieper 条件成立 → 闭式解析解存在**
- **代码回验**：`analytic_piper` 的球腕指纹判定（最小二乘共点残差）实测 **59µm ≤ 1mm 容差**
  （测试 `analytic_piper/test/run_wrist_check_test.sh`，含合成正/反例校准）

## 2. 定位裁决（2026-09-22 用户）

| 解 | 定位 |
|---|---|
| DlsIk（数值，Eigen SVD + 避限位 + 种子阶梯） | 7 轴冗余臂主场；全机型通用 fallback；现行链上默认 + AB 基准 |
| 解析解（待建） | 6 轴球腕臂的正解——接入后成为 piper 推荐解 |
| 用户 7 轴数值解（待接入） | `IkSolver` 接口预留位（5 硬契约 hpp） |

## 3. 候选路径（解析解来源，2026-09-22 用户裁决后收窄）

| 路径 | 评估 |
|---|---|
| **IKFast（OpenRAVE）自动生成** | **定路**：工业标准做法，URDF → C++ 闭式求解器自动生成，快且确定。门：跑一次生成器 + license 检查（OpenRAVE LGPL，生成代码分发条款需过目） |
| 厂商 SDK IK | **排除**（2026-09-22 用户裁决：不依赖厂商，走自建基础设施） |
| 自研闭式推导 | **排除**：违反"数学成熟操作不自研"纪律（CLAUDE.md Conventions） |

**工作重心随裁决转移**：不做"找现成"，做**基础设施 + 算法合理接入**——结构指纹自校验工具、
解析求解器脚手架、接入模式沉淀（接口契约 + yaml 选择 + 六阶段验证 + AB 对比）。

## 4. 接入方式（定稿）

- 实现 IkSolver 抽象接口 → CM 参数 `ik_solver: analytic_piper`（per-robot yaml 推荐解）
- **特化解 init() 自校验结构指纹**：初始化时重做 §1 的球腕三轴交点判定，不匹配 fail-fast 拒启
  （求解器最懂自己的数学前提，不做外部登记表）
- 组合矩阵：机型（launch `robot:=`）× 仿真链（`chain:` / `use_gazebo`）× 求解器（yaml `ik_solver:`）
  三轴正交——IK 在 CM 内，与后端无关
- 验证：`docs/guides/ik_validation_playbook.md` 六阶段（oracle 对拍 / TF 对拍 / 覆盖率）+
  与 DlsIk 平级 AB 对比；换装不改变任何链级流程

## 5. 状态

选路完成（IKFast），**脚手架已落地**（2026-09-22）：`unistackbot_algorithm/analytic_piper/` =
球腕指纹自校验（init 选拒）+ 诚实 NOT_READY 求解器；CM 分支 `analytic_piper` 可被 yaml 显式选中。
**剩余**：①规范化 URDF（88µm 归零）②podman 容器内 IKFast 生成 ③license 过目 ④闭式数学填入
solve()（ready 翻真）⑤六阶段 + AB 对比。

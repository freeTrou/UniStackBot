# 7 轴机械臂 IK 决策卡（以 xArm7 为参考机）

> 状态： 定稿（2026-09-15）
> 背景： 公司或立项 7 轴机械臂，UniStackBot 承担大部分开发。本文沉淀 IK 路线的全部裁决与实测依据——每条结论都有本仓库的实测数据或生产级实现背书，无一条来自教科书惯性。机械团队出第一版构型草案时直接引用 §5.4。
> 关联： 构型数据见本目录 `config/`；框架侧分层裁决见 `docs/architecture/hardware_framework_design.md` §14。

## 0. 一句话结论

**数值内核 + 种子阶梯 + 显式冗余参数 + 结构化失败码；机械设计阶段用 ssik 当构型 lint，换免费解析红利。**

---

## 1. 参考机构型判定（实测，非推断）

由 `config/kinematics/default/xarm7_default_kinematics.yaml` 数值计算关节轴几何（q=0 位形，相邻轴关系为构型无关量）：

| 关节对 | 夹角 | 轴线关系 | 偏置 |
|---|---|---|---|
| j1–j2 | 90° | 相交 | 0 |
| j2–j3 | 90° | 相交 | 0 |
| j3–j4 | 90° | 异面 | **52.5 mm** |
| j4–j5 | 90° | 异面 | **77.5 mm** |
| j5–j6 | 90° | 相交 | 0 |
| j6–j7 | 90° | 异面 | **76 mm** |

链长： 肩高 267 / 上臂 293 / 前臂 342.5 mm，最大伸展 ~0.75 m 量级。

**判定： 带球形肩的正交偏置 7R 链，不是 S-R-S**（需同时满足球形肩+肘无侧偏+球形腕，仅第一条成立）。交叉印证： ssik 5.0.0 对同一 URDF 自动分类为 `seven_r.spherical_shoulder_polished`，与手算一字不差。

后果： **arm-angle 解析路线排除**——不是工程取舍，是该构型上闭式解不存在。数值 IK 是唯一合理路径。

## 2. 概念基线：解析解 vs 数值解

| | 解析解 | 数值解 |
|---|---|---|
| 形式 | 公式，代入一次出 | 循环，迭代逼近 |
| 给你什么 | 全部解（全家福） | 种子附近的一个 |
| 失败报告 | 明确（如判别式<0=不可达） | 模糊（"不收敛"，原因未知） |
| 速度 | 快 | 也快（µs 级）——**速度不是区别** |

一般偏置 7R 无闭式解，与"一元五次方程无通用求根公式"同性质：解存在且可数值求出，但不存在有限项通用表达式。偏置 7R 的解集是隐式空间曲线。

**自运动曲线实测**（钉死末端位姿沿零空间追迹）： 曲线 210 步闭合，路径最小奇异值 σ₆≥0.157；自运动主体是 j1+j5 同步整圈缠绕，**j4 全程仅漂 0.10 rad（5.7°）**——肘部偏置使"定边三角形"退化为变形四连杆，S-R-S 式的"肘绕肩腕连线转圆"不存在。

## 3. 市场扫描结论（2026-09）

| 臂 | 构型 | 控制器 IK | 开源生态 IK |
|---|---|---|---|
| KUKA iiwa/Med | 教科书 S-R-S（刻意设计） | 闭源 | MoveIt KDL；arm-angle 文献载体 |
| Franka Panda/FR3 | 近 S-R-S 小偏置 | 闭源（libfranka 模型开源） | KDL + 社区解析解（He 2021 系） |
| Kinova Gen3 7DoF | 偏置 7R | 闭源 | KDL/TRAC-IK，无公开解析解 |
| UFACTORY xArm7 | 偏置 7R | 闭源固件，**种子数值解**（SDK 纯 RPC 已验证：`get_inverse_kinematics` → `core->get_ik`，带 `ref_angles` 种子 + `limited` 标志） | MoveIt 默认 KDL（配置未覆盖 kinematics 段） |
| ABB YuMi / Schunk LWA4P | 7R | 闭源 | ROS-I + KDL |

三层格局： ①控制器固件全行业闭源、普遍种子数值解（IK 是商业资产+安全栈）；②开源生态几乎清一色数值（KDL/TRAC-IK，**ikfast 不支持 7R 冗余链**）；③解析解只存在于"为解析而设计的几何"（iiwa）或"研究标准机+社区眼球"（Panda）。

澄清： **TRAC-IK 是纯数值**（KDL 牛顿 + SQP 并行 + 随机重启），无任何解析成分，"结合解析和数值"的说法是误传。

## 4. ssik 5.0.0 实测记录（2026-09-15）

| 项目 | 结果 |
|---|---|
| 从本目录 URDF 构建 | ✅ `seven_r.spherical_shoulder_polished`，诊断原话 "Covers uFactory xArm7" |
| 预构建 | ✅ 72 臂属实，`xarm7_ik` 在列 |
| FK 交叉对拍（vs 本仓独立 numpy 实现） | ✅ 平移差 2.5e-16 m（机器精度） |
| IK | ✅ 16 分支全闭合 1e-12 m，24 原始候选−8 限位淘汰=16，j4 限位全守，~6–10 ms |
| **带种子复现任意冗余构型** | ❌ `q_seed` 不改变输出——16 组固定在内部规范冗余参数（"锁末关节"配方）上 |
| 内部机制（`explain=True` 自曝） | 闭式球形肩配方出种子 + **LM 数值抛光**——即"混合法"的生产实现 |

**oracle 用法三件套**（ssik 只进测试套件，运行时不引入——Python/10ms/新库）：
1. 可达性真值： 不可达位姿 ssik 返回空+明确诊断 ↔ 数值法只报"不收敛"——失败语义测试的 ground truth
2. 分支骨架： 16 分支作分支选择逻辑的测试基准
3. 闭合自检用自己的 FK： `FK(IK(p))≈p` 自己算，不劳驾 ssik

## 5. 决策卡

### 5.1 算法层（P1.4）

1. **种子阶梯（混合法的免推导实现）**：
   ```
   流式跟踪   → 种子 = 上一周期解（连续性最好，零成本）
   冷启动     → 限位感知启发式种子 × N 随机重启（TRAC-IK 配方）
   构型若支持 → ssik 解析种子（免费午餐，见 5.4）
   ```
   逐级启用，渐进求解，不做全家族枚举。
2. **限位是目标函数的一部分**： H = w₁·限位中心距 + w₂·可操作度，第一版就在。教训： xarm7 j4 限位 −0.19~+3.93，**家位距下限仅 0.19 rad（11°）**——限位只 clip 不进目标，避奇异一甩肘即撞。默认假设限位不对称（布线/结构干涉所致是常态）。
3. **分支粘性**： 求解失败 → 沿解流形找最近有效解 → 上报；**绝不跳分支续跑**（分支跳变 = 关节瞬间大位移）。
4. **两级 IK**： 点 IK（位姿→关节角）上游 ~100Hz；将来 OTG 门若要 1kHz 笛卡尔伺服，用微分 IK（纯雅可比一步，无迭代，RT 安全）。同一雅可比，两种消费——"1ms 完成迭代"的市场痛点不进入本架构。

### 5.2 接口层（unistackbot_interface, P0.2）

- `JointCmd`： 目标位姿 + **冗余偏好枚举**（`PRESERVE`（默认，跟随种子）/ `LOCK_JOINT<i>` / `ARM_ANGLE(ψ)`）+ 步长限幅
- 返回： **结构化失败码**（`UNREACHABLE` / `NEAR_SINGULAR` / `ITERATION_LIMIT` / `LIMIT_CONFLICT`），不是布尔——数值法"不收敛"的歧义在接口层消解（显式错误流原则的 IK 落点）

四个独立证据支撑冗余参数显式化： ssik 锁末关节 / SSRMS 论文锁单关节 / GeoFIK 四种参数 / xArm `ref_angles`。

### 5.3 验证层（测试先于求解器本体）

现有家底： 独立 numpy FK（对拍 2.5e-16）、零空间追迹器、ssik oracle。验收用例：

1. FK 回代自洽（`FK(IK(p))≈p` 至 1e-9）
2. 分支匹配（数值解分支签名 ∈ ssik 16 分支）
3. 可达性标注（ssik 空解的位姿必须报 `UNREACHABLE`，不许干转）
4. 路径连续性（平滑轨迹的解流形连续，无跳变）
5. **j4 回归**（家位附近零空间运动不越下限——为公司臂量身定制）

### 5.4 机械设计输入（构型草案评审用）

**核心工具： 用 ssik 做构型 lint。** 每版 URDF 草案 → `ssik.Manipulator.from_urdf()` → 读自动分类与 `dispatch_reason`：

- 分类进已知闭式类（球形肩/球形腕家族）→ **绿灯**： 白得解析种子 + 永久 oracle，数值收敛半径也更好
- 掉到通用数值类 → **黄灯**： 非阻塞（行业全在数值上跑），但明确知道丢了什么

设计偏好（按性价比排序）：
1. **腕三轴汇交**（球形腕）——用几何买回解析可能性；代价是腕部结构挤、线缆难走
2. **限位尽量对称于家位**——不对称限位的算法代价见 §5.1-2
3. 肩部保持汇交（xarm7 已有此性质，保留成本低）
4. **全绝对式编码器**——省上电找零流程；零位偏差仍需零点标定消除（2026-09-17 八阶段对照审计补入）
5. **减速器类型进精度链评估**——谐波（零背隙）/RV（高刚度）/行星（有背隙），背隙直接进末端重复定位精度预算（同上）
6. **走线通道与关节热管理**——7 轴线缆贯穿全关节，运动干涉与疲劳是结构评审必查项（同上）

## 6. 参考资源

- He 2021, *Analytical IK for Franka Emika Panda*（几何法，8 解）—— [RG](https://www.researchgate.net/publication/357238256_Analytical_Inverse_Kinematics_for_Franka_Emika_Panda_-_a_Geometrical_Solver_for_7-DOF_Manipulators_with_Unconventional_Design) / 实现 [ffall007/franka_analytical_ik](https://github.com/ffall007/franka_analytical_ik)
- *Analytical IK for Moz1 NonSRS 7-DOF*（腕偏置+SEW 角，16 解）—— [arXiv 2511.22996](https://arxiv.org/abs/2511.22996)
- NASA 立体投影 SEW 角（arm-angle 失效区域的解法）—— [arXiv 2307.13122](https://arxiv.org/html/2307.13122v2) / [stereo-sew](https://github.com/rpiRobotics/stereo-sew)
- GeoFIK（q4/q6/q7/摆动角四种冗余参数）—— [arXiv 2503.03992](https://arxiv.org/abs/2503.03992)
- [ssik](https://github.com/personalrobotics/ssik)（本仓实测，见 §4）/ [frankik](https://libraries.io/pypi/frankik)
- [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK)（控制器 RPC 行为证据，`xarm_api.cc` `get_inverse_kinematics`）

# whole_body/ —— 全身/上身多任务解算缝 (契约 + 参考实现)

> **算法域架构定稿**（2026-11-01）: 本目录住 Pkg 1 (SDK) 的 `whole_body.hpp`（契约）+ Pkg 2 (参考实现) 的三个实现示例。
> 物理拆分后 `whole_body.hpp` 留 `unistackbot_algorithm`（SDK）; 实现文件迁 `unistackbot_algorithm_solvers`。
> 详见 `docs/architecture/algorithm_architecture.md`（normative）。

## 给算法作者的三步（维护者入口，从这读起）

如果你是带着自己求解器算法来的（如 mos arm_controller 作者），迁移路径只有三步，全程**不需要学 ros2_control/launch/总线**：

1. **读契约**：`whole_body.hpp`（50 行）——你的数学要套进去的全部形状；你的线程/锁/模式机不搬（宿主壳已备等价物，对照表在你家仓 `mos/docs/00-lumos-unified-base-ode.md` 附录 E.2）。
2. **审适配层**：`upper_body_hqp_mos.hpp` 是框架侧已替你写好的适配件（mos 数学零改动）——你要做的是审它，特别是下面这张**行为变更对照表**（co-sign 签的就是这张）：
3. **做手术**：手术清单行级版在 ODE 附录 E.4（你家仓 docs/）；净效果是你的代码变小。

**行为变更对照表（重宿管 vs mos 原行为）**

| 项 | mos 原行为 | 重宿主行为 | 为什么 |
|---|---|---|---|
| 状态推进 | `setCartesianTarget` 内部开环积分，从不回看测量 | 每拍以实测重锚再解 | 闭环消漂（原行为真机漂移无收敛机制） |
| 速度平滑参考 | P2 平滑项拉向上拍**解** | 拉向**实测速度**（重锚副作用） | 与真实世界连续，语义增强 |
| ddq 注入 | 软停路径喂规划轨迹 | 恒传零（QP 不消费） | 软停职责移交急停域 |
| 不可达 | bool 混杂所有失败 | `UNREACHABLE` 独立分类（预检） | 上层可向用户反馈"目标给远了" |
| 目标映射 | 左右手双参数焊死 | goals **按名**映射左右槽（序无关） | 拓扑出签名（同实现可服务单臂手术后形态） |

**仿真层验证路径**：验证阶段**直接实例化**（`new UpperBodyHqpMos` + 喂 input 对拍），零 ROS 零插件；插件注册是 M6 壳的事，届时三行照 `ik_plugins/` 抄。链接需 mos 实现体（vendor 头只保编译）。

---



| 件 | 身份 |
|---|---|
| `whole_body.hpp` | **契约 v0.3 草案**（2026-10-29 三律 + v0.1 枚举返回 + v0.2 base_frame/dt_s + 10-30 v0.3 urdf 字符串与三条约定成文；待 co-sign 定稿，定稿前签名不消费不落地） |
| `upper_body_hqp_mos.hpp` | **参考实现示例一**: mos WbcSolver (双臂 HQP, pinocchio+qpOASES) 经适配层重宿主——验证两方法薄契约承载真数学, mos 数学零改动。生产归宿=外部仓/whole_body_plugins |
| `single_arm_dls.hpp` | **参考实现示例二**: 单臂速度级 DLS 伺服, 纯仓内栈 (urdf_fk+Eigen, 零 pinocchio)——与示例一并列演示"同契约第二种算法": 拓扑 (单/双臂) 与流派 (DLS/HQP) 都住实现层 |
| `vendor/mos_wbc/` | mos 接口头快照 (见下) |

## vendor/mos_wbc 来源与 patch

- 来源: `mos/arm_controller/include/arm_controller/{types.h, wbc_controller/wbc_solver.h}`, 快照 2026-10-29 (合并过渡期资产; 维护者吸收后按外部仓模式收敛, 届时本目录可删)。
- **唯一 patch**: `wbc_solver.h` 的 `#include "arm_controller/types.h"` → `#include "types.h"` (同目录相对引用, 脱离 mos 目录树编译)。
- 依赖闭包: 两头只需 Eigen (pimpl 把 pinocchio/qpOASES/yaml-cpp 藏在实现侧)——**编译适配层零额外依赖; 链接需 mos 实现体** (在 mos 仓, 或未来可选子库)。

## 规范检查命令 (g++ 直编, 零 ROS)

```bash
cd unistackbot_algorithm
g++ -std=c++17 -Wall -Wextra -fsyntax-only -I. -I/usr/include/eigen3 \
    whole_body/upper_body_hqp_mos.hpp
```

## 实例暴露的契约缺口

1. **已收口（v0.1，2026-10-29 用户批）**：`solve` 返回 `WholeBodyResult` 五值枚举（OK/NOT_READY/UNREACHABLE/LIMIT_CONFLICT/SOLVE_FAILED，IkResult 同款）+ `whole_body_result_message`；适配层预检 `isCartesianTargetReachable` 分类 UNREACHABLE，预检后仍失败 = SOLVE_FAILED（mos 返回 bool 不细分，LIMIT_CONFLICT 预留给手术后实现）；
2. 关节序校验缺出口: mos 未暴露 `getJointNames()`, 适配层暂依赖 yaml 与 URDF 同序 (mos 侧一个名字 getter 即闭);
3. 任务误差遥测适配层自算 (目标−当前帧), 免 mos 加误差出口。

## 示例二暴露的契约项 (v0.2 已收口, 2026-10-29 用户批)

1. **`dt_s` 已入 `WholeBodyConfig`**（无默认）——worker 节拍约定值，单一真相在壳（装配期壳对照自身节拍校验，不一致拒启）；示例二直接用它积分/差分，示例一与 mos vendor yaml 的 frequency_hz 对齐（手术后两源归一）；
2. **`base_frame` 已入 `WholeBodyConfig`**（无默认）——全部 CartesianGoal/FramePose 的参考系显性化（原注释"base 系"是隐性假设）；实现须校验存在不许忽略：示例二直接用它建链，示例一用 UrdfFk 探针 base→end_frames[0] 校验。

另: 示例二 `out.gravity` 留空 = 诚实能力声明 (运动学栈无动力学源, 永不伪造; 壳按空=无前馈)。

## 重宿主语义决定 (适配层职责)

mos 原设计开环积分 (setCartesianTarget 内部 integrate 推进 current_q_ 不回锚); 适配层改为**每拍测量重锚 = 闭环消漂**; `setNewJoint` 实为状态注入口 (ddq 注入侧未被 QP 消费, 传零)。

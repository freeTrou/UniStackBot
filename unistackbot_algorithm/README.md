# unistackbot_algorithm

**算法 SDK（Pkg 1）**——契约头 + 可选共享 FK，**零本仓包依赖、零 ROS、零 nlopt**（ldd 实证：libunistackbot_algorithm.so 不链任何 unistackbot 库）。
2026-11-01 两包拆分定稿：参考实现与插件注册住 **Pkg 2 `unistackbot_algorithm_solvers`**（算法组之家）；**算法团队最小开发闭包 = 本包一个**（兄弟目录拷走即用，系统依赖仅 Eigen/KDL/urdfdom）。架构定稿与接口文档（normative）：`docs/architecture/algorithm_architecture.md`。

## 内容

| 目录 | 是什么 |
|---|---|
| `ik_solver/` | **IkSolver 抽象接口**（CM 的可插拔求解层，5 条硬契约在 hpp 头注释：失败不改 out_q / 墙钟预算 / SolveMode 语义 / 调用域自声明 / stats）+ `redundancy.hpp`（RedundancyPreference——7 轴冗余消解词汇，2026-11-02 自 interface 迁入：契约归契约的主人） |
| `ik_result/` | **IkResult 结果码枚举** + 人话消息函数（2026-11-02 自 interface 迁入） |
| `whole_body/` | **WholeBodySolver 契约**（v0.3：同步 solve 纯函数 + goals 集合 + WholeBodyResult 五值枚举 + WholeBodyConfig 显式无默认）。**待算法团队 co-sign**——适配层行为变更对照表见 Pkg 2 `whole_body/README.md` |
| `base_kinematics/` | **BaseKinematics 抽象接口** + 注册工厂（底盘运动学契约；实现住 Pkg 2） |
| `urdf_fk/` | 可选共享库：URDF→KDL 链→FK/雅可比/限位/链序名/结构指纹数据源/`chain()` 只读访问（想用就用，不用完全不碰；实例线程私有，RT 零分配）。测试 `urdf_fk/test/run_urdf_fk_test.sh` |

## 边界（SDK 不提供的东西）

- **不含任何 IK/WB 实现**——参考实现住 Pkg 2，正式归宿是算法团队自己的包
- **不含 gravity_torque / collision**——SDK 定义接口不选库（2026-11-01 原则性修正：算法用什么 pinocchio/KDL/qpoase 自己的事；`WholeBodyOutput.gravity/tasks` 字段由求解器填）
- 不含 rclcpp/pluginlib/nlopt——ARM 交叉编译友好

## 实现一个求解器（三步）

1. 实现接口（范本：Pkg 2 `whole_body/dualarm_dls.hpp` 纯仓内栈 / `analytic_piper/` IKFast 适配层）
2. 自带 `PLUGINLIB_EXPORT_CLASS` + plugin.xml 注册
3. 机型 yaml 一行选用（`ik_solver:` / `solver:`），启动日志回显激活解

完整指南（外部包接入三步 + 独立测试路径 + 归属终表）：`docs/architecture/algorithm_architecture.md` §4。

## 纪律

- 契约词汇（IkResult/Redundancy）住本包，**实现永不反向定义契约**；虚签名变更 = 破坏性变更需评审
- 验证方法论 SOP（六阶段 + 参数审计 + 换臂流程）：`docs/guides/ik_validation_playbook.md`

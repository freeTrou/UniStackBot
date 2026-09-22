# 仿真验证流水线（机型接入 → 验收归档）

> 2026-09-21 立项。基于机械臂的首个端到端流程，目标 = **可复用资产**：新机型只带
> 资产进流程（URDF/MJCF/yaml），流程本身零改动——与框架"形态盲"哲学同构
> （复用性验收 = 跑第二台机型时本流程与流程引擎 git diff 为零）。
> 工具不先行：**流程位置倒推工具需求**（§5），流程设计好，在哪需要哪些工具就好做了。

## 1. 设计原则

- **流程引擎只做编排**，不实现验证逻辑——逻辑在各工具内，引擎复用其退出码与输出
- **阶段门禁**：不过门不进下一阶段，fail fast 且定位到阶段；跨链验收门层级（G1 冒烟→G6 真机）见 `sim_environment_and_test_plan.md` §5.5
- **产物可归档**：每阶段留输出文件，汇总为验收报告——报告是复用资产的历史样本
- **链次序 = 互补漏斗**（`sim_environment_and_test_plan.md` §1.3）：mock 验逻辑 → mujoco 验物理 → gz 验集成
- **机型无关**：流程不写死机型/关节，全部从 URDF/控制器参数推算（verify 已实现该原则）

## 2. 流程总览

| 阶段 | 内容 | 工具 | 验收门 | 产物 |
|---|---|---|---|---|
| **A 模型接入** | 四态 xacro 展开 + check_urdf + 可视化目检 + 限位/mimic 完整性 | verify §1 ✓、display ✓ | 全过 | 静态记录 |
| **B mock 逻辑** | 三件套激活 + JS 流跟踪 + CM 切换收敛；F1-F6；/sim_control 全语义 | verify §3 ✓、fault_injection ✓、smoke ✓ | 全绿 | 三件日志 |
| **C mujoco 物理** | （前置：MJCF 资产生成+策展）流跟踪 + CM 收敛 + F1-F7 | fault_injection `--chain mujoco` ✓ | 全绿 | 回归日志 |
| **D gz 集成** | 激活 + 跟踪 + /sim_control 桥 + pause/resume 端到端 | verify `--with-gazebo` ✓ | 全绿 | gz 日志 |
| **E RT 基准** | （前置：rt_tune_boot）cyclictest 三档 + CM 链路 WCET/误差 | rt_chain_bench ✓ | WCET p99 < 预算；误差 < 阈值；无异常签名 | `results/rt_<标签>.md` |
| **F 数据与场景** | 标准场景跑 + 录制 + 曲线检。首版：demo_motion + 手动 bag；完整版：场景库 + 录制器 + 对比器（工具缓建，场景族立项时到位） | 🔧 control_recorder、跨链对比器 | 曲线无异常 | bag/CSV |
| **G 验收归档** | 汇总 A-F 产物 → 验收报告 → `results/` 入库 → git tag | 🔧 **run_acceptance.sh** + 报告模板 | 报告生成 | 报告 + tag |

阶段特殊说明：

- **A**：可视化目检（mesh 丢失/装配错位）目前只有人眼——机器可检部分已被 §1 覆盖
- **C**：MJCF 生成+策展是**换机型的主要手工点**（再生成命令见 `arms/<robot>/mujoco/README.md`；equality/接触/执行器策展清单同）
- **E**：平台级 cyclictest 基线跨机型复用（权威基线已入库），换机型只需重跑链路段
- **B/C/D 的断言执行器都是 verify_robot.sh / fault_injection.sh**——流程引擎调它们，不重复实现

## 3. 流程引擎与报告（run_acceptance.sh）

```bash
bash test/run_acceptance.sh <robot> [--with-rt]   # A→G 编排, 任一门挂则停并定位
```

- 编排：verify（全链）→ fault_injection（mock + mujoco）→ [可选 rt_chain_bench] → 汇总
- 报告模板：环境快照（内核/cmdline/commit）+ 各阶段结果表 + 产物路径 + 结论签名位
- 报告落 `test/results/acceptance_<robot>_<日期>.md`，随结果文档一起入库
- 首跑对象 = **piper**（已三链全绿 → 流程自验证）；第二跑 xarm7 = 首次复用实测（**尚未跑过完整 A-G**，随 run_acceptance.sh 落地一并进行：验证引擎 + 复用实测一石二鸟）

## 4. 工具需求汇总（由流程位置倒推）

| 优先 | 工具 | 服务的阶段 | 状态 |
|---|---|---|---|
| 1 | **run_acceptance.sh 流程引擎 + 验收报告模板** | G（串起 A-F） | 待开发（本次） |
| 2 | 跨链对比器（同命令流三链指标 diff） | F | 待开发 |
| 3 | 扰动注入 CLI（wrench 脉冲/阶跃） | C 进阶 / S2 场景 | 待开发 |
| 4 | /sim_control/diagnostics 计数器话题 | B/C 长跑断言抓手 | 待开发 |
| 5 | control_recorder 录制节点 | F 完整版 | **缓建**（用户裁定 2026-09-21：流程到位、场景族需要时再上） |
| 6 | 即时故障注入 CLI（ad-hoc 调试） | 流程外 | 待开发 |

场景族（S1-S6）与批次路线见 `sim_environment_and_test_plan.md` §5-§6；本文只管**串流程**。

## 5. 与现有资产的关系

- `verify_robot.sh` / `fault_injection.sh` / `smoke_sim_control.sh` / `rt_chain_bench.sh` = 各阶段断言执行器（珠）
- 本文 + run_acceptance.sh = 流程（串）
- `sim_environment_and_test_plan.md` = 体系文档（三链定位/测试矩阵/批次）
- `ik_validation_playbook.md` = 算法侧 SOP（B/C 的 IK 深验证按它走，流程不重复）

## 6. 复用性验收（本资产的完成标准）

1. piper 首跑全绿，报告入库（流程自验证）
2. xarm7 第二跑零流程改动全绿（首次复用实测）
3. 未来新机型（如 7 轴）：只准备机型资产 + playbook 参数表，run_acceptance 一条命令出报告

**已知边界（2026-09-21 充分性审计）**：流程覆盖"链对命令流的响应"；算法在环系统场景由批次 6
上层消费者样例节点覆盖；传感器闭环（数天级资产策展）与多形态（双臂/人形）保持挂账——
等 G3 前置/立项信号，不预做。真机就绪按定义超出仿真边界（G6）。

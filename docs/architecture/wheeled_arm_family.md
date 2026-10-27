# 轮臂家族设计文档（模块化轮式机器人）

状态：设计定稿（2026-10-27 落笔；裁决积累于 2026-10-22~27 会话）
消费方：新轮式机器人立项实施、真机批次拓扑定案。
定位：**家族 = 一套物理模块 × N 种组合，每种组合是框架的一等机型**；组合只发生在配置层，框架代码零改动（验收 = 框架包 git diff 为零）。

## 1. 需求

可拆卸、自由组合的轮式机器人（半自研 + 半 Lumos 硬件），五个出厂组合：

底盘 / 单臂 / 双臂 / 单臂+底盘 / 完整机器人。

## 2. 核心裁决清单（全部已拍板，勿重议）

| # | 裁决 | 内容与理由 |
|---|---|---|
| R1 | **单 CM** | 任何形态永远单 CM 实例（2026-10-26 用户定死）。CM 的 base/tip 单链语义 = 框架笛卡尔面的全部；多臂 = 其余臂走关节域 JS（stationlite/r1_pro 同款）。`cm_left/cm_right` 双实例提案作废；双臂协同单 QP（Lumos mos 参照）不是我们的路线，记档为他家产品形态 |
| R2 | **多总线 = 模块 = 链** | 一模块一条总线一块 ros2_control 组件（原"混总线预裁决"随本文正式定案）。故障域三重重合：模块域=总线域=故障域 |
| R3 | **配置期组合** | "可拆卸" = 按形态启动使用；不做运行期热插拔（带电拔臂=量级跃升，挂账无触发器） |
| R4 | **变体 = 机型** | 每组合一个薄顶层 xacro，文件名 = launch 解析契约；不做单文件开关参数（piper use_gripper 参数穿透坑在案） |
| R5 | **CM 链落单总线** | base→tip 链应完整落在一条总线上；单 CM 裁决下天然不需要跨总线链 |
| R6 | **模块清单** | {底盘, 臂×2}（完整 = 双臂+底盘；无腰/头，出现即扩宏库） |

## 3. description 层组织

```
<family>/
  modules/                 # 模块宏库 = 几何/关节唯一事实源
    arm.xacro              #   臂 (params: parent, side)
    base.xacro             #   底盘 (轮系; 轮接口仅物理链声明 — mock 无世界语义)
    mounts.xacro           #   模块间安装变换, 一处定义全部偏移
  urdf/                    # 薄变体顶层 (~15-30 行, 只有装配声明+ros2_control 组装)
    <family>_base.urdf.xacro
    <family>_arm.urdf.xacro        # standalone 臂: 挂支架根 (安装面即模块边界)
    <family>_arms.urdf.xacro       # = r1_pro 资产模式照抄 (js_arms + 单CM)
    <family>_arm_base.urdf.xacro
    <family>_full.urdf.xacro
```

纪律：变体文件里**禁止**出现实体几何/关节字面量（出现即重演 Lumos HARD CODE 病，只是更隐蔽）；改一个限位只碰模块宏，五变体同刻同步。

## 4. 配置层

- `bringup/config/<family>_<variant>_controllers.yaml` × 5：单 CM 形态全部同款（`ik_solver: trac_ik` 类 7 轴 / 解析解类 6 轴按臂型选，插件按名装载）；底盘件（BaseVelocityController）仅含底盘的变体配；
- `bringup/config/<family>_<variant>_acceptance.yaml` × 5：指纹（子链数=变体实际 link 数）/关节集按链（底盘变体轮系仅 gz/mujoco 声明，mock 无轮——与 r1_pro 裁决一致）/容差；
- JTC 常驻注册同五机型惯例。

## 5. 对外话题面（统一仿真接入口的家族口径）

**臂（全部变体同款，单实例裸名）**：

| 方向 | 话题 | 语义 |
|---|---|---|
| 命令 L1 | `/joint_stream_controller/command` | 点流可打断，**断流=保持最后目标** |
| 命令 L2 | `/cartesian_motion_controller/command` | 需 CM active；单 CM 裁决下 `cm_left/cm_right` 不存在 |
| 反馈 | `/joint_states`（URDF 声明序）/ `/ee_state` | 单 EEB 裸名无撞名问题（单 CM 红利） |

**底盘（仅含底盘变体；契约已定案）**：

| 方向 | 话题 | 语义 |
|---|---|---|
| 命令 | `/cmd_vel` | **断流=零速**（与臂相反——移动底盘失令必停，接入文档第一行） |
| 反馈 | `/odom` + TF `odom→base_link` + `/joint_states`(轮/舵) | |

组合差异面完全体现在实例清单上——这正是 acceptance 指纹要登记的东西。supervisor 源期望：全变体同源集（单 CM 红利，无 per-variant 配置缺口）。

## 6. 硬件拓扑（半自研 + 半 Lumos 的首例组合）

```
            unistackbot 框架 (单 CM × 多总线)
                     │ ResourceManager (跨组件认领)
      ┌──────────────┴──────────────┐
【Lumos 半边】                   【自研半边】
lumos_board 总线组件              canfd / spi 总线组件
├ 传输: UDP                       ├ 传输: SocketCAN can0/can1 (Kvaser 盒已环回验收)
├ 协议: lumos 帧 codec            ├ 协议: 达妙 dm 系 (资料+A1 codec 已备)
│  (mos_hal protocol.h=完整规格)   └ (或 spi_gateway→MCU, 设计已成文)
├ 状态机: 薄翻译层 (板侧模式字)
└ ★板固件一行不改: 帧只代表 Lumos 自己的电机, 整机关节拼装在 ResourceManager
```

故障域红利：任一半边故障，另一半照常（模块=总线=故障域的兑现）。

## 7. 已知缺口与触发器

| 缺口 | 方案 | 触发器 |
|---|---|---|
| 轻量底盘后端（mock 链底盘闭环） | SimControlHardware 加 velocity 命令接口 + `base_kinematics` 积分节点发 /odom+TF（2~3 天，全复用现有库；不违反"mock 无世界语义"旧裁决——独立可选节点） | 底盘算法作者开始高频迭代 / 家族底盘变体要在 mock 验收 |
| 底盘单机变体 | 家族第一件实操（最简变体，单模块） | 家族实施启动日 |
| standalone 臂的安装面 | 支架/bench 定义为模块边界的一部分 | 单臂变体落笔时 |
| ground truth 评估话题 | gz 链可选 /ground_truth | 算法评估需求出现 |

## 8. 验收矩阵

5 变体 × 3 链 verify；底盘变体 mock 链 = 关节集空/无轮（按 r1_pro 口径），gz/mujoco 全量；轮臂混合变体按臂+底盘各自断言叠加。CI 纪律：**模块宏改动 → 全变体回归联动**。

## 9. 参照系要点（Lumos mos 剖档结论，防"要不要学他们简化"的动摇）

- 五趋同（Ruckig/身份在消息里/伺服在有损链上一层/驱动器侧闭环/配置轴）= 本征解，可放心；
- 四不学：环内锁与分配 / 静默降级 / 焊死组合层（HARD CODE 19 的位置就是我们模块宏的位置）/ 无校验链；
- 一句话：**他们的环简单，是因为他们的环不实时；我们的层多，是每一层都在换一个真失败模式。**

## 10. 实施排序（建议）

1. 底盘单机变体（家族第一件，走通模块宏+薄变体组织）；
2. 单臂变体（复用臂模块）；
3. 轻量底盘后端（底盘变体 mock 闭环前置）；
4. 双臂 / 混合 / 完整（r1_pro 模式照抄 + 组合叠加）；
5. 真机批次对上第 6 节拓扑（与达妙/CAN 线并轨）。

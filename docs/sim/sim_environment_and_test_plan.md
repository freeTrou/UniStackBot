# 仿真环境完整度盘点与测试方案

> 2026-09-21 定稿（方案审核通过后入库）。本文是三链仿真体系的**现状基线 + 接口契约 +
> 测试矩阵 + 补全路线**的单一事实源；实施批次见 §6，每批完成后本文对应条目打勾更新。

## 1. 现状盘点

### 1.1 三链状态

| 链 | 包（目录） | 定位 | 状态 | 缺口 |
|---|---|---|---|---|
| mock (kinematic) | `unistackbot_sim_control/core` | 第一时间验证（静态/可视化/冒烟）+ 逻辑权威 | **绿** | — |
| gz (Fortress) | `unistackbot_sim_control/gazebo` | 系统集成权威 | **绿**（关节冻结破案 2026-09-21：#165 限位咬合，initial_value 修复入库；verify 50/50） | 手指耦合方案（待裁决）；运行时残余=避免命令停限位静置 |
| mujoco | `unistackbot_sim_control/mujoco` | **物理仿真权威**（RL 训练同引擎 MJX） | **绿**（双机型；/sim_control 桥 0d 已接） | 力矩执行器（批次 7） |

绿的定义：三件套控制器（joint_state_broadcaster + joint_stream_controller active +
cartesian_motion_controller inactive 注册）正常起链、关节流跟踪 mrad 级、CM 切换收敛、
断流受控减速（0c）生效。

### 1.2 横切欠账（JTC→JointStream 迁移半途）

launch/yaml/控制器侧已完成迁移；以下五个文件仍挂 JTC（对当前链必失败或不可用）：

| 文件 | 问题 | 处置（批次 1） |
|---|---|---|
| `test/verify_robot.sh` | 断言 JTC active + follow_joint_trajectory goal | 重写为 JointStream 发流跟踪 + CM 切换收敛；加 `--with-mujoco` |
| `test/check_fk_tf.sh` | 等 JTC active 才继续 | 改等 joint_stream_controller |
| `unistackbot_bringup/scripts/demo_motion.py` | ActionClient→JTC | 重写为 JointCommand 50Hz 点流 |
| `unistackbot_controller/tools/ik_demo_node.cpp` | ActionClient→JTC | 改发 CM `~/target`（PoseStamped） |
| `unistackbot_controller/tools/fk_tool.cpp` | JTC 参数服务硬依赖 | 去依赖（robot_description 已足够） |

### 1.3 三链互补性定位（2026-09-21 裁决：互补不重复）

验证漏斗：**mock（最快、先跑）→ mujoco（物理）→ gz（集成）**。

| | mock | mujoco（物理权威） | gz（集成权威） |
|---|---|---|---|
| 独有面 | 逐位确定性、瞬移建态、纯算法 WCET、静态/可视化第一时间验证 | 惯量/重力/接触动力学、执行器模型（位置伺服→力矩扩展）、外力扰动、equality mimic、原生 reset、**RL 训练同引擎（MJX）= sim2real 中间验证站** | ros_gz 生态、传感器桥（FT/相机/深度）、世界/场景管理、parameter_bridge、多节点系统负载、RTF 观测 |
| 测试重心 | 命令语义、防线逻辑、预算、确定性金丝雀 | 动力学回归全权、扰动/力矩/接触场景、RL policy 上链验证 | 传感器管线端到端、桥接可靠性、系统级服务、标准生态兼容（后续 MoveIt 等） |
| 收缩 | — | — | 跟踪类深测维持回归门水位，深场景移交 mujoco |

载体：三链同款话题契约 → **同一场景 YAML + 录制节点三链复用**（`control_recorder`，批次 5），指标各取所需。

## 2. Launcher 入口

### 2.1 统一入口（批次 1 落地）

```bash
ros2 launch unistackbot_bringup sim.launch.py chain:=<mock|gz|mujoco> robot:=<piper|xarm7> \
    [use_rviz:=true] [gui:=false]      # gui 仅 gz 链
    [headless:=false]                  # headless 仅 mujoco 链 (默认 true)
```

- 实现住 bringup（顶层启动编排层）；OpaqueFunction 按 `chain` Include 对应链 launch，
  只做编排+参数转发；未知 chain/robot fail-fast 列合法值。
- 三链的 spawner 集合保持各自 launch 内定义（规范实现不动）。

### 2.2 分别入口（保留，规范实现）

| 链 | 命令 | 特有参数 |
|---|---|---|
| mock | `ros2 launch unistackbot_bringup control.launch.py robot:=X` | use_rviz |
| gz | `ros2 launch unistackbot_gazebo ign.launch.py robot:=X` | gui, use_rviz |
| mujoco | `ros2 launch unistackbot_mujoco mujoco.launch.py robot:=X` | headless, use_rviz |

可视化（非链）：`ros2 launch unistackbot_description display.launch.py`。

## 3. 对上接口契约（反馈流 + 控制流）

### 3.1 控制流（上层 → 链）

| 接口 | 消息 | 语义 | QoS |
|---|---|---|---|
| `/joint_stream_controller/command` | `unistackbot_interface/JointCommand` | **关节弧度控制**：点流（每条=最新目标）；CSP 生效，CSV/CST/MIT 真机批次启用 | reliable + KeepLast(1) |
| `/cartesian_motion_controller/target` | `geometry_msgs/PoseStamped` | **末端位姿控制**：base 系值通道，CM 以 inactive 注册需 switch 接管 | reliable + KeepLast(1) |
| `/cartesian_motion_controller/control` | `unistackbot_interface/CartesianControl` | TRACKING/HOLD 事件（HOLD 冻结目标） | transient_local |
| `/sim_control/*` | std_srvs | reset / set_joint_state / pause / resume / step | mock 全量；gz 经适配器（reset/set_joint_state 拒绝）；**mujoco 适配器已接 (0d, 2026-09-21: pause/resume/step/reset ✓, set_joint_state 拒绝)** |

安全兜底（对上层透明）：NaN 门→限位 clamp→步长饱和（控制器层拒绝 + write 层 clamp 双层）；
断流受控减速（StaleWatch, JS 默认 200ms 判定 + 200ms 线性刹停；CM 同款默认关）。

### 3.2 反馈流（链 → 上层）

| 接口 | 内容 | 频率 | 状态 |
|---|---|---|---|
| `/joint_states` | 关节位置/速度/effort（**乱序，按名对齐**） | 控制频率 | ✓ |
| `/tf` | 全身位姿（含 EE：link6/link7） | JSB 节拍 | ✓ |
| `/cartesian_motion_controller/status` | 误差 + current_pose + min_sigma + stream_stale | 20Hz，**仅 CM active** | ✓ |
| `/ee_state` | EE 位姿独立反馈流（与控制器选择无关; 话题全名 `/ee_state_broadcaster/ee_state`） | 50Hz | ✓ 2026-09-21 (mock 实测 47Hz, TF 对拍 4e-16) |
| `/clock` | 仿真时间（gz/mujoco） | — | ✓ |

`/ee_state` 设计（批次 2）：控制器插件 `ee_state_broadcaster`（ControllerInterface，读
关节 state 接口 → UrdfFk → `~/ee_state` PoseStamped, base 系, RealtimePublisher trylock
50Hz）。**旋转表示决策挂起**（用户待定四元数/RPY/矩阵）：先以 PoseStamped 四元数为载体
（TF 兼容、任何表示可导出），决策后升自定义 `EeState.msg` 加并行字段、不破坏现有消费者。

### 3.3 进程内契约（不走 ROS 话题）

`RobotFeedback` / `RobotCommand`（`unistackbot_interface/include/`，POD + static_assert）
是**真机总线帧的进程内值通道契约**（SpLatest 载体），与 ROS 侧 `JointCommand.msg` 构成
"单一契约两载体"。仿真链上不出现这两个类型——上层若走 ROS，一律用 §3.1/§3.2 的话题。

## 4. 算法嵌入仿真的四条路径

| 层 | 机制 | 现状 | 验证路径 |
|---|---|---|---|
| **控制器插件**（链内 RT） | ControllerInterface + pluginlib：yaml 注册 + spawner | ✓ 零框架改动（JointStream/CM 即此形态） | mock 冒烟 → 三链回归 |
| **IK 求解器**（CM 内 RT） | `IkSolver` 抽象接口 + `ik_solver` 参数选择 | **接口已就绪** (2026-09-21: DlsIk 已继承接口, CM/worker 走接口指针; 用户 7 轴数值解 = 实现接口加分支, 平级 AB 对比) | `docs/guides/ik_validation_playbook.md` 六阶段 |
| **上层算法节点**（链外 non-RT） | 订 §3.2 反馈流 → 发 §3.1 命令流（RL/VLA ingress，设计文档 §6.1）；断流减速已兜底 | ✓ 已支持 | fault_injection F6 场景 |
| **仿真后端** | hardware_interface 插件（`sim_backend_factory` 预留多后端） | ✓ 预留 | 组件测试 + 链冒烟 |

## 5. 分链测试矩阵

> **承诺边界（2026-09-21 评审补）**：批次 6 落地前，矩阵承诺 = 冒烟 + 故障注入 + RT 基准；
> 标"后续 / 前置"的行不作为现行验收门，只挂账。

### 5.1 mock（第一时间验证 / 逻辑权威）

| 项目 | 断言 | 工具 | 状态 |
|---|---|---|---|
| 静态一致性 | 四态 xacro 展开 + check_urdf + 限位/mimic 推算 | verify §1 | ✓ |
| 起链与生命周期 | 三件套激活 + CM inactive 注册 + switch 互切 | verify §3 | ✓ |
| JS 命令语义 | 点流"最新目标" + 每周期步进 + 关节列全校验 | verify §3 + bed | ✓（非法模式丢弃断言 🔧批次6） |
| CM 收敛与遥测 | 误差→0 + result=0 + INACTIVE 门闩 + stream_stale 置位 | verify §3 | ✓ |
| write/read 防线 | F1 断流 / F2 NaN / F3 限位 / F5 超速 / F6 断流刹停 | fault_injection | ✓ |
| /sim_control 全语义 | pause/resume/step/reset + set_joint_state 瞬移 + 非法拒绝 | smoke | ✓ |
| 断流受控减速参数 | JS 200/200 生效；CM 默认关（--once 合法） | bed F6 | ✓ |
| FK↔TF 对拍 | 独立实现同 URDF 同 q 对拍 ~1e-13（FK 正确性裁判） | check_fk_tf.sh | ✓ |
| **确定性金丝雀** | 同命令流重放 → /joint_states 序列逐位一致（**仅 mock 可做**） | 场景库 S6 | 🔧批次6 |
| **ruckig 档链上行为** | OtgStream 整形 + Hold 输出恒有效 + 10-100Hz 率失配填充 | 场景 S1 变体 | 🔧批次6 |
| 纯算法 WCET | p99/max + 冷启动签名 | rt_chain_bench mock | ✓ |

### 5.2 mujoco（物理权威）

| 项目 | 断言 | 工具 | 状态 |
|---|---|---|---|
| 动力学跟踪 | 50Hz 流 <10 mrad | bed | ✓ |
| 防线在动力学下 | F2 NaN 零位移 / F3 限位截停 / F5 速度界内 / F6 刹停账面吻合 | bed --chain mujoco | ✓ |
| mimic equality | gj = ±0.5×gripper（±0.005） | bed F7 | ✓ |
| CM 动力学收敛 | 误差 <1mm / 收敛 ~1s | rt_chain_bench mujoco | ✓ |
| reset 语义 | 真复位（重力下垂逐位一致） | /sim_control reset | ✓ (0d) |
| **扰动响应** | wrench 阶跃/脉冲 → 不发散 + 恢复收敛 + CM 抗扰 | 扰动 CLI + S2 | 🔧批次5/6 |
| **接触场景** | 接触参数策展 + 接触突变下稳定 | 批次7 | 🔧批次7 |
| **力矩执行器 / MIT** | motor 执行器 + effort 接口 + τ=kp·e+kd·ė+τ_ff | 批次7 + S5 | 🔧批次7 |
| 动力学 WCET | 物理线程争用下 RT 面（508µs 签名已归档） | rt_chain_bench mujoco | ✓ |
| RL policy 上链 | 输出→关节命令映射正确；episode reset 确定性 | 批次7 MIT 链 + policy 回放 | 🔧前置：RL policy（未到） |
| MJX↔mujoco 同引擎验证 | 分布一致（**统计容差**；两实现逐位一致不可达） | 同场景统计对比 | 🔧前置：RL 训练产出（未到） |

### 5.3 gz（系统集成权威）

| 项目 | 断言 | 工具 | 状态 |
|---|---|---|---|
| 起链 + 跟踪 + 桥 | active + 跟踪 + /clock + pause/resume 端到端 | verify --with-gazebo | ✓ |
| 拒绝语义 | reset / set_joint_state 诚实拒绝（Fortress 有毒实证） | verify + smoke | ✓ |
| RTF 观测 | /stats rtf≈1 且非暂停 | verify | ✓ |
| **FT 传感器管线** | /ft 更新率 / 量程 / 时间戳与 /clock 对齐 | 前置：传感器资产策展 | 🔧G3 前置（模型现 0 sensor） |
| **相机/深度管线** | 帧率 / 分辨率 / 时间戳对齐 | 同上 | 🔧G3 前置 |
| **世界管理** | 物体生成 / 场景操作 | 未立项 | 挂账 |
| **多节点负载** | 生态负载下链路稳定 | 未立项 | 挂账 |

> F 场景不在 gz 链跑 = 设计使然：write 层防线住 SimControlHardware（我们的插件），gz 链插件是
> 上游 GazeboSimSystem；控制器层防线已由 mock+mujoco 覆盖。

**场景族 ↔ 清单对应**：S1 轨迹族→mock ruckig/CM 边界 · S2 扰动族→mujoco 扰动 · S3 故障族→F8-F10 ·
S4 生命周期→mock 切换应力 · S5 MIT→mujoco 力矩 · S6 长跑→mock 确定性金丝雀+mujoco 稳态。
✓ 项 = run_acceptance 今日可跑集；🔧 项 = 场景库/批次 backlog。

### 5.4 RT 基准（跨链, 双基线已成）

`rt_chain_bench.sh <标签> [chain]`：cyclictest 三档 + hwlatdetect + CM 链路 E2E。
- mock = 纯算法 WCET（权威基线 `rt_baseline_isolcpus.md`: p99 7.5µs / max 70µs）
- mujoco = 动力学仿真下 WCET（`rt_mujoco_baseline.md` 2026-09-21: p50 <2µs,
  p99 无负载 55µs / 负载档 4µs, **max 508.6µs = 500µs 求解预算剪枝签名**
  （机制实锤: 激活后首批冷 IK 烧穿 update_timeout 被剪 +8.6µs 迭代检查粒度;
  冷因 = mujoco 物理线程 mj_step 冲刷 RT 核缓存; 三档 max 逐位相同 = 确定性机制;
  worker 3 拍自愈 35µs 解出, 周期 2000µs 未超, 无害。详见基线文档归因段）

### 5.5 验收门（跨链, 门以"放行什么"定义）

| 门 | 条件 | 放行 | 现状 |
|---|---|---|---|
| **G1 冒烟门** | 三链 verify 全绿（active+跟踪+切换+桥语义） | 场景/深测开工 | ✓ (50/50, 2026-09-21) |
| **G2 物理门** | mujoco 动力学回归 + F1-F7 + CM 收敛 | RL 上链 / 力控批次 | ✓ (误差 0.74mm, F 全绿) |
| **G3 集成门** | gz 桥语义 + 传感器管线端到端 | MoveIt / 上层生态接入 | 部分（桥 ✓；传感器前置未到） |
| **G4 一致性门** | 跨链对比器三链 diff 各自在容差内（§5.6） | 对外发布仿真结论/数据 | 未开（对比器批次 5） |
| **G5 场景门** | S1-S4 场景族通过 | 真机迁移启动 | 未开（批次 6） |
| **G6 真机门** | §10 迁移清单通过 | 真机运行 | 未开（真机批次） |

### 5.6 跨链一致性测试设计（G4 的具体化）

- **输入**：同一命令序列（场景 YAML 的命令流，链无关——三链同款话题契约是载体）
- **对比内容**：逐拍关节轨迹（/joint_states 按 /clock 对齐）、跟踪误差、收敛拍数、目标→到位延迟、CM min_sigma 轨迹
- **容差原则**：一致性 ≠ 跨链一致——按各链保真度预算判：mock ≈0（逻辑真值）/ gz ≈0 但碰撞生效（position 直设）/ mujoco mm 级（真实动力学）。输出 = 三链指标对照表；**超出该链自身预算才判失败**
- **工具**：跨链对比器（批次 5）；过渡路径（对比器就位前）：`ros2 bag record -s mcap` 同命令流分链手动录制 → 离线对比——**录制器缓建不阻塞本项**，bag 即输入源

### 5.7 测试项目定义（规范先行：代码按定义写；实现与定义冲突 = 实现 bug）

每项 = **定义**（测什么概念）+ **判据**（"过"的可判定条件）+ **边界**（明确不归本层管的，防止层间扯皮）。

**静态一致性**
- 定义：模型资产在不执行仿真的前提下，能被工具链一致地消费成控制链。"坏"= 三判据任一不满足：
  ①**可生成**——入口 xacro 在全部支持形态下展开成功；②**结构合法**——产物为合法 URDF（单根树/引用闭合/mimic 指向存在）且形态指纹（子链数/关节数）符合机型登记值；③**可命令**——命令关节集非空、限位完备（min<max）、mimic 主关节在集内、各形态关节集与登记一致。
- 边界：只判"**可用**"不判"**对**"——坐标系/运动学错归 FK↔TF 对拍，惯量/mesh 缺陷归仿真器暴露。

**起链与生命周期**
- 定义：控制器满足框架生命周期契约（注册→配置→激活→切换→归还）。
- 判据：三件套按预期态就位（JSB/JS active、CM inactive 注册）；切换后命令权完整转移、原控制器静默；切换竞态不产生半激活态。
- 边界：不判运动正确性（归命令语义项）。

**JS 命令语义**
- 定义：`~/command` 是点流——每条消息只表达"此刻最新目标"，与发送频率和历史无关。
- 判据：任意合法消息率下轨迹收敛于目标序列；关节列全是消息合法性前提（缺 → 拒绝并报因）；每周期步进不超速度上限。
- 边界：轨迹形状归上层（路径插值永不在控制器）。

**CM 收敛与遥测**
- 定义：可达目标在有限时间内把末端误差驱赶到零，且全程状态可观测。
- 判据：可达目标误差→容差内且 result=0；不可达目标按结果码诚实报告不静默；INACTIVE 不发遥测；断流时 stream_stale 置位。
- 边界：收敛时间上界归性能基线项，不在此定。

**write/read 防线（F1-F6）**
- 定义：无论上层行为如何，进入执行器的命令恒在安全域（有限性 ∧ 限位 ∧ 步长），断流时保持或受控刹停。
- 判据：F1 保持不漂 / F2 零位移 / F3 精确截停 / F5 顶界不越 / F6 刹停在账面区间；全程计数器留痕。
- 边界：只验 write 层 clamp 语义；"拒绝"语义归控制器层（分层：拒绝在上，clamp 兜底）。

**/sim_control 全语义**
- 定义：仿真控制服务的每个动词有唯一精确语义，非法请求诚实拒绝。
- 判据：pause/resume 真停真走；step 恰进一步；reset 回确定性初态；set_joint_state 仅暂停下生效；越界请求拒绝并说明。
- 边界：各链动词差异（gz 拒绝/mujoco 原生）归适配器文档。

**断流受控减速参数**
- 定义：断流判定是**参数化策略**——按上层语义选默认，不是固定行为。
- 判据：JS 默认 200/200ms 生效且刹停受控；CM 默认关时单发目标不被误刹；开启后与 JS 同构。
- 边界：定义管语义正确，不管数值最优（参数可调）。

**FK↔TF 对拍**
- 定义：FK 求解正确性由独立实现交叉验证——同 URDF 同关节角，末端位姿必须一致。
- 判据：抽样位姿集上偏差 ≤ 机器精度量级（~1e-12）。
- 边界：建链同源（kdl_parser）——抓求解/顺序/实现错，不抓 URDF 原始错（归静态+目检）。

**确定性金丝雀**
- 定义：mock 链输出是命令流的**纯函数**——同输入必同输出。
- 判据：同命令流重放两次，/joint_states 逐位一致。
- 边界：仅对无物理链成立；物理链数值混沌不适用。

**ruckig 档链上行为**
- 定义：慢上层率失配下，插值档任意时刻输出有效目标，无未定义态。
- 判据：10-100Hz 任意率流下输出连续、速度限内、库错误码不外泄（Hold 恒有效）。
- 边界：轨迹形状品质归上层。

**纯算法 WCET / 动力学 WCET**
- 定义：CM 单周期最坏执行时间——mock 档隔离纯算法成本，mujoco 档度量同机物理负载的干扰代价。
- 判据：p99/max 低于单周期预算；签名变化可对照基线解释。
- 边界：只判资源不判功能。

**动力学跟踪 / CM 动力学收敛**
- 定义：控制律在真实动力学（惯量/重力）下仍成立。
- 判据：流跟踪 mrad 级、笛卡尔收敛 mm 级（按机型登记阈值）。
- 边界：接触/扰动极端工况归批次 7 与 S2 专项。

**mimic equality**
- 定义：被动关节由物理约束按 multiplier/offset 契约精确跟随主动关节。
- 判据：跟随偏差 ≤ 登记容差（±0.005）。
- 边界：mock 的软件推导是同一契约的另一实现，分链各自验。

**reset 语义**
- 定义：reset = 回到确定性初态（状态可复现），不是"看起来回去了"。
- 判据：reset 后静止态与首次启动逐位一致。

**拒绝语义**
- 定义：不可执行的请求必须显式失败并说明原因，禁止假成功。
- 判据：无原生等价/已知有毒的操作返回拒绝 + 原因。

**RTF 观测**
- 定义：仿真器具备可读的时间健康度。判据：real_time_factor ≈1 且暂停态可判别。

**FT/相机管线（G3 前置）**
- 定义：传感数据从仿真器到 ROS 话题全链路可用。判据：更新率 ≥ 登记值；时间戳与 /clock 同源可对齐；量程/分辨率符合策展值。

**接触场景 / MIT 力矩（批次 7）**
- 定义：接触参数策展后接触工况稳定；力矩命令通路 τ=kp·e+kd·ė+τ_ff 端到端可执行。
- 判据：接触突变不发散；MIT 命令产生预期力矩行为。

**RL 上链 / MJX 同引擎（前置）**
- 定义：策略输出经命令通道映射到关节执行；训练与验证物理同引擎。
- 判据：映射正确 + episode 复位确定；MJX↔MuJoCo 同场景分布一致（统计容差，非逐位）。

> **定义暴露的两个实现 gap（定义先行的产出）**：~~①形态指纹未断言 ②登记值散落~~
> ✅ 已修（2026-09-22）：机型验收登记表 `unistackbot_bringup/config/<robot>_acceptance.yaml`
> （形态指纹 + 容差 + mimic + 软关节登记值单一事实源）+ verify §1 断言指纹（子链数=登记、
> 三形态关节集=登记）；运动容差/soft/mimic 改读登记。改模型先改登记表，verify 按此把关。

## 6. 实施批次

| 批次 | 内容 | 量级 | 状态 |
|---|---|---|---|
| 0 | 本文档入库 | 0.5h | ✓ |
| 1 横切清理 | sim.launch.py 统一入口；verify_robot.sh 重写(+--with-mujoco)；check_fk_tf.sh 修；demo_motion.py 重写；ik_demo_node 改 CM target；fk_tool 去 JTC | ~1 天 | ✓ (f20e439) |
| 2 ee_state_broadcaster | 新控制器插件 → `/ee_state` 50Hz（旋转表示决策挂起，先四元数） | ~0.5 天 | ✓ |
| 3 mujoco 测试接入 | fault_injection `--chain` 参数化 + F7 mimic 断言；rt_chain_bench mujoco 档 | ~0.5 天 | ✓ (0917e44) |
| 4 排后项 | gz gripper 回归（需 GitHub 代理查上游 0.7.21）；~~/sim_control mujoco 桥~~ ✓ 0d 已接 (sim_control_mujoco_node, 四服务 E2E 实测)；IkSolver 接口（IK 已定性: 7轴接近通用数值解, 与 DlsIk 平级替换）；CM warmup 多轮带预算求解（消 508µs 首拍冷签名, 用户裁定 2026-09-21 暂缓——机制/无害性已归因入档, 见 rt_mujoco_baseline.md） | 各单开 | |
| 5 工具地基（按流程倒推，见 `sim_validation_pipeline.md` §4） | **run_acceptance.sh 流程引擎 + 验收报告模板**（串 verify/fault/bench → 报告；首跑 piper 流程自验证，二跑 xarm7 复用实测）→ 跨链对比器 → 扰动注入 CLI → /sim_control/diagnostics；control_recorder 缓建（用户裁定 2026-09-21，场景族需要时再上） | ~2-3 天 | 进行中 |
| 6 场景族 | S1 轨迹族（冷启动/近奇异/限位贴边）· S2 扰动族（mujoco wrench）· S3 故障族 F8-F10（控制器崩溃/reset 时间回卷/预算饥饿/乱流）· S4 生命周期应力（切换循环/激活态 reset）· S6 长跑稳态（30-60min 流, 稳态零 malloc/计数器零增长）入场景库；**上层消费者样例节点**（50Hz 订阅 `/ee_state` → 发 `~/target`——算法在环系统场景骨架, 兼 RL/VLA ingress 预演, 设计文档 §6.1）+ 长程在环稳定性（率失配/断流真实触发率/时序漂移）；跨链对比首批报告 | ~3 天 | |
| 7 MIT 力矩链 | MJCF motor 执行器策展 + URDF effort 命令接口 + JS effort 消费分支（CSV/MIT）+ S5 场景（双机型）；接触参数策展 | ~3-4 天 | |
| 8 fake master 驱动壳 | 小设计裁决（独立链 vs 复用骨架）→ SystemInterface 壳 + master.hpp + fake master（DeviceState 状态机/看门狗超时/模式拒绝脚本化）+ F1-F7 新壳复跑 | ~3-5 天 | |
| 9 timesync 协议仿真（可选） | pty 虚拟串口 + fake MCU：两帧协议逻辑/锚点表落盘验证（真实抖动等硬件） | ~1 天 | |

每批次独立提交；提交前三链回归（mock 必跑，涉及链加跑）。

## 7. 验收命令速查

```bash
# 统一入口三链冒烟
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=xarm7
ros2 launch unistackbot_bringup sim.launch.py chain:=mujoco robot:=piper headless:=true

# 测试套件 (预期输出见注释)
bash test/verify_robot.sh xarm7              # mock 全量 → "结果: PASS=N FAIL=0"
bash test/verify_robot.sh piper --with-gazebo --with-mujoco   # 三链 → PASS=50 FAIL=0 (piper 实测)
bash test/fault_injection.sh                 # mock → F1-F6 全绿, 死链硬门不产出空洞 PASS
bash test/fault_injection.sh --chain mujoco  # mujoco → 8 场景全绿 (F6 刹停账面吻合)
bash test/rt_chain_bench.sh <标签> [chain]   # → test/results/rt_<标签>.md (p99/max/误差表)
```

## 8. 故障定位树（测试挂了先看这里）

| 症状 | 第一现场 | 处置 |
|---|---|---|
| spawner `Failed to find a free participant index` | DDS 参与者创建失败 | 清残留进程 + `ros2 daemon stop` + 等租约；`ip link show lo` 查 MULTICAST（丢则 `sudo ip link set lo multicast on`） |
| 控制器不起 / `Controller already loaded` | spawner 日志（`/tmp/verify_*_{mock,ign,mujoco}.log`） | 残留同名进程 → 清场重启；一次只跑一套栈 |
| gz 关节冻结不动 | 是否停限位静置 | 上游 #165 限位咬合（启动位已 initial_value 修复）；运行时避免命令精确停限位静置 |
| 跟踪超差 | `/joint_states` **按名对齐**（JSB 乱序坑） | 原始报文人眼核序；yaml 容差/关节集核对 |
| CM 不收敛 | `~/status` 结果码 + stream_stale | 折叠零位远目标=已知边界（预摆位或等 worker 冷启动）；stale 误判查 yaml（CM 默认关） |
| gz 桥/服务断 | parameter_bridge 日志 → `ros2 topic hz` | ign 孤儿 → `gz_clean.sh`（**永远单独一条命令执行**，pkill 会误杀宿主） |
| 慢退出孤儿污染下一链 | 杀链后 1-2min 内同名单例仍活 | verify cleanup_launch 已等家族真退场（5e07cf5）；手动跑链自行等租约 |

## 9. 资源与时间预算（含自动化边界）

| 项 | 实测（2026-09-21, piper） |
|---|---|
| 各链控制器就绪 | mock 4s / gz 6s / mujoco 4s |
| verify 三链全量 | **104s**（含节间收场 ~19s/链，家族等待+gz_clean） |
| fault_injection | 分钟级（mock+mujoco 双链；未精确计时） |
| rt_chain_bench | ≥15min（cyclictest 120s×3 档主导） |
| 硬件/环境前置 | 本机 28 核 lowlatency + isolcpus=1,2；`rt_tune_boot` + `lo multicast on`（均重启即失） |

**自动化边界**：`run_acceptance.sh`（批次 5）= 退出码 + 报告产物，天然 CI-ready；但 RT/仿真链依赖
本机状态（isolcpus / lo multicast / rt_tune_boot / DDS 清场），**云端 runner 不可用**——CI 化 =
自托管 runner 决策，挂账（需要时再立，不预建）。

## 10. 仿真→真机迁移门（G6 的具体化）

**仿真不可证明项**（上真机才暴露）：总线仲裁抖动与负载预算、DC 同步、驱动器真看门狗、上电时序与
零点标定、编码器噪声/齿槽/摩擦、力矩饱和与带宽、STO 硬件急停回路、EMI/线缆/温漂。
**真机必重跑**：verify 断言形态（换硬件插件后同脚本逻辑）、F1-F7（升级为安全验收）、RT 基准
（真机时序归档）。
**接口差异**：仿真 position 直设/理想执行器 vs 驱动器真实 CSP 带宽；CSV/CST/MIT 真机批次才启用。

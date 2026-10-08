# Sim2Real 课题路线图（r1_7a 首目标, 2026-10-08 立项）

> 状态: 方案定稿（用户逐项拍板）, 实施未开始。本文档 = 方案单一事实源; 阶段推进回填状态。
> 格式仿 `architecture/real_hardware_architecture.md` §6A 立项体例。

## 0. 总纲: 双轨制 + 评估器 + 同构骨架

第一梯队现行共识（2024-26 检索定稿, 来源见 §5）: **数据驱动为主线, 模型保真度为保底,
仿真当评估器省真机时间**。对齐到本工程:

- **轨道 A·主线（数据驱动）**: 遥操作采种子 → 数据制造（MimicGen 式回放变换量产）→
  IL（ACT/DP, 本体感受先行）→ 部署 →（接触任务）SERL 式真实机 RL 精调
- **轨道 B·保底（模型驱动）**: G(q) 重力补偿（T1/T2 对拍床）+ 域随机化宽撒;
  **不做精密执行器 sysid**（第一梯队已证: 接触 gap 辨识不经济, DR+数据补课更便宜）
- **评估器（真→仿, SimplerEnv 思想）**: 真机任务环境数字化进 mujoco, 策略先仿评再上真机
- **骨架（已有, 不动）**: MJCF=MJX 同引擎（训练/量产/评估/部署零翻译）· MIT 同构契约
  （仿真 actuator ≡ 宇树 IM 电机）· 策略=不可信命令源经校验+OTG 进门（框架定位的兑现）

## 1. 数据流全图

```
[种子采集] 手柄/主从臂遥操作 (JS 域直发) → 50-100 条种子演示
     ↓
[数据制造] mujoco 链回放+位姿变换 (MimicGen 三步: 物体相对轨迹搬移 →
           回放 → 控制器闭合接触段, 成功才收) → 数千~数万条合成演示
     ↓
[训练]     RL 仓 (MJX/conda): ACT / Diffusion Policy, 本体感受观测先行
     ↓
[仿评]     mujoco 评估器: 真机环境数字化 (视觉匹配/变体聚合), 成功率+鲁棒性
     ↓ 过门
[部署]     policy 节点 → JointCommand (CSP/MIT, 10-50Hz) → JS/CM → 同一条链
     ↓
[精调]     (接触任务) SERL 式真实机 RL 精调 (数十分钟~两小时真机时间)
```

人力解耦原则: 操作员只生产**种子**, 数据量与人力脱钩（MimicGen 一夜数万条）。

## 2. 分阶段与门禁

| 阶段 | 硬件 | 内容 | 门禁 |
|---|---|---|---|
| P0 | 零硬件 | ①手柄前置（JS 域键控遥操已有 `demo_teleop_joint`, joy 映射 ~1 天）②**数据录制规范**（mcap: JointCommand+`/joint_states` 按名对齐, 种子=成功完成的整段命令+状态流）③**制造管线原型**（mujoco 回放一条种子自证成功——管线正确性先在"参数已知"的仿真里证）④T1/T2 gravcomp 对拍（§6A 既有立项, 服务轨道 B） | 回放自证 100% |
| P1 | 手柄 | 手柄遥操采种子 → 制造量产 → IL v1（reach/pick 类结构化任务起步）→ 仿评 | 种子全成功; 量产 ≥5000 条; 仿评 ≥80%（变体聚合） |
| P2 | 真机 | serial 回环 → 时序账 → CM/JS 上真机+gravcomp（T1 产出直接用）→ 同款任务真机部署 → sim2real delta 报告 | 真机成功率 ≥ 仿评×0.6; EE 误差 ≤ 仿真×5（系数进登记表, 改需说明） |
| P3 | 真机 | SERL 式精调 / RMA 式在线自适应（本体感知历史推隐参数, 替代 sysid）/ 视觉观测扩展 | 按任务定 |

## 3. 分工边界

| 侧 | 产出 |
|---|---|
| 框架包（本仓） | 遥操接口（joy→JS/twist→CM）、数据录制/回放工具、制造管线、mujoco 评估器、真机 SystemInterface 插件、delta 报告工具 |
| 算法侧（RL 仓） | IL/RL 训练、策略实现（只依赖 `unistackbot_interface` + 本仓部署链） |
| 硬件侧 | 到货后: 回环/辨识实验/标定（protocol: protocol 文档备料先行） |

## 4. 已知风险与边界

- 种子质量 = 生成数据质量的天花板（种子差则量产的是差数据）
- 精细接触段回放成功率会掉（MimicGen 擅长接近/搬运段）——P1 任务选型避开精细插入
- 视觉观测的渲染 gap 是独立大坑——本体感受先行, 视觉等第一性需求
- 真机时序账（RS485 轮询相位+延迟）仿真没有——bench 已量化（撞窗 ±100µs, errRMS+2%）, 进 delta 报告预期内

## 5. 来源与依据

- [SimplerEnv](https://github.com/simpler-env/SimplerEnv): 真机策略仿真评估（visual matching / variant aggregation）
- [MimicGen](https://github.com/NVlabs/mimicgen): 少量种子→自动化演示量产（轨迹变换+回放闭合）
- π0（Physical Intelligence, 2024）: 7 本体 ~10k 小时真机数据; ALOHA/Mobile-ALOHA: 50 条种子+ACT
- RMA 及机械臂延伸（arXiv "Rapid Motor Adaptation for Robotic Manipulator Arms"）: 隐参数在线推断替代离线辨识
- ETH locomotion 系: 大规模 DR 路线; 第一梯队共识: manipulation 上 DR 保底+真数据主力

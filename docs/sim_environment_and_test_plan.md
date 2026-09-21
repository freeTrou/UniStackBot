# 仿真环境完整度盘点与测试方案

> 2026-09-21 定稿（方案审核通过后入库）。本文是三链仿真体系的**现状基线 + 接口契约 +
> 测试矩阵 + 补全路线**的单一事实源；实施批次见 §6，每批完成后本文对应条目打勾更新。

## 1. 现状盘点

### 1.1 三链状态

| 链 | 包（目录） | 定位 | 状态 | 缺口 |
|---|---|---|---|---|
| mock (kinematic) | `unistackbot_sim_control/core` | 运动学真值 / 控制逻辑层 | **绿** | — |
| gz (Fortress) | `unistackbot_sim_control/gazebo` | 物理协同 | **半绿** | ① gripper prismatic 不响应位置命令 ② 手指无耦合漂移（gz_ros2_control 0.7.21 apt 连带升级引入，2026-09-21 实锤；修复需查上游，**排后**） |
| mujoco | `unistackbot_sim_control/mujoco` | 真实动力学回归 | **绿**（双机型） | ① 测试脚本接入（本轮）② /sim_control 桥（留 0d 编排层） |

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
| **IK 求解器**（CM 内 RT） | `IkSolver` 抽象接口 + `ik_solver` 参数选择 | **接口已就绪** (2026-09-21: DlsIk 已继承接口, CM/worker 走接口指针; 用户 7 轴数值解 = 实现接口加分支, 平级 AB 对比) | `docs/ik_validation_playbook.md` 六阶段 |
| **上层算法节点**（链外 non-RT） | 订 §3.2 反馈流 → 发 §3.1 命令流（RL/VLA ingress，设计文档 §6.1）；断流减速已兜底 | ✓ 已支持 | fault_injection F6 场景 |
| **仿真后端** | hardware_interface 插件（`sim_backend_factory` 预留多后端） | ✓ 预留 | 组件测试 + 链冒烟 |

## 5. 分链测试矩阵

### 5.1 mock（运动学真值 / 逻辑层）

| 用例 | 工具 | 断言 | 状态 |
|---|---|---|---|
| 静态一致性 | `verify_robot.sh` §1 | xacro 展开 + check_urdf + 限位推算 | 已有 |
| 三件套激活 + JS 流跟踪 + CM 切换收敛 | `verify_robot.sh` §3（重写） | 跟踪 <0.01 rad；CM 误差 <1mm | 批次 1 |
| write/read 防线 F1/F2/F3/F5/F6 | `fault_injection.sh` | 全绿（F6=断流刹停不到远目标+静止） | ✓ |
| /sim_control 全语义（含 mimic/限位拒绝/reset） | `smoke_sim_control.sh` | 全绿 | ✓ |
| FK↔TF 对拍 | `check_fk_tf.sh`（修 JTC 等待） | ~1e-13 | 批次 1 |
| 演示流 | `demo_motion.py`（重写） | RViz 可视化正弦运动 | 批次 1 |

### 5.2 mujoco（动力学回归）

| 用例 | 工具 | 断言 | 状态 |
|---|---|---|---|
| 激活 + 7 关节流跟踪 | `fault_injection.sh --chain mujoco`（参数化） | 50Hz 流跟踪 <10 mrad | ✓ (实测 F5 位移界内/跟踪绿) |
| 防线在动力学上的表现 | 同上 F2/F3/F5/F6 | NaN 零位移 / 限位截停 / 速度界内 / 断流刹停 | ✓ 8 场景全绿 (F6 刹停 1.945 rad 账面吻合) |
| mimic equality | bed F7 | gj1 = 0.5×gripper (±0.005) | ✓ 偏差 0.0000 |
| CM E2E（WCET/误差/收敛，隔离核） | `rt_chain_bench.sh <标签> mujoco` | WCET p99 <20µs；误差 <1mm | ✓ `rt_mujoco_baseline.md`: p99 3.8-55µs / 误差 0.74mm / 收敛 ~1s |
| 扰动注入 | mujoco 原生 `apply_external_wrench` | 力矩阶跃不发散、恢复收敛 | 后续（力控批次） |

### 5.3 gz（gripper 回归修复后）

| 用例 | 工具 | 断言 | 状态 |
|---|---|---|---|
| 激活 + 跟踪 | `verify_robot.sh --with-gazebo`（修 JTC 断言） | 控制器 active + 跟踪 | 批次 1（断言修）；gripper 修复排后 |
| 仿真控制桥 | 同上 | /clock、RTF、pause/resume 端到端 | 已有段落保留 |

### 5.4 RT 基准（跨链, 双基线已成）

`rt_chain_bench.sh <标签> [chain]`：cyclictest 三档 + hwlatdetect + CM 链路 E2E。
- mock = 纯算法 WCET（权威基线 `rt_baseline_isolcpus.md`: p99 7.5µs / max 70µs）
- mujoco = 动力学仿真下 WCET（`rt_mujoco_baseline.md` 2026-09-21: p50 <2µs,
  p99 无负载 55µs / 负载档 4µs, **max 508.6µs = 500µs 求解预算剪枝签名**
  （机制实锤: 激活后首批冷 IK 烧穿 update_timeout 被剪 +8.6µs 迭代检查粒度;
  冷因 = mujoco 物理线程 mj_step 冲刷 RT 核缓存; 三档 max 逐位相同 = 确定性机制;
  worker 3 拍自愈 35µs 解出, 周期 2000µs 未超, 无害。详见基线文档归因段）

## 6. 实施批次

| 批次 | 内容 | 量级 | 状态 |
|---|---|---|---|
| 0 | 本文档入库 | 0.5h | ✓ |
| 1 横切清理 | sim.launch.py 统一入口；verify_robot.sh 重写(+--with-mujoco)；check_fk_tf.sh 修；demo_motion.py 重写；ik_demo_node 改 CM target；fk_tool 去 JTC | ~1 天 | ✓ (f20e439) |
| 2 ee_state_broadcaster | 新控制器插件 → `/ee_state` 50Hz（旋转表示决策挂起，先四元数） | ~0.5 天 | ✓ |
| 3 mujoco 测试接入 | fault_injection `--chain` 参数化 + F7 mimic 断言；rt_chain_bench mujoco 档 | ~0.5 天 | ✓ (0917e44) |
| 4 排后项 | gz gripper 回归（需 GitHub 代理查上游 0.7.21）；~~/sim_control mujoco 桥~~ ✓ 0d 已接 (sim_control_mujoco_node, 四服务 E2E 实测)；IkSolver 接口（IK 已定性: 7轴接近通用数值解, 与 DlsIk 平级替换）；CM warmup 多轮带预算求解（消 508µs 首拍冷签名, 用户裁定 2026-09-21 暂缓——机制/无害性已归因入档, 见 rt_mujoco_baseline.md） | 各单开 | |

每批次独立提交；提交前三链回归（mock 必跑，涉及链加跑）。

## 7. 验收命令速查

```bash
# 统一入口三链冒烟
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=xarm7
ros2 launch unistackbot_bringup sim.launch.py chain:=mujoco robot:=piper headless:=true

# 测试套件
bash test/verify_robot.sh xarm7              # mock 链全量
bash test/verify_robot.sh piper --with-mujoco
bash test/fault_injection.sh                 # mock F1-F6
bash test/fault_injection.sh --chain mujoco  # 批次 3 后
bash test/rt_chain_bench.sh mujoco_baseline  # 批次 3 后
```

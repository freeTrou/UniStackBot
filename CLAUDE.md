# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 启动必读（每次会话最先执行）

先读记忆目录恢复工作上下文，再读本文件：
1. `~/.claude/projects/-home-work-project-git-project-UniStackBot-ws-src-UniStackBot/memory/MEMORY.md` — 记忆索引
2. 重点读 `p15-progress.md`（进行中项目的快照：已完成/验收数据/下一步/环境坑）
3. 记忆是时点快照不是实时状态——引用的文件/参数先对照当前代码核实再用

## 工作纪律：一切依据来自实际工程（用户规则，2026-09-21）

- **任何结论下笔前读实际代码验证**——接口形态、参数名、行为、某实现是否存在，都以代码为准；文档（含本文件）、记忆、外部资料都是时点快照，可能与代码脱节，冲突时代码赢。
- **有任何不确定 = 去读代码，不要猜**；代码里找不到实物 = 向用户确认所指，不要用推测补齐（反例教训 2026-09-21：用户说"我的 IK"，未核实所指即默认"外部新实现"凭空写了接入骨架——实际指工程内的 dls_ik）。

## Project Overview

UniStackBot is a ROS 2 Humble workspace for a general-purpose, multi-morphology real-time robot control framework. It targets fixed-base arms, wheeled bases, quadrupeds, wheeled-arm humanoids, and bipedal humanoids from a single layered architecture. One-line positioning (README): 对上是 VLA/RL/传统控制等算法的**可靠接入底座**（不可信命令源经校验+OTG 进门）；对下是**总线与电机协议的实时主站**（CAN/EtherCAT，驱动器自带看门狗为安全终点）；URDF+契约是贯穿全框架的配置轴；核心对形态与算法保持"形态盲、流派盲"——扩展只发生在插件与配置，**验收 = 每次接入新机器人后框架包 git diff 为零**。The first concrete robot is the **Piper arm**: URDF/Xacro description + the unified sim-control layer (`unistackbot_sim_control`, ros2_control hardware plugin with pluggable backends) + Gazebo integration. `unistackbot_controller` hosts the CartesianMotionController CM plugin (P1.5, IK servoing on the control loop) and JointStreamController (关节级 topic 流式控制器, 2026-09-18 取代 JTC action 层); `unistackbot_hardware` (real driver) is still a skeleton.

## Build & Run

Build from the **workspace root** (`UniStackBot_ws/`), not from this repo directory:

```bash
cd <path-to>/UniStackBot_ws
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package: `colcon build --symlink-install --packages-select <pkg>`

Tests (per package, via `ament_lint_auto` — lint checks only, no unit tests yet):
```bash
colcon test --packages-select <package>
colcon test-result --verbose
```

`unistackbot_common` 组件**不进 colcon**——在组件目录下用 g++ 直接编译运行测试（规范命令见各组件 README，`ulog/README.md` 为范本）:
```bash
cd unistackbot_common/ulog
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_ulog.cpp -o test_ulog && ./test_ulog
# TSAN 变体: -fsanitize=thread，且 TSAN_OPTIONS="suppressions=tsan_suppressions.txt" setenv ... setarch $(uname -m) -R ./test_tsan
```
测试二进制（test_ulog/bench_ulog 等）已在 .gitignore，勿提交。

统一入口 + 四个底层入口 (控制链三选一: mock/gz/mujoco):

```bash
# 0. 统一入口 (2026-09-21 批次1): chain 一参切三链
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=piper
#    args: chain:=mock|gz|mujoco robot use_rviz gui headless (转发到下面 2-4 对应链的 launch)

# 1. Visualization only (RViz + joint_state_publisher_gui; ros2_control off by default)
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro
#    model 必填 (机型无默认值); args: model:=<xacro> use_gripper use_ros2_control use_world gui rviz

# 2. Mock control chain: standalone controller_manager + SimControlHardware (kinematic backend), no simulator
ros2 launch unistackbot_bringup control.launch.py robot:=piper   # robot:=xarm7 等; use_rviz:=true|false

# 3. Gazebo Sim / Fortress: controller manager inside gz_ros2_control
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper        # gui:=true|false, use_rviz:=true 可选

# 4. MuJoCo (mujoco_ros2_control, 2026-09-20): controller manager = mujoco 定制 ros2_control_node
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper     # headless:=false(默认, 渲染窗, 需 DISPLAY)|true 无头(基准/录制), use_rviz:=true 可选
```

All control chains spawn `joint_state_broadcaster` + `joint_stream_controller` (active) + `cartesian_motion_controller` (inactive 注册) + `ee_state_broadcaster` (只读, 三链第四 spawner). **JTC 已从链上移除** (2026-09-18)。**spawner 竞态定稿** (2026-09-23, 官方依据 ros2_control issue #2071): 三链 launch 各收拢为 **2 个 spawner 进程**（激活组列表 + inactive 组, 官方 launch_utils/example_13 模式）+ `--service-call-timeout 30` + TimerAction(2s) 让路——根因是 CM 忙时 10s 响应窗口过期触发**盲目重试撞半途状态**（spawner 本有 3 次内建重试, 勿再包壳）; `--controller-manager-timeout` 默认已永远等服务勿调; RMW 层 wait 卡死无解由残留检查兜底。命令通道:

- 关节空间: `/joint_stream_controller/command`（`unistackbot_interface/JointCommand`, reliable+KeepLast(1), 点流语义——每条消息是"最新目标"）。仿真链（position 命令接口）只消费 CSP 模式。关节按 `joint_names` 字段对名映射，必须列全（**mujoco 链只列 7 个主关节**——手指 passive, 见 `unistackbot_mujoco/README.md` 差异表）。
- 笛卡尔空间: `ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped ...`（base 系）。
- 曲线分析录包: `ros2 bag record -s mcap /joint_states /joint_stream_controller/command /cartesian_motion_controller/{status,target} /tf`。

双控制器切换 (三条链同款): CM 以 inactive 注册, `ros2 control switch_controllers --deactivate joint_stream_controller --activate cartesian_motion_controller` 接管 (切回反向同理)。Intended workflow (README has the switch table): iterate algorithms on the mock chain, regress each version on Gazebo/MuJoCo, both green = pass.

RT 备注: 控制器管理器 RT 线程调优**经预留参数接口由我们设置**（参数名是 ros2_control 暴露的接口、值是我们定的, 住 `<robot>_controllers.yaml`: `thread_priority: 80` / `cpu_affinity: 1` / `lock_memory: true`; 核1=CM update, 核2 预留总线）。`lock_memory` 依赖 `ulimit -l unlimited`（非 root 下失败仅 WARN 不阻塞）。

All control/sim launches take a **required** `robot:=<机型>` argument (resolves `unistackbot_description/arms/<robot>/` + `unistackbot_bringup/config/<robot>_controllers.yaml`; unknown robots fail fast with the available list). Current robots: `piper` (6-DoF 球腕 arm + gripper — IK 决策卡 `arms/piper/ik_decision_card.md`: 闭式解析解定路), `xarm7` (7-DoF arm, vendor: UFACTORY — see `arms/xarm7/README.md` + `arms/xarm7/ik_decision_card.md`).

## Architecture

Three layers orchestrated by `unistackbot_bringup`. `unistackbot_description` is横切层 (cross-cutting), consumed by all layers via URDF/TF:

```
unistackbot_bringup            → 顶层启动 / 参数编排
unistackbot_demo               → 控制 demo 独立包 (三命令域: 末端位姿流/关节慢流/关节满速流)
unistackbot_controller         → 控制器集成 (CM 笛卡尔流式 + JointStream 关节流式) + 工具节点
unistackbot_algorithm          → 纯算法库 (FK/IK/种子库, 零控制器依赖)
unistackbot_hardware           → 硬件抽象 / 驱动通信     (hardware_interface, pluginlib, serial)
  └─ unistackbot_sim_control/  → 仿真集成组 (容器目录, 三独立包; 见其 README)
      ├─ core     (包 unistackbot_sim_control: 统一仿真控制层 kinematic 后端 + /sim_control 服务)
      ├─ gazebo   (包 unistackbot_gazebo: Gazebo 集成 world + launch + gz 适配器)
      └─ mujoco   (包 unistackbot_mujoco: MuJoCo 集成 launch; MJCF 资产住 description)
物理硬件 / 仿真器
```

Morphology differences are isolated to: `description` model files, `hardware`/`sim_control` backend plugins, and `controller`/`algorithm` kinematics/balance algorithms. Upper-layer logic and the bringup framework stay shared.

### Package responsibilities

- **unistackbot_bringup** — top-level launch + controller config. `control.launch.py robot:=<机型>` starts `robot_state_publisher`, a standalone `ros2_control_node`, and spawners for `joint_state_broadcaster` + `joint_stream_controller` (+ CM inactive); `robot` is required and no robot name is hardcoded anywhere in the launch.（demo 脚本已迁出——见 `unistackbot_demo`）Controller config is per-robot: `config/<robot>_controllers.yaml` (update_rate 500 Hz; JointStream position command interface + RT 线程调优经 yaml 预留接口, 见上), shared by both the standalone node and Gazebo mode. 编排层最小版 (0d, 2026-09-21): `src/supervisor_node.cpp` —— 非RT诊断节点 (设计裁决: diagnostic_msgs), 汇聚 cm status (stream_stale/结果码/静默) + /joint_states 断流检测 (ERROR) + /ee_state 断流 (WARN) → `/supervisor/alerts` (DiagnosticArray @1Hz), 三链 launch 均拉起; 纯订阅+定时器零轮询, 任何被监控源死亡表现为话题静默被陈旧检测捕获。机型验收登记表 `config/<robot>_acceptance.yaml`（形态指纹/容差/mimic/软关节单一事实源, `test/verify_robot.sh` 启动即断言指纹, 缺表 fail-fast）。
- **unistackbot_demo** — 控制 demo 独立包（2026-09-23 从 bringup/scripts 独立）: 三命令域演示程序（§16.2 透传原则）= 链路契约的独立外部消费者（关节表/限位/mimic 自持解析 URDF、频率自校准、零机型硬编码）。① `demo_cartesian.py` 末端位姿流: 三 pattern `up`(z 往返, `--traverse` 插值流 @`--stream-hz`) / `axes`(抬升基准+8 向 2cm 逐点扫描, 诚实拒绝记"拒"不判死) / `sweep`(大幅: 抬升+前伸部署→r10cm 圆周 16 点→收回, `--lift/--fwd/--span/--speed`; piper 探针: 零位仅 +z 可行, 部署位邻域 ±150mm 球全解, 活链 22/22 腿 11.7s)。② `demo_motion.py --hz 50` 关节慢流（JS hold 填充）。③ `demo_joint_fullrate` **C++** 关节满速流（=控制频率, hold 退化=透传; **python rclpy 500Hz 全抖动故 C++**; rclcpp wall 定时器下发, 校准用消息 stamp 差分中位数——到达时刻差会被轮询量化偏差低读(实测 480.5 假象); mock 实测 500.0Hz 校准/99.9% 达成, 结束如实报告名义/实际）。
- **unistackbot_controller** — 控制器集成 + ROS 工具节点；算法库住 `unistackbot_algorithm`（2026-09-17 拆分），本包 `<depend>unistackbot_algorithm</depend>` 只消费类型。两个自研控制器:
  - **CartesianMotionController**（P1.5; **2026-09-23 §16.6 语义统一改版**: 每条 ~/target 消息=最新终点, 流式/终点=同契约频率档, 默认可打断）: `cartesian_motion_controller/` 自包含文件夹。契约不变: `~/target`(PoseStamped, reliable+KeepLast1, base系) / `~/control`(TRACKING/HOLD) / `~/status`(误差+min_sigma+结果码+stream_stale, 20Hz)。**新通路**: 回调线程逐条 IK(种子=实测通道 meas_ch_, COLD_START, `ik_timeout_ms` 必填) → SpLatest<SolvedTarget> → RT 环 OtgStream C2 追最新关节终点(vmax=URDF/a与j=机型资产必填) → applySolution 皮带扣(NaN门+限位+步长)。**旧机制退役**: RT 内流式 IK/500µs 预算/worker 冷启动/线性减速窗/give_up; 断流=OtgStream 自目标刹停; HOLD=自目标保持; 重激活水位防重放。update_rate 定源=权威参数优先(yaml `/**` 通配节, 旧中位数污染 35 倍减速 bug 已修)。mock E2E: 稳态 0.0885mm=账地板/sweep 22/22/reject 4/4(负路径)/WCET p50 0.9µs p99 24µs。验证 SOP 见 `docs/guides/ik_validation_playbook.md`。 Scope update (2026-09-17): 本包承载通用基础设施控制器（OTG ingest gate 等安全链成员 + 插件模板）**和算法工程**——自研算法控制器落地本包。Full boundary analysis: `docs/architecture/hardware_framework_design.md` §14.
  - **JointStreamController**（2026-09-18）: 关节级 topic 流式控制器, 取代 JTC action 层（topic 性能优, 点流取代 goal/结果握手）。`joint_stream_controller/` 自包含文件夹。契约: `~/command`(`unistackbot_interface/JointCommand`, reliable+KeepLast(1); CSP/CSV/CST/MIT 四模式, 仿真链只消费 CSP)。插值档位参数 `interpolation: ruckig`（**链上默认 ruckig**, 2026-09-23 A/B 定音: 同轨迹 dv/拍 平滑 42-100×, C2 兜底第三方粗糙输入——生产姿态: 上层守不守规矩不可假设; hold=每周期步长饱和逼近已采纳目标, 保留为满速透传语义档; ruckig=OtgStream 点流整形, 慢上层 10-100Hz 率失配填充, 路径插值永不在控制器——形状轴归上层/MoveIt; **已知坑已修**: 校准块 otg_.init 清 initialized_ 后必须重锚 reset 否则激活 32ms 后 ruckig 永久冻结——F6 当年 ruckig 刹停属空洞通过, 需补"从运动中刹停"断言）。安全层与 CM 同款。**断流受控减速**（0c, 2026-09-21）: `stale_timeout_ms`/`stale_decel_ms`（链上 yaml 默认 200/200）——StaleWatch 判 `~/command` 断流后 hold 档=关节速度线性衰减刹停、ruckig 档=OtgStream 刹停, 恢复自动续接。rosidl msg 非 POD 不进 SpLatest——经 CmdSnapshot 翻译层（先例）。gz E2E: 50Hz 命令流跟踪误差 0.0053 rad, 与 CM 切换共存。
  - **EeStateBroadcaster**（2026-09-21, 仿真测试方案批次2）: EE 位姿独立反馈流, `ee_state_broadcaster/` 自包含文件夹。契约: `~/ee_state`(PoseStamped, base 系, FK(关节状态), 默认 50Hz)。**只读控制器**（零命令接口认领, 与 JS/CM/JSB 共存——CM 切走后 EE 反馈不断流）。旋转表示决策挂起: 先四元数载体 (TF 兼容), 决策后升 EeState.msg 加并行字段不破坏消费者。
  - **OtgGateController**（OTG 门, 2026-09-23 落地; **§16.6 语义统一后架构并入 CM, 退役中——下会话删代码**, 详见设计 §16.6）: "上层只有终点"的对齐服务, `otg_gate/` 自包含文件夹。流: `~/target`(PoseStamped, base 系, 点流) → 回调线程一次 IK(50ms 预算, ik_solver 必填无默认同 CM) → RT 环 OtgStream 每拍整形 → `/joint_stream_controller/command`(CSP @500Hz 实测, JS hold 满速退化=透传)——本路径引入关节 C2+加速度界。**零接口认领只读控制器**(与 JS/CM/EE 全共存, inactive 注册, spawner 手动拉起); 非链关节(手指)=实测回显; RT 零运动学零分配; URDF 命令关节解析与 JS 同源(`controller_common/urdf_command_joints.hpp`)。mock E2E: 到位 0.088mm(账地板)/命令流 500.03Hz/不可达诚实拒绝保持。
  - 工具节点: `fk_tool` / `ik_tool`（算法调试 CLI, `tools/`; fk_tool 的 `--base/--tip` 必填无默认: piper `base_link`/`link6`, xarm7 `link_base`/`link7`）+ `ik_demo_node`（RViz 交互拖动 → 发 CM `~/target`, 2026-09-21 重写）。
- **unistackbot_algorithm** — 纯算法库（零控制器依赖）, 2026-09-17 从 controller 拆出: `ik_solver`（**IkSolver 抽象接口**, 2026-09-21: CM 的可插拔求解层——5 条硬契约在 `ik_solver.hpp`（失败不改 out_q/墙钟预算/SolveMode 语义/RT 纪律/stats）; 新数值求解器=实现接口+CM 加选择分支（CM 参数 `ik_solver` **必填无默认**——缺配置 fail-fast 列可选项, 主路径与 worker 双分支同款; 现可选 `dls | analytic_piper`, per-robot yaml 显式声明: piper=analytic_piper, xarm7=dls, 启动日志回显激活解）+playbook 六阶段; 用户 7 轴数值解按此接入, 与 DlsIk 平级 AB 对比）、`urdf_fk`（kdl_parser 建链 + FK/雅可比/限位/链序名 + `JointAxis`/`jointAxesAtZero()`（结构指纹数据源; urdfdom 父链累计 origin 抽轴——kdl_parser 的 Joint::axis 是 private 不可用）, RT 零分配; `test/check_fk_tf.sh` TF 对拍实测 ~4e-13, 机型通用化——frame 从 yaml 读）+ `dls_ik`（Eigen SVD 阻尼伪逆 + boxed DLS 避限位 + SolveMode 流式/冷启动解耦 + 四分支代表种子 + `timeout_ns` 墙钟预算 + `min_sigma` 奇异遥测 + `NEAR_SINGULAR` 失败分类; 600 样本 Oracle 冷启动 99.5%）+ `analytic_piper`（piper 球腕 6 轴闭式解析解, IKFast 生成, 2026-09-22: `check_spherical_wrist` 结构指纹（三轴共点最小二乘残差, piper 实测 59µm≤1mm; init() 指纹不过=配错机型 fail-fast 拒启——选拒不代选）+ `AnalyticPiper:IkSolver` 适配层（全解枚举→限位过滤**带 `kLimitEpsRad`=1e-3 rad 微容差**（2026-09-23 实锤: 零位压限位线的臂回零解微越界 j2=-8.8e-5/j3=+3.4e-4 被严格过滤全拒——"上得去回不来"; 只放宽接受判定, 输出仍经 applySolution 限位 clamp+write 层硬钳）→距 seed 最近解; 0 解=UNREACHABLE/全解越限=LIMIT_CONFLICT/冗余偏好=UNSUPPORTED）+ `piper_ikfast.cpp`/`ikfast.h` vendored 生成物（Apache-2.0, CMake 单文件关固有警告）+ 再生管线 `gen/`（podman+OpenRAVE 容器, 规范化 URDF 88µm 偏差归零——IKFast 要求结构精确; 跑通配方含三坑见 `gen/README.md`）; 测试 `test/run_wrist_check_test.sh` = 指纹正反例 + FK-IK 回代黄金测试（随机 q→FK→IK→FK, 198/200 解出, 最差位置误差 88µm, 2 跳过=工作空间边界）; 决策记录 `arms/piper/ik_decision_card.md`（DlsIk=7 轴冗余臂主场+全机型 fallback, 解析解=6 轴球腕正解）。已知 RT 欠账: IKFast IkSolutionList 内部小分配（≤8 解）, 同 urdf_fk KDL 暂存先例）+ 可选**种子库**（`loadSeedLibrary` 读机型资产 `arms/<robot>/ik/seed_lib_<robot>.txt`, COLD_START 阶梯级0.5 分支封顶 top-6; 预算档 1ms 96.2% / 2ms 96.9% / 5ms 98.3%; 生成器 `test/gen_seed_library.py` + 增量 `test/seed_lib_incremental.py`, xarm7 冻结 v1.1=12k 条）。
- **unistackbot_description** — URDF/Xacro, RViz config, `display.launch.py`.
- **unistackbot_hardware** — real-driver plugin skeleton (empty `placeholder.cpp`); deps (`hardware_interface`, `pluginlib`) and the `device`/`baudrate`/`loop_rate` params in `piper_ros2_control.xacro` anticipate the real Piper CAN driver.
- **unistackbot_sim_control** — 统一仿真控制层（ 对 ros2_control 提供统一插件接口，对内按后端分类）:
  - `SimControlHardware` (`SystemInterface`): URDF `<hardware>` 里固定写 `<plugin>unistackbot_sim_control/SimControlHardware</plugin>` + `<param name="backend">kinematic</param>`。接口按 URDF 声明镜像导出（effort 恒 0）；关节动态数量 ≤16，限位/`max_velocity`/mimic 全部来自 `<ros2_control>` 的 `<param>`。
  - 后端 `kinematic`（`BackendKinematic`）: 理想执行器 —— 限位 clamp + 每关节 `max_velocity` 饱和的一阶逼近；mimic 关节按 multiplier/offset 从源关节推导。
  - **write()/read() 最终防线**（2026-09-20, 阶段0a; **真机驱动同型照抄**）: read 侧状态有限性门(非有限保持上一拍); write 侧不论上层控制器是谁, 下发前逐一过门 NaN → 限位 clamp → 步长饱和, 全程故障计数器(fault_cmd_nonfinite/clamped 等) + 限流 WARN。激活时 guard arm 建基准, 重激活重新 arm。分层语义: 控制器层"拒绝", write 层"clamp"兜底。测试床: 仓库根 `test/fault_injection.sh` + `fault_inject_bed.py`（F1 断流/F2 NaN/F3 限位/F5 超速/F6 断流受控减速 五场景全绿; 2026-09-21 F6 顺带实锤修复两处潜伏 bug: ①Humble `get_update_rate()` configure 期无效→hz 改由首 16 拍 period 中位数校准（两控制器同修, step_limits 一直在按 hz=1 装配被 write 层掩盖）②JointStream hold 档步进从"每消息"改为"每周期"）。
  - `/sim_control/*` 服务（reset / set_joint_state / pause / resume / step）: 插件进程内自建（`on_configure` 启动、`on_cleanup` 销毁），命令经 SPSC 无锁队列交给实时循环。**`/sim_control` 契约住本包**（`include/unistackbot_sim_control/sim_control_contract.hpp` 的 SimCommand/SimCmdType/SimControlServer; 2026-09-17 终局: 实现归实现的家、契约归契约的主人, 曾短暂上收 interface 后回迁）; 应答 `success=true` 只代表命令已被接受（入队/ign 请求已发出），不代表执行完成。注意：控制器激活时其保持命令每周期都会覆盖瞬移，**set_joint_state/reset 需在 pause 下使用**。
  - 日志走 ulog 宏（`ULOG_INFO`/`ULOG_ERROR`，非 RCLCPP）: `on_init` 里 `ulog_init` —— 终端 sink 恒开（launch 捕获 stdout），URDF `<ros2_control>` 块加 `<param name="ulog_file">` 可选开文件 sink；`on_shutdown` 里 `ulog_shutdown` 排空落盘。
  - gz 链路的 `/sim_control/*` 由 `unistackbot_gazebo` 包里的 `sim_control_gz_node` 承载（gz 知识归集成层; 契约头从本包 include——依赖方向 gazebo → sim_control）。
  - 回归测试 (均自包含, 需先 colcon build + source, 脚本自行定位工作区并套用 `~/cyclonedds.xml`): `test/verify_robot.sh <robot> [--with-gazebo] [--with-mujoco]`（2026-09-21 重写: JS 点流 + CM 切换断言, 助手 `test/verify_motion.py`, 各链关节集自适应——链上已无 JTC）；`test/fault_injection.sh --robot <piper|xarm7> [--chain mock|mujoco]`（F1 断流/F2 NaN/F3 限位/F5 超速/F6 断流受控减速/F7 mimic 断言; F4 状态跳变由 smoke 覆盖; 死链硬门防空洞 PASS; 机型阈值账内置, 关节集按 机型×链 装配）；`test/smoke_sim_control.sh` 深测 `/sim_control` 服务语义（piper 关节表，断言含 mimic/限位拒绝/reset）。
- **unistackbot_interface** — 公共接口定义包（ROS msg/srv/action + 纯 C++ 共享契约头）：跨包/跨仓库共享的类型放在这里（单一事实源，供算法团队外部仓库依赖）。已落地：① 机器人级契约: `RobotFeedback`/`RobotCommand`（含冗余偏好与 MIT 模式; 与 `JointCommand.msg` 字段对齐——单一契约两载体）、`IkResult`/`SimResult` 分层结果码、`joint_capacity.hpp` 的 kMaxJoints 唯一定义; ② 消息: `JointCommand.msg`（CSP/CSV/CST/MIT 四模式点流）、`CartesianControl.msg`（TRACKING/HOLD 事件通道）、`CartesianMotionStatus.msg`; ③ `test_contract.cpp` 契约自检。**`/sim_control` 契约不住本包**——住 `unistackbot_sim_control`。
- **unistackbot_common** — 组件库，**不是 ROS 包**（无 package.xml/CMakeLists，colcon 自动忽略）：纯代码存放层，保持可在非 ROS 环境（RT 主站线程/单元测试/ARM 交叉编译）中直接复用。现有组件（每个独立子文件夹 = 文档 + 实现 + 测试三件套）:
  - `sp_latest/` — 双缓冲覆盖写/取最新原语（最新 **1** 个，"值通道"，seqlock 宣告式，seq 位宽自适应 64/32 位）；sim_control `threaded` 后端三通道 + CM worker 回灌 + JointStream 命令通道用它
  - `stale_watch/` — 陈旧看门狗（IDLE/LIVE/STALE 三态, 周期计数制零 syscall; 配 SpLatest::seq() 轮询判"上游还活着吗"; 0c: JointStream/CM 断流受控减速的判定原语, "无流=待命≠断流"）
  - `sp_ring/` — SPSC 无锁环形队列（**逐条必达**，"事件通道"，满拒新 push 语义）；sim_control 的 `sim_command_queue.hpp` 引用 `SpscRing`，域类型留在 sim_control
  - `mpsc_ring/` — 多写单读覆盖式无锁环（**丢旧保新**：日志缓冲/滑动窗口/音视频环; TSAN 零竞争）
  - `ulog/` — 高性能异步日志组件（前端宏：级别过滤→snprintf 定长 POD→无锁入队；后端单线程双 sink 终端+文件、error 强刷、10MB×5 轮转; emit 零 malloc）
  - `ruckig/` — vendored [pantor/ruckig](https://github.com/pantor/ruckig) v0.14 + `otg_stream.hpp` 防御封装（社区版已知 -101/-110/-111 数值风险的防线; `update()` 只有 Ok/Hold 两结局, Hold 输出恒有效; RT 补丁一处: trajectory.hpp SetIntegrate 模板化消除堆分配, 稳态零 malloc）+ `cartesian_shaper.hpp` 笛卡尔位姿流整形器（2026-09-23: 位置 OtgStream<3> + 姿态锚点切空间 OtgStream<3> 组合, 社区版无 SE(3) 的补齐, 契约/观测面与 OtgStream 同款; 教训: 每拍 reset 注入实测状态不可行——reset 强制下拍重算致推进/停滞交替）。**OtgStream = 全工程平滑唯一入口**（2026-09-23 裁决: 无人裸 include ruckig.hpp, 新平滑消费者一律经 OtgStream/CartesianShaper; CM 步长饱和是安全钳位不走此入口）; OTG a/j 限值 = per-robot yaml 机型资产（`max_acceleration`/`max_jerk`, URDF 无此项）。**不是单头文件**: `src/` 13 个 .cpp 需参与编译（controller 的 CMake GLOB 直引）。
  - `rt_tune/` — `RtTune` RT 线程调优参数化应用（亲和性/调度策略只能作用于调用线程自身, 仅供**自有线程**用; 控制器管理器主线程走 yaml 预留参数接口 (值我们定), 二者分工勿混）。测试/基准 `bench_rt_tune.cpp`: 双线程 FIFO80 钉核1/2 周期唤醒自统计 (p50/p99/max+阈值计数) + apply 断言; 三档参考数据见 README（2026-09-20, p99 稳定 8-11µs）。
  - sim_control 与 controller 的 CMake 都以 `$<BUILD_INTERFACE:...>/../unistackbot_common` 直引组件库整个目录（monorepo 内，未 install——对外发布前需调整）
- **unistackbot_gazebo** — Gazebo integration, single chain: `ign.launch.py` (Gazebo Sim/Fortress via `ros_gz_sim` + `empty_ign.world` + bridges + `sim_control_gz_node`). The Gazebo Classic chain was removed 2026-09-17 (EOL; recover from git history if ever needed). Takes a required `robot:=<机型>`; no robot name is hardcoded. The controller manager lives inside the sim's ros2_control plugin — no standalone `ros2_control_node`. Constraints baked into the launch, each fixes a hard failure observed on dev machines:
  - URDF is re-serialized to a **single line** before use: the plugin forwards it to the CM as a `--param robot_description:=<urdf>` rule and rcl's parser rejects newlines → CM never starts.
  - The sim gets the URDF via a temp file (`-file` for spawn_entity / `create`), not the `/robot_description` topic: TRANSIENT_LOCAL latched re-delivery is unreliable under iceoryx/SHM CycloneDDS configs.
  - DDS comes from the machine-wide `~/cyclonedds.xml` (`CYCLONEDDS_URI` in `.bashrc`): binds `lo` + unicast `Peers 127.0.0.1`. This machine's `lo` lacks the MULTICAST flag, so unicast-only discovery intermittently dropped late joiners; the Peers bootstrap fixed it. The launches deliberately do NOT override `CYCLONEDDS_URI`. Run at most ONE launch stack at a time — leftover same-name nodes (robot_state_publisher / controller_manager) poison new runs.
  - The ign chain prepends the description package's ament share root to `IGN_GAZEBO_RESOURCE_PATH`: URDF→SDF conversion rewrites `package://` to `model://`, and Fortress resolves those only via that env var.
  - `scripts/gz_clean.sh` (`ros2 run unistackbot_gazebo gz_clean.sh`) kills the whole launch process family — ign servers routinely survive launch shutdown and poison the next run; run it before every launch. **永远单独执行**——它的 pkill 模式会击杀同一命令行里含 "ros2 launch" 字样的宿主进程。残留的典型症状: `Controller already loaded` / 臂不动 / `Failed to find a free participant index`（DDS 参与者被占满）。
  - **gz 链关节冻结 (2026-09-21 破案, 非 0.7.21 回归)**: gz/ODE 行为 (上游 gz_ros2_control issue #165)——关节静置恰好压在限位上被限位约束咬死 (捕获区微米级), 无法再命令离开; joint2/joint3/gripper 启动位=限位线故开链即冻。修法=官方 demo 同款: ign 分支三关节 position state_interface 加 `initial_value` 子参数 (0.01/-0.01/0.001) 略离限位; **运行时残余**: 命令精确停限位仍会冻, gz 测试流程避免。手指无耦合漂移=插件 mimic 只认 ros2_control 块参数 (URDF `<mimic>` 标签被 sdformat 丢弃, 0.7.20=0.7.21), 耦合方案待裁决 (`_mimic` 接口后缀污染 /joint_states), 独立工作项。
  - `src/sim_control_gz_node.cpp` — `/sim_control/*` 的 gz 适配器（**纯 ROS 构建，零 ign 编译依赖**）。gz 链路的 ros2_control 插件是 `GazeboSimSystem` 而非 SimControlHardware，故 `/sim_control/*` 由这个 side-car 独立节点承载：pause/resume/step 经 launch 里 parameter_bridge 桥接的 `/world/<world>/control`（`ros_gz_interfaces/srv/ControlWorld`）下发；`reset` 拒绝（Fortress 实测有毒：返回 success=true 但世界 negative-timestep 停摆）、`set_joint_state` 拒绝（无原生等价，Garden+ 才有）。契约头从 `unistackbot_sim_control` include。由 `ign.launch.py` 启动，`world` 参数（`unistack_world`）须与 `empty_ign.world` 的 `<world name>` 及桥接服务名一致。
- **unistackbot_mujoco** — MuJoCo 集成 (第三条控制链, 2026-09-20): `mujoco.launch.py` 纯 launch 胶水零 C++。控制器管理器宿主 = `mujoco_ros2_control` 的**定制 `ros2_control_node`**（controller_manager + MuJoCo 引擎 + 物理线程 + 渲染同进程; 上游 ros2_control 合入仿真 PR 后可换回标准节点——临时措施）。真实动力学回归链, 对照 mock (理想执行器)/gz (ODE)。要点:
  - 安装: apt 二进制 `ros-humble-mujoco-ros2-control` 0.1.2 (内嵌 MuJoCo 3.12.0); RT 线程调优经同一 yaml 预留接口 (500Hz/FIFO80/核1) 实测生效, 与 mock 链同份 `<robot>_controllers.yaml`。
  - robot_description 参数直供（单行化, 同 gz 链坑）; `use_sim_time: true`; MJCF 缺失/未知机型 fail-fast。
  - headless 默认 **false 带界面** (2026-09-23 翻转, 与 gz 链 gui 默认对齐; URDF 硬件参数——apt 0.1.2 无 `MUJOCO_HEADLESS` 环境变量支持, PR #157 未随发布; 基准/录制及全部工具脚本显式 `headless:=true`, 自动路径不受影响)。GUI 收场有 GL 析构段错误 (demo 同款, 控制器已干净关闭, 无害)。
  - MJCF 资产住 description 包 `arms/<robot>/mujoco/`（URDF→MJCF 用包自带 `robot_description_to_mjcf` 转换 + 手工策展; mesh 直引 collision STL 零复制; **equality 方向坑**: MuJoCo 3.x 实测 `joint1 = poly(joint2)`, 官方文档文字相反——详见 `arms/piper/mujoco/README.md`）。
  - `/sim_control` 适配器已接 (0d, 2026-09-21): `src/sim_control_mujoco_node.cpp` 桥 pause/resume/step/reset 到原生四服务 (reset 原生支持比 gz 强; set_joint_state 诚实拒绝——reset_world 覆写是世界级复位语义≠瞬移; 映射表+E2E 实测在 `unistackbot_mujoco/README.md`)。另有 gz 链没有的 `apply_external_wrench`。无孤儿进程问题（普通进程收场, 不需要 gz_clean 类脚本）。

## RT 基准与调优（仓库根 test/）

- `test/rt_chain_bench.sh <标签> --robot <piper|xarm7> [--chain mock|mujoco]` — 一键 RT 基准套件: cyclictest 三档(无负载/50%/90%, `cpu_load.py` 造载) + SMI/中断采样 + hwlatdetect + RT 带宽记录 + CM 链路三档 E2E(WCET/误差/收敛; CM 目标从 ee_state 动态生成, 机型无关); 终端分步进度, 报告入 test/results/。结果 markdown 入库 `test/results/`（mock 权威基线 `rt_baseline_isolcpus.md`, 2026-09-20 隔离全套生效后: 负载档 Max 6-12µs; mujoco 基线 `rt_mujoco_baseline.md`: p99 4-55µs/误差 0.74mm/**max 508.6µs = 500µs IK 预算剪枝签名**——激活后首批冷 IK 烧穿预算, worker 3 拍自愈, 无害; warmup 多轮加固用户裁定暂缓排批次4; 历史基线同目录; piper 机型首跑报告 `rt_piper_first_0922.md`）。内核背景: 本机是**低延迟内核 (lowlatency), 不是 PREEMPT_RT**——措辞勿混。cmdline 已加 `isolcpus=domain,managed_irq,1,2 nohz_full=1,2 rcu_nocbs=1,2 irqaffinity=0,3-27`（隔离核1/2）。
- RT 裁决 (2026-09-20, 勿回退): `sched_rt_runtime_us` **保持默认 950000**——"500ms 事件 = RT throttling" 归因已翻案 (SCHED_OTHER 负载不进 RT 带宽账本, RT 占空比 <1% vs 95% 门槛; 详见基线文档翻案段); 950ms 默认值是同核 RT 疯转时的保险丝, 不持久化 -1。
- isolcpus 坑: cyclictest 直接 `-a 1,2` 会 FATAL（不突破继承亲和 mask）——必须 `taskset -c 1,2 cyclictest ...`（套件已内置）。
- `test/rt_tune_boot.sh [0-3]` — **每次开机手动执行** `sudo bash test/rt_tune_boot.sh` (默认 = 核0-3 前四核): idle 按退出延迟>2µs 禁深睡 (x86 通用, 兼 ACPI/intel_idle 档表) + performance governor + EPP; uncore 锁频段默认关 (对照实验无收益)。幂等, sysfs 直写零依赖; 换机器改脚本内 DEFAULT_CPUS。生产环境再转 systemd oneshot。
- `test/rate_matrix_bench.sh <标签> --robot <机型> --chain mock|mujoco --bus-hz 500|1000 --cmd-hz 50|100|200 [--domain joint|cartesian] [--seconds 30]` — **频率矩阵** (2026-09-23): 控制端低频 × 总线/CM 频率的组合测试接口, 一格一跑 (清场→起链→录包→demo→指标→结果行自动追加 `test/results/rate_matrix_<机型>_<链>.md`), 指标 = 命令达成Hz/JS实测Hz/max|dv|拍/2阶差。**gz 链忽略 bus_hz 不报错** (CM 在 gz 插件进程内, launch 参数本就不可达; 用户裁决 2026-09-23: 三链各有各端功能, 频率矩阵属 mock 理想执行器 + mujoco 动力学回归两域, gz 要换频直接改 yaml `/**.update_rate`)。配套: launch `bus_hz:=` 参数 (运行期覆盖 update_rate, yaml /** 通配节单一事实源)。
- 跑链前先 `ros2 run unistackbot_gazebo gz_clean.sh`; 详见记忆 `p15-progress.md` 环境坑（杀进程用精确 PID 勿 pkill 等）。

## ros2_control wiring (cross-file contract)

`piper.urdf.xacro` declares five args: `use_gripper`, `use_ros2_control`, `use_world`, `use_gazebo`, `headless` (mujoco 链专用). `use_gazebo` is four-way (xacro quirk: `$(arg …)` turns `true` into Python `True`, so comparisons must match both):

- `use_gazebo:=false` → `unistackbot_sim_control/SimControlHardware` (backend=kinematic; bringup's standalone `ros2_control_node`)
- `use_gazebo:=true`/`classic` → Gazebo Classic `gazebo_ros2_control/GazeboSystem`（**链已删**，仅 xacro 分支保留为惰性能力，无消费者）
- `use_gazebo:=ign` → Gazebo Sim (Fortress) `gz_ros2_control/GazeboSimSystem` + `gz_ros2_control-system`
- `use_gazebo:=mujoco` → `mujoco_ros2_control/MujocoSystemInterface` + 硬件参数 `mujoco_model`（MJCF 路径, `arms/<robot>/mujoco/<robot>.xml`）/ `sim_speed_factor` / `headless`（apt 0.1.2 无 `MUJOCO_HEADLESS` 环境变量支持, headless 走此参数）

All `<gazebo>` plugin blocks live in each robot's `<robot>_ros2_control.xacro` and point at `$(find unistackbot_bringup)/config/<robot>_controllers.yaml` — the *description* package depends on bringup's installed config in both sim modes.

Contracts to keep in sync when changing joints or interfaces:

- `SimControlHardware` accepts dynamic joints (≤16). Each joint must declare exactly one `position` command interface; state interfaces mirror the URDF (position required, velocity/effort optional, effort is always published as 0). Per-joint `max_velocity` `<param>` caps the kinematic backend (default 5 rad/s).
- `gripper_joint1`/`gripper_joint2` follow `gripper` via URDF `<mimic>` tags **plus** `<param name="mimic">`/`multiplier` entries in the `<ros2_control>` block (gz/Classic 链需要; kinematic 后端在 `step()` 推导)。**mujoco 链不同**: 手指在 `<ros2_control>` 里只声明 state 接口（passive）, 跟随由 MJCF 侧 equality 约束实现（方向坑: MuJoCo 3.x 实测 `joint1 = poly(joint2)`, 见 `arms/piper/mujoco/README.md`）→ JointStream 解析 URDF 后只管 7 个主关节。
- Gotcha: inside `piper_ros2_control.xacro` the gripper block tests `$(arg use_gripper)` — the *global* xacro arg, not the macro's `use_gripper` param, which is therefore dead there. It only works because `piper.urdf.xacro` passes the arg through unchanged.
- MJCF 资产 (`arms/<robot>/mujoco/`) 与 URDF 惯量/结构/限位联动——URDF 改动后须再生成（`mujoco/README.md` 有命令）; mesh 直引 `../meshes/collision/` STL, 无复制资产。

## Description package conventions

Robot models live under `unistackbot_description/arms/<robot>/urdf/` with mesh assets under `arms/<robot>/meshes/{visual,collision}/`. Mesh paths in URDF use the `package://unistackbot_description/...` URI — keep this prefix when adding new robots so resolved paths work after `--symlink-install`. `arms/<robot>/original/` = 导入时点原始 URDF 留档（死档案 diff 基线，方法见 `arms/original_README.md`；git 仍是真版本管理）。

The **Piper arm** is the reference model and the pattern to follow for new robots:
- `piper.urdf.xacro` — top-level entry; declares the xacro args above and conditionally includes the macros below.
- `piper_macro.xacro` — `piper_arm` macro defining `base_link` → `link6` with a `parent` param. With `use_world:=true` a `world` link is added and the arm is fixed to it; otherwise `base_link` is the root (what Gazebo spawns).
- `piper_gripper_macro.xacro` — `piper_gripper` macro (flange + gripper_base + prismatic `gripper` joint + two mimic fingers). Keep an `<inertial>` on the otherwise empty `gripper_link` — Gazebo drops inertia-less links, which would remove the `gripper` joint the controller config depends on.
- `piper_ros2_control.xacro` — `<ros2_control>` block with the plugin switch described above; joints declare a `position` command interface and `min`/`max`/`max_velocity` limit params the kinematic backend enforces.
- `piper.urdf` and `piper_with_gripper.xacro` — flat (non-xacro-macro) variants kept alongside the macro versions.

`display.launch.py` uses `OpaqueFunction` to defer xacro processing so launch args can be passed into the `xacro` command substitution. It selects `joint_state_publisher_gui` vs `joint_state_publisher` via mutually exclusive `IfCondition`/`UnlessCondition` on the `gui` arg.

## docs/ — 设计文档与课程笔记（中文，2026-09-21 分类：architecture/bus/sim/guides）

Not build input, but two items are normative for code. 分类索引见 `docs/README.md`。

- `guides/cpp_style_guide.md` — **repo-wide C++ style, binding**（全工程适用）; the Conventions section below defers to it.
- `architecture/hardware_framework_design.md` — master working doc for the real-hardware layer (`unistackbot_hardware` + RT core): 分层架构、线程模型、协议后端契约（master.hpp 五要素、形态盲对象模型）、v3 语义状态机、形态与算法边界（§14）、与 ros2_control 生态的对照审核（§15）. Status: 讨论中、未收口 (2026-09).
- `architecture/rt_software_architecture.md` — RT software architecture manifesto（七支柱）. Normative for how RT threads and cross-thread data paths are written.
- `bus/socketcan_master_design.md` / `bus/ethercat_master_design.md` — full designs of the two bus-master backends (SocketCAN CAN FD; IgH ecrt, 1kHz + DC), structurally parallel, both written against the master.hpp contract.
- `bus/timesync_design.md` — MCU↔主机时间同步两帧协议（主机为时间服务器, MCU 落 sync.csv 锚点表; v1.9 USB 转串口部署版）。
- `bus/udp_internal_bus_argument.md` — 内部总线选型论证（插值器以下链路不采用 UDP 的文献证据版, 服务于 CAN FD 定版决策）。
- `guides/linux_rt_guide.md` — RT system tuning (PREEMPT_RT, isolcpus, IRQ affinity, cyclictest/抖动排查).
- `guides/ik_validation_playbook.md` — 换臂/换 IK 求解器的六阶段验证方法论 + 参数审计表 + 验收模板 + §9 球腕 6 轴机型解析解接入流水线（piper 实例化, 2026-09-23 收拢——下一台同构型机型的照抄入口）。
- `sim/sim_environment_and_test_plan.md` — 三链仿真现状 / 互补性定位 / 对上接口契约 / 分链测试矩阵 / 实施批次（2026-09-21 批次1-3 全勾; 5-9 已排）。
- `sim/sim_validation_pipeline.md` — 机型接入→验收归档的端到端流程（A-G 七阶段门禁, 可复用资产）。
- `sim/chain_interface_dictionary.md` — 链上接口词典（运行面可观测话题/类型/频率/服务的规范描述; 定义先行——实现与词典冲突=实现 bug; piper mock 为基线, 三链差异见其 §6）。
- `sim/acceptance_report_template.md` — 机型验收报告模板（A-G 阶段同构表, 手动首跑版; run_acceptance.sh 就位后由引擎自动填写）。
- `control_course/` — control-theory course notes（传递函数 → 三环级联、PM/带宽账）. Background for the numbers cited in the design docs; not code documentation.

## Conventions to preserve

- Math/engineering rule: **for mathematically mature operations (linear algebra, SVD/pseudo-inverse, optimization, polynomial root-finding), always prefer a mature library (Eigen, KDL) over hand-rolling; self-written code is reserved for strategy and composition** (e.g. our IK null-space objective, seed ladder). Numerical layer = libraries; policy layer = ours.
- **机型/拓扑/算法选择类参数一律无默认值**（2026-09-22 用户纪律）：robot、model、--base/--tip、`ik_solver` 等必须显式指定，fail-fast 列可用项——默认值是隐性硬编码；算法↔机型的对应永远由人在 per-robot yaml 里显式声明，永不自动推断（结构指纹只做选拒校验，不做代选）。

C++ style is governed by `docs/guides/cpp_style_guide.md`. Non-negotiables: Allman braces on their own line everywhere; **Tab indentation** (display width 4), never spaces; filename = snake_case of the main class (`sim_control_hardware.hpp` ↔ `SimControlHardware`); `PKG__FILE_HPP_` include guards; `k`-prefixed constexpr constants, `snake_case_` members; **explicit error flow** — our code never throws (errors via return value + message, `[[nodiscard]]` on error-returning functions); system calls that are documented to throw (`std::stod` etc.) are caught at the call boundary and translated into return values — exceptions never enter the control chain (style guide rule 27). All C++ packages compile with `-Wall -Wextra -Wpedantic` and `cxx_std_17`, zero warnings. RT hot paths: lock-free structures/atomics only — no mutex, malloc, printf, or IO in the control loop (RT code discipline lives in `docs/guides/linux_rt_guide.md` part 2, not the style guide).
- **ROS I/O 纪律（2026-09-21 用户定稿，控制器/RT 线程相关节点一律适用）**：① RT 线程内发布一律 `RealtimePublisher` trylock——非阻塞，拿不到锁跳本拍，序列化/DDS 发送在其后台线程做；② 订阅回调永不进 RT 线程——executor 回调经无锁通道交接（`SpLatest` 值通道 / `SpscRing` 事件通道，POD 硬断言），RT 侧读撕裂/无更新沿用旧值；③ 任何新节点不得给 CM RT 线程引入等待、锁或分配。先例：CM target/worker 回灌/status、JS CmdSnapshot、EE broadcaster。验收 = rt_chain_bench WCET 前后对比。
- Each package uses `ament_lint_auto` with `ament_lint_common` for tests — match this when adding new packages.
- `bringup`'s `package.xml` lists the packages it orchestrates as `exec_depend` (including `unistackbot_sim_control`); `unistackbot_gazebo` similarly declares its runtime deps. Add new runtime-consumed packages there.
- Keep `.gitkeep` files in empty `launch/`, `config/`, `include/` directories so the package structure survives in git.

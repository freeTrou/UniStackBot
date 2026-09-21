# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 启动必读（每次会话最先执行）

先读记忆目录恢复工作上下文，再读本文件：
1. `~/.claude/projects/-home-work-project-git-project-UniStackBot-ws-src-UniStackBot/memory/MEMORY.md` — 记忆索引
2. 重点读 `p15-progress.md`（进行中项目的快照：已完成/验收数据/下一步/环境坑）
3. 记忆是时点快照不是实时状态——引用的文件/参数先对照当前代码核实再用

## Project Overview

UniStackBot is a ROS 2 Humble workspace for a general-purpose, multi-morphology real-time robot control framework. It targets fixed-base arms, wheeled bases, quadrupeds, wheeled-arm humanoids, and bipedal humanoids from a single layered architecture. The first concrete robot is the **Piper arm**: URDF/Xacro description + the unified sim-control layer (`unistackbot_sim_control`, ros2_control hardware plugin with pluggable backends) + Gazebo integration. `unistackbot_controller` hosts the CartesianMotionController CM plugin (P1.5, IK servoing on the control loop) and JointStreamController (关节级 topic 流式控制器, 2026-09-18 取代 JTC action 层); `unistackbot_hardware` (real driver) is still a skeleton.

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

Four launch entry points, all currently Piper-specific (控制链三选一: mock/gz/mujoco):

```bash
# 1. Visualization only (RViz + joint_state_publisher_gui; ros2_control off by default)
ros2 launch unistackbot_description display.launch.py
#    args: model:=<xacro> use_gripper use_ros2_control use_world gui rviz

# 2. Mock control chain: standalone controller_manager + SimControlHardware (kinematic backend), no simulator
ros2 launch unistackbot_bringup control.launch.py robot:=piper   # robot:=xarm7 等; use_rviz:=true|false

# 3. Gazebo Sim / Fortress: controller manager inside gz_ros2_control
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper        # gui:=true|false, use_rviz:=true 可选

# 4. MuJoCo (mujoco_ros2_control, 2026-09-20): controller manager = mujoco 定制 ros2_control_node
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper     # headless:=true(默认)|false, use_rviz:=true 可选
```

All control chains spawn `joint_state_broadcaster` + `joint_stream_controller` (active) + `cartesian_motion_controller` (inactive 注册). **JTC 已从链上移除** (2026-09-18)。命令通道:

- 关节空间: `/joint_stream_controller/command`（`unistackbot_interface/JointCommand`, reliable+KeepLast(1), 点流语义——每条消息是"最新目标"）。仿真链（position 命令接口）只消费 CSP 模式。关节按 `joint_names` 字段对名映射，必须列全（**mujoco 链只列 7 个主关节**——手指 passive, 见 `unistackbot_mujoco/README.md` 差异表）。
- 笛卡尔空间: `ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped ...`（base 系）。
- 曲线分析录包: `ros2 bag record -s mcap /joint_states /joint_stream_controller/command /cartesian_motion_controller/{status,target} /tf`。

双控制器切换 (三条链同款): CM 以 inactive 注册, `ros2 control switch_controllers --deactivate joint_stream_controller --activate cartesian_motion_controller` 接管 (切回反向同理)。Intended workflow (README has the switch table): iterate algorithms on the mock chain, regress each version on Gazebo/MuJoCo, both green = pass.

RT 备注: 控制器管理器 RT 线程调优走**官方参数**（`<robot>_controllers.yaml` 的 `thread_priority: 80` / `cpu_affinity: 1` / `lock_memory: true`; 核1=CM update, 核2 预留总线）。`lock_memory` 依赖 `ulimit -l unlimited`（非 root 下失败仅 WARN 不阻塞）。

All control/sim launches take a **required** `robot:=<机型>` argument (resolves `unistackbot_description/arms/<robot>/` + `unistackbot_bringup/config/<robot>_controllers.yaml`; unknown robots fail fast with the available list). Current robots: `piper` (6-DoF arm + gripper), `xarm7` (7-DoF arm, vendor: UFACTORY — see `arms/xarm7/README.md` + `arms/xarm7/ik_decision_card.md`).

## Architecture

Three layers orchestrated by `unistackbot_bringup`. `unistackbot_description` is横切层 (cross-cutting), consumed by all layers via URDF/TF:

```
unistackbot_bringup            → 顶层启动 / 参数编排
unistackbot_controller         → 控制器集成 (CM 笛卡尔流式 + JointStream 关节流式) + 工具节点
unistackbot_algorithm          → 纯算法库 (FK/IK/种子库, 零控制器依赖)
unistackbot_hardware           → 硬件抽象 / 驱动通信     (hardware_interface, pluginlib, serial)
  ├─ unistackbot_sim_control  → 统一仿真控制层（kinematic 后端 + /sim_control 服务）
  ├─ unistackbot_gazebo        → Gazebo 集成（world + launch）
  └─ unistackbot_mujoco        → MuJoCo 集成（mujoco_ros2_control 链 launch; MJCF 资产住 description）
物理硬件 / 仿真器
```

Morphology differences are isolated to: `description` model files, `hardware`/`sim_control` backend plugins, and `controller`/`algorithm` kinematics/balance algorithms. Upper-layer logic and the bringup framework stay shared.

### Package responsibilities

- **unistackbot_bringup** — top-level launch + controller config. `control.launch.py robot:=<机型>` starts `robot_state_publisher`, a standalone `ros2_control_node`, and spawners for `joint_state_broadcaster` + `joint_stream_controller` (+ CM inactive); `robot` is required and no robot name is hardcoded anywhere in the launch. Controller config is per-robot: `config/<robot>_controllers.yaml` (update_rate 500 Hz; JointStream position command interface + RT 线程官方参数, 见上), shared by both the standalone node and Gazebo mode. `scripts/demo_motion.py` 是 JTC 时代遗留（还引用 follow_joint_trajectory action），**当前链路上不可用**——与 `test/verify_robot.sh`、`ik_demo_node` 同列已知待适配欠账。
- **unistackbot_controller** — 控制器集成 + ROS 工具节点；算法库住 `unistackbot_algorithm`（2026-09-17 拆分），本包 `<depend>unistackbot_algorithm</depend>` 只消费类型。两个自研控制器:
  - **CartesianMotionController**（P1.5, 2026-09-18 双链验收）: IK 上环宿主, `cartesian_motion_controller/` 自包含文件夹。契约: `~/target`(PoseStamped, reliable+KeepLast1, base系, 值通道) / `~/control`(`unistackbot_interface/CartesianControl`, TRACKING/HOLD, transient_local 事件通道) / `~/status`(`unistackbot_interface/CartesianMotionStatus`, 误差+min_sigma+结果码+stream_stale, 20Hz+INACTIVE 门闩); 关节反馈=/joint_states 不新增。三防线: 流式 500µs 预算(实测 max 11µs) / worker 冷启动出环(SpLatest seq 对账回灌; worker 参数 `worker_cpu`/`worker_nice`) / 安全层(限位+步长饱和, URDF max_velocity 单一事实源)。双机型 mock+gz E2E 绿, WCET p99 4.8µs。已知边界: 折叠零位远目标需预摆位或 worker 冷启动; 加速度界挂真机前。**换臂/换求解器的验证 SOP 见 `docs/ik_validation_playbook.md`**。OTG 门另立项。 `~/target` 断流受控减速同款（0c; **默认关**——`--once` 单发目标是合法用法, 流式跟踪场景 yaml 开 `stale_timeout_ms`; 断流时跳过 IK 省预算+关节速度线性刹停+status.stream_stale 置位）。 Scope update (2026-09-17): 本包承载通用基础设施控制器（OTG ingest gate 等安全链成员 + 插件模板）**和算法工程**——自研算法控制器落地本包。Full boundary analysis: `docs/hardware_framework_design.md` §14.
  - **JointStreamController**（2026-09-18）: 关节级 topic 流式控制器, 取代 JTC action 层（topic 性能优, 点流取代 goal/结果握手）。`joint_stream_controller/` 自包含文件夹。契约: `~/command`(`unistackbot_interface/JointCommand`, reliable+KeepLast(1); CSP/CSV/CST/MIT 四模式, 仿真链只消费 CSP)。插值档位参数 `interpolation: hold | ruckig`（hold=**每周期**步长饱和逼近已采纳目标——消息只采纳目标、步进按控制周期, 与消息率解耦; ruckig=OtgStream 点流整形, 慢上层 10-100Hz 率失配填充, 路径插值永不在控制器——形状轴归上层/MoveIt）。安全层与 CM 同款。**断流受控减速**（0c, 2026-09-21）: `stale_timeout_ms`/`stale_decel_ms`（链上 yaml 默认 200/200）——StaleWatch 判 `~/command` 断流后 hold 档=关节速度线性衰减刹停、ruckig 档=OtgStream 刹停, 恢复自动续接。rosidl msg 非 POD 不进 SpLatest——经 CmdSnapshot 翻译层（先例）。gz E2E: 50Hz 命令流跟踪误差 0.0053 rad, 与 CM 切换共存。
  - 工具节点: `fk_tool` / `ik_tool`（算法调试 CLI, `tools/`）+ `ik_demo_node`（RViz 交互拖动; 还走 JTC, 待改发 CM `~/target`）。
- **unistackbot_algorithm** — 纯算法库（零控制器依赖）, 2026-09-17 从 controller 拆出: `urdf_fk`（kdl_parser 建链 + FK/雅可比/限位/链序名, RT 零分配; `test/check_fk_tf.sh` TF 对拍实测 ~4e-13）+ `dls_ik`（Eigen SVD 阻尼伪逆 + boxed DLS 避限位 + SolveMode 流式/冷启动解耦 + 四分支代表种子 + `timeout_ns` 墙钟预算 + `min_sigma` 奇异遥测 + `NEAR_SINGULAR` 失败分类; 600 样本 Oracle 冷启动 99.5%）+ 可选**种子库**（`loadSeedLibrary` 读机型资产 `arms/<robot>/ik/seed_lib_<robot>.txt`, COLD_START 阶梯级0.5 分支封顶 top-6; 预算档 1ms 96.2% / 2ms 96.9% / 5ms 98.3%; 生成器 `test/gen_seed_library.py` + 增量 `test/seed_lib_incremental.py`, xarm7 冻结 v1.1=12k 条）。
- **unistackbot_description** — URDF/Xacro, RViz config, `display.launch.py`.
- **unistackbot_hardware** — real-driver plugin skeleton (empty `placeholder.cpp`); deps (`hardware_interface`, `pluginlib`) and the `device`/`baudrate`/`loop_rate` params in `piper_ros2_control.xacro` anticipate the real Piper CAN driver.
- **unistackbot_sim_control** — 统一仿真控制层（ 对 ros2_control 提供统一插件接口，对内按后端分类）:
  - `SimControlHardware` (`SystemInterface`): URDF `<hardware>` 里固定写 `<plugin>unistackbot_sim_control/SimControlHardware</plugin>` + `<param name="backend">kinematic</param>`。接口按 URDF 声明镜像导出（effort 恒 0）；关节动态数量 ≤16，限位/`max_velocity`/mimic 全部来自 `<ros2_control>` 的 `<param>`。
  - 后端 `kinematic`（`BackendKinematic`）: 理想执行器 —— 限位 clamp + 每关节 `max_velocity` 饱和的一阶逼近；mimic 关节按 multiplier/offset 从源关节推导。
  - **write()/read() 最终防线**（2026-09-20, 阶段0a; **真机驱动同型照抄**）: read 侧状态有限性门(非有限保持上一拍); write 侧不论上层控制器是谁, 下发前逐一过门 NaN → 限位 clamp → 步长饱和, 全程故障计数器(fault_cmd_nonfinite/clamped 等) + 限流 WARN。激活时 guard arm 建基准, 重激活重新 arm。分层语义: 控制器层"拒绝", write 层"clamp"兜底。测试床: 仓库根 `test/fault_injection.sh` + `fault_inject_bed.py`（F1 断流/F2 NaN/F3 限位/F5 超速/F6 断流受控减速 五场景全绿; 2026-09-21 F6 顺带实锤修复两处潜伏 bug: ①Humble `get_update_rate()` configure 期无效→hz 改由首 16 拍 period 中位数校准（两控制器同修, step_limits 一直在按 hz=1 装配被 write 层掩盖）②JointStream hold 档步进从"每消息"改为"每周期"）。
  - `/sim_control/*` 服务（reset / set_joint_state / pause / resume / step）: 插件进程内自建（`on_configure` 启动、`on_cleanup` 销毁），命令经 SPSC 无锁队列交给实时循环。**`/sim_control` 契约住本包**（`include/unistackbot_sim_control/sim_control_contract.hpp` 的 SimCommand/SimCmdType/SimControlServer; 2026-09-17 终局: 实现归实现的家、契约归契约的主人, 曾短暂上收 interface 后回迁）; 应答 `success=true` 只代表命令已被接受（入队/ign 请求已发出），不代表执行完成。注意：控制器激活时其保持命令每周期都会覆盖瞬移，**set_joint_state/reset 需在 pause 下使用**。
  - 日志走 ulog 宏（`ULOG_INFO`/`ULOG_ERROR`，非 RCLCPP）: `on_init` 里 `ulog_init` —— 终端 sink 恒开（launch 捕获 stdout），URDF `<ros2_control>` 块加 `<param name="ulog_file">` 可选开文件 sink；`on_shutdown` 里 `ulog_shutdown` 排空落盘。
  - gz 链路的 `/sim_control/*` 由 `unistackbot_gazebo` 包里的 `sim_control_gz_node` 承载（gz 知识归集成层; 契约头从本包 include——依赖方向 gazebo → sim_control）。
  - 回归测试: 仓库根 `test/verify_robot.sh <robot> [--with-gazebo]`（**JTC 残余待适配**: 脚本还断言 joint_trajectory_controller active + follow_joint_trajectory goal, 当前链路上会失败）、`test/smoke_sim_control.sh` 深测 `/sim_control` 服务语义（piper 关节表，断言含 mimic/限位拒绝/reset）。两者都自包含（需先 colcon build + source，脚本自行定位工作区并套用 `~/cyclonedds.xml`）。
- **unistackbot_interface** — 公共接口定义包（ROS msg/srv/action + 纯 C++ 共享契约头）：跨包/跨仓库共享的类型放在这里（单一事实源，供算法团队外部仓库依赖）。已落地：① 机器人级契约: `RobotFeedback`/`RobotCommand`（含冗余偏好与 MIT 模式; 与 `JointCommand.msg` 字段对齐——单一契约两载体）、`IkResult`/`SimResult` 分层结果码、`joint_capacity.hpp` 的 kMaxJoints 唯一定义; ② 消息: `JointCommand.msg`（CSP/CSV/CST/MIT 四模式点流）、`CartesianControl.msg`（TRACKING/HOLD 事件通道）、`CartesianMotionStatus.msg`; ③ `test_contract.cpp` 契约自检。**`/sim_control` 契约不住本包**——住 `unistackbot_sim_control`。
- **unistackbot_common** — 组件库，**不是 ROS 包**（无 package.xml/CMakeLists，colcon 自动忽略）：纯代码存放层，保持可在非 ROS 环境（RT 主站线程/单元测试/ARM 交叉编译）中直接复用。现有组件（每个独立子文件夹 = 文档 + 实现 + 测试三件套）:
  - `sp_latest/` — 双缓冲覆盖写/取最新原语（最新 **1** 个，"值通道"，seqlock 宣告式，seq 位宽自适应 64/32 位）；sim_control `threaded` 后端三通道 + CM worker 回灌 + JointStream 命令通道用它
  - `stale_watch/` — 陈旧看门狗（IDLE/LIVE/STALE 三态, 周期计数制零 syscall; 配 SpLatest::seq() 轮询判"上游还活着吗"; 0c: JointStream/CM 断流受控减速的判定原语, "无流=待命≠断流"）
  - `sp_ring/` — SPSC 无锁环形队列（**逐条必达**，"事件通道"，满拒新 push 语义）；sim_control 的 `sim_command_queue.hpp` 引用 `SpscRing`，域类型留在 sim_control
  - `mpsc_ring/` — 多写单读覆盖式无锁环（**丢旧保新**：日志缓冲/滑动窗口/音视频环; TSAN 零竞争）
  - `ulog/` — 高性能异步日志组件（前端宏：级别过滤→snprintf 定长 POD→无锁入队；后端单线程双 sink 终端+文件、error 强刷、10MB×5 轮转; emit 零 malloc）
  - `ruckig/` — vendored [pantor/ruckig](https://github.com/pantor/ruckig) v0.14 + `otg_stream.hpp` 防御封装（社区版已知 -101/-110/-111 数值风险的防线; `update()` 只有 Ok/Hold 两结局, Hold 输出恒有效; RT 补丁一处: trajectory.hpp SetIntegrate 模板化消除堆分配, 稳态零 malloc）。**不是单头文件**: `src/` 13 个 .cpp 需参与编译（controller 的 CMake GLOB 直引）。
  - `rt_tune/` — `RtTune` RT 线程调优参数化应用（亲和性/调度策略只能作用于调用线程自身, 仅供**自有线程**用; 控制器管理器主线程归官方参数, 二者分工勿混）。测试/基准 `bench_rt_tune.cpp`: 双线程 FIFO80 钉核1/2 周期唤醒自统计 (p50/p99/max+阈值计数) + apply 断言; 三档参考数据见 README（2026-09-20, p99 稳定 8-11µs）。
  - sim_control 与 controller 的 CMake 都以 `$<BUILD_INTERFACE:...>/../unistackbot_common` 直引组件库整个目录（monorepo 内，未 install——对外发布前需调整）
- **unistackbot_gazebo** — Gazebo integration, single chain: `ign.launch.py` (Gazebo Sim/Fortress via `ros_gz_sim` + `empty_ign.world` + bridges + `sim_control_gz_node`). The Gazebo Classic chain was removed 2026-09-17 (EOL; recover from git history if ever needed). Takes a required `robot:=<机型>`; no robot name is hardcoded. The controller manager lives inside the sim's ros2_control plugin — no standalone `ros2_control_node`. Constraints baked into the launch, each fixes a hard failure observed on dev machines:
  - URDF is re-serialized to a **single line** before use: the plugin forwards it to the CM as a `--param robot_description:=<urdf>` rule and rcl's parser rejects newlines → CM never starts.
  - The sim gets the URDF via a temp file (`-file` for spawn_entity / `create`), not the `/robot_description` topic: TRANSIENT_LOCAL latched re-delivery is unreliable under iceoryx/SHM CycloneDDS configs.
  - DDS comes from the machine-wide `~/cyclonedds.xml` (`CYCLONEDDS_URI` in `.bashrc`): binds `lo` + unicast `Peers 127.0.0.1`. This machine's `lo` lacks the MULTICAST flag, so unicast-only discovery intermittently dropped late joiners; the Peers bootstrap fixed it. The launches deliberately do NOT override `CYCLONEDDS_URI`. Run at most ONE launch stack at a time — leftover same-name nodes (robot_state_publisher / controller_manager) poison new runs.
  - The ign chain prepends the description package's ament share root to `IGN_GAZEBO_RESOURCE_PATH`: URDF→SDF conversion rewrites `package://` to `model://`, and Fortress resolves those only via that env var.
  - `scripts/gz_clean.sh` (`ros2 run unistackbot_gazebo gz_clean.sh`) kills the whole launch process family — ign servers routinely survive launch shutdown and poison the next run; run it before every launch.
  - `src/sim_control_gz_node.cpp` — `/sim_control/*` 的 gz 适配器（**纯 ROS 构建，零 ign 编译依赖**）。gz 链路的 ros2_control 插件是 `GazeboSimSystem` 而非 SimControlHardware，故 `/sim_control/*` 由这个 side-car 独立节点承载：pause/resume/step 经 launch 里 parameter_bridge 桥接的 `/world/<world>/control`（`ros_gz_interfaces/srv/ControlWorld`）下发；`reset` 拒绝（Fortress 实测有毒：返回 success=true 但世界 negative-timestep 停摆）、`set_joint_state` 拒绝（无原生等价，Garden+ 才有）。契约头从 `unistackbot_sim_control` include。由 `ign.launch.py` 启动，`world` 参数（`unistack_world`）须与 `empty_ign.world` 的 `<world name>` 及桥接服务名一致。
- **unistackbot_mujoco** — MuJoCo 集成 (第三条控制链, 2026-09-20): `mujoco.launch.py` 纯 launch 胶水零 C++。控制器管理器宿主 = `mujoco_ros2_control` 的**定制 `ros2_control_node`**（controller_manager + MuJoCo 引擎 + 物理线程 + 渲染同进程; 上游 ros2_control 合入仿真 PR 后可换回标准节点——临时措施）。真实动力学回归链, 对照 mock (理想执行器)/gz (ODE)。要点:
  - 安装: apt 二进制 `ros-humble-mujoco-ros2-control` 0.1.2 (内嵌 MuJoCo 3.12.0); RT 线程官方参数 (500Hz/FIFO80/核1) 实测生效, 与 mock 链同份 `<robot>_controllers.yaml`。
  - robot_description 参数直供（单行化, 同 gz 链坑）; `use_sim_time: true`; MJCF 缺失/未知机型 fail-fast。
  - headless 默认 true (URDF 硬件参数; apt 0.1.2 无 `MUJOCO_HEADLESS` 环境变量支持——PR #157 未随发布)。GUI 收场有 GL 析构段错误 (demo 同款, 控制器已干净关闭, 无害)。
  - MJCF 资产住 description 包 `arms/<robot>/mujoco/`（URDF→MJCF 用包自带 `robot_description_to_mjcf` 转换 + 手工策展; mesh 直引 collision STL 零复制; **equality 方向坑**: MuJoCo 3.x 实测 `joint1 = poly(joint2)`, 官方文档文字相反——详见 `arms/piper/mujoco/README.md`）。
  - `/sim_control` 适配器未接: mujoco 定制节点自带 `reset_world`/`set_pause`/`step_simulation`/`set_free_joint_state` 四服务（映射表在 `unistackbot_mujoco/README.md`）, 另有 gz 链没有的 `apply_external_wrench`。无孤儿进程问题（普通进程收场, 不需要 gz_clean 类脚本）。

## RT 基准与调优（仓库根 test/）

- `test/rt_chain_bench.sh <标签>` — 一键 RT 基准套件: cyclictest 三档(无负载/50%/90%, `cpu_load.py` 造载) + SMI/中断采样 + hwlatdetect + RT 带宽记录 + CM 链路三档 E2E(WCET/误差/收敛)。结果 markdown 入库 `test/results/`（当前权威基线 `rt_baseline_isolcpus.md`, 2026-09-20 隔离全套生效后: 负载档 Max 6-12µs; 历史基线同目录）。内核背景: 本机是**低延迟内核 (lowlatency), 不是 PREEMPT_RT**——措辞勿混。cmdline 已加 `isolcpus=domain,managed_irq,1,2 nohz_full=1,2 rcu_nocbs=1,2 irqaffinity=0,3-27`（隔离核1/2）。
- RT 裁决 (2026-09-20, 勿回退): `sched_rt_runtime_us` **保持默认 950000**——"500ms 事件 = RT throttling" 归因已翻案 (SCHED_OTHER 负载不进 RT 带宽账本, RT 占空比 <1% vs 95% 门槛; 详见基线文档翻案段); 950ms 默认值是同核 RT 疯转时的保险丝, 不持久化 -1。
- isolcpus 坑: cyclictest 直接 `-a 1,2` 会 FATAL（不突破继承亲和 mask）——必须 `taskset -c 1,2 cyclictest ...`（套件已内置）。
- `test/rt_tune_boot.sh [0-3]` — **每次开机手动执行** `sudo bash test/rt_tune_boot.sh` (默认 = 核0-3 前四核): idle 按退出延迟>2µs 禁深睡 (x86 通用, 兼 ACPI/intel_idle 档表) + performance governor + EPP; uncore 锁频段默认关 (对照实验无收益)。幂等, sysfs 直写零依赖; 换机器改脚本内 DEFAULT_CPUS。生产环境再转 systemd oneshot。
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

Robot models live under `unistackbot_description/arms/<robot>/urdf/` with mesh assets under `arms/<robot>/meshes/{visual,collision}/`. Mesh paths in URDF use the `package://unistackbot_description/...` URI — keep this prefix when adding new robots so resolved paths work after `--symlink-install`.

The **Piper arm** is the reference model and the pattern to follow for new robots:
- `piper.urdf.xacro` — top-level entry; declares the xacro args above and conditionally includes the macros below.
- `piper_macro.xacro` — `piper_arm` macro defining `base_link` → `link6` with a `parent` param. With `use_world:=true` a `world` link is added and the arm is fixed to it; otherwise `base_link` is the root (what Gazebo spawns).
- `piper_gripper_macro.xacro` — `piper_gripper` macro (flange + gripper_base + prismatic `gripper` joint + two mimic fingers). Keep an `<inertial>` on the otherwise empty `gripper_link` — Gazebo drops inertia-less links, which would remove the `gripper` joint the controller config depends on.
- `piper_ros2_control.xacro` — `<ros2_control>` block with the plugin switch described above; joints declare a `position` command interface and `min`/`max`/`max_velocity` limit params the kinematic backend enforces.
- `piper.urdf` and `piper_with_gripper.xacro` — flat (non-xacro-macro) variants kept alongside the macro versions.

`display.launch.py` uses `OpaqueFunction` to defer xacro processing so launch args can be passed into the `xacro` command substitution. It selects `joint_state_publisher_gui` vs `joint_state_publisher` via mutually exclusive `IfCondition`/`UnlessCondition` on the `gui` arg.

## docs/ — 设计文档与课程笔记（中文）

Not build input, but two items are normative for code:

- `cpp_style_guide.md` — **repo-wide C++ style, binding**（全工程适用）; the Conventions section below defers to it.
- `hardware_framework_design.md` — master working doc for the real-hardware layer (`unistackbot_hardware` + RT core): 分层架构、线程模型、协议后端契约（master.hpp 五要素、形态盲对象模型）、v3 语义状态机、形态与算法边界（§14）、与 ros2_control 生态的对照审核（§15）. Status: 讨论中、未收口 (2026-09).
- `socketcan_master_design.md` / `ethercat_master_design.md` — full designs of the two bus-master backends (SocketCAN CAN FD; IgH ecrt, 1kHz + DC), structurally parallel, both written against the master.hpp contract.
- `linux_rt_guide.md` / `rt_software_architecture.md` — RT system tuning (PREEMPT_RT, isolcpus, IRQ affinity, cyclictest/抖动排查) and the RT software architecture manifesto（七支柱）. Normative for how RT threads and cross-thread data paths are written.
- `timesync_design.md` — MCU↔主机时间同步两帧协议（主机为时间服务器, MCU 落 sync.csv 锚点表; v1.9 USB 转串口部署版）。
- `udp_internal_bus_argument.md` — 内部总线选型论证（插值器以下链路不采用 UDP 的文献证据版, 服务于 CAN FD 定版决策）。
- `ik_validation_playbook.md` — 换臂/换 IK 求解器的六阶段验证方法论 + 参数审计表 + 验收模板。
- `control_course/` — control-theory course notes（传递函数 → 三环级联、PM/带宽账）. Background for the numbers cited in the design docs; not code documentation.

## Conventions to preserve

- Math/engineering rule: **for mathematically mature operations (linear algebra, SVD/pseudo-inverse, optimization, polynomial root-finding), always prefer a mature library (Eigen, KDL) over hand-rolling; self-written code is reserved for strategy and composition** (e.g. our IK null-space objective, seed ladder). Numerical layer = libraries; policy layer = ours.

C++ style is governed by `docs/cpp_style_guide.md`. Non-negotiables: Allman braces on their own line everywhere; **Tab indentation** (display width 4), never spaces; filename = snake_case of the main class (`sim_control_hardware.hpp` ↔ `SimControlHardware`); `PKG__FILE_HPP_` include guards; `k`-prefixed constexpr constants, `snake_case_` members; **explicit error flow** — our code never throws (errors via return value + message, `[[nodiscard]]` on error-returning functions); system calls that are documented to throw (`std::stod` etc.) are caught at the call boundary and translated into return values — exceptions never enter the control chain (style guide rule 27). All C++ packages compile with `-Wall -Wextra -Wpedantic` and `cxx_std_17`, zero warnings. RT hot paths: lock-free structures/atomics only — no mutex, malloc, printf, or IO in the control loop (RT code discipline lives in `docs/linux_rt_guide.md` part 2, not the style guide).
- Each package uses `ament_lint_auto` with `ament_lint_common` for tests — match this when adding new packages.
- `bringup`'s `package.xml` lists the packages it orchestrates as `exec_depend` (including `unistackbot_sim_control`); `unistackbot_gazebo` similarly declares its runtime deps. Add new runtime-consumed packages there.
- Keep `.gitkeep` files in empty `launch/`, `config/`, `include/` directories so the package structure survives in git.

# 仿真验证流水线（机型接入 → 验收归档）

> 一条流水线完成一台机型的全部仿真验收，产出一份可归档报告。**可复用资产**：新机型只带资产
> 进流程（URDF/MJCF/登记表），流程零改动——复用性验收标准见 §5。
> 配套文档：测试项定义见 `sim_environment_and_test_plan.md` §5.7；接口字段查
> `chain_interface_dictionary.md`；故障定位查 `sim_environment_and_test_plan.md` §8；
> 换臂/换求解器 SOP 与球腕解析解机型接入流水线查 `guides/ik_validation_playbook.md`（§9）。

## 1. 流程总览

链次序 = 互补漏斗：**mock 验逻辑 → mujoco 验物理 → gz 验集成**。

| 阶段 | 内容 | 工具 | 验收门 | 产物 |
|---|---|---|---|---|
| A 模型接入 | 四态展开 + 形态指纹 + 可视化目检 | verify §1、display | 指纹=登记；目检无异常 | 静态记录 |
| B mock 逻辑 | 三件套 + JS 流 + CM 切换 + 防线 F1-F6 + /sim_control 语义 | verify §3、fault、smoke | 全绿 | 日志 |
| C mujoco 物理 | 动力学跟踪 + 防线在动力学下 + F7 mimic | verify 段、fault --chain mujoco | 全绿 | 日志 |
| D gz 集成 | 跟踪 + 桥语义 + pause/resume 端到端 | verify --with-gazebo 段 | 全绿 | 日志 |
| E RT 基准 | cyclictest + CM 链路 WCET/误差 | rt_chain_bench | 对比基线无回归 | results/rt_*.md |
| F 数据场景 | 三命令域 demo 流（关节慢/关节满速/末端）+ 终点/负路径 + 录包 + 曲线人检 | demo_motion + demo_cartesian、bag | 曲线无异常 | bag |
| G 归档 | 汇总填写验收报告 | 报告模板 | 报告生成 | acceptance_*.md |

## 2. 手动执行手册（当前标准走法；每步门不过即停）

### 2.1 环境前置（开机一次）

```bash
sudo bash test/rt_tune_boot.sh
ip link show lo | grep -q MULTICAST && echo "环境 OK"   # 无则: sudo ip link set lo multicast on
```

### 2.2 链路验收（A 静态 + B + C + D，一条命令）

```bash
bash test/verify_robot.sh piper --with-gazebo --with-mujoco
```

门：`结果: PASS=N FAIL=0`（piper 现行 53 项，含形态指纹②③断言）。

### 2.3 可视化目检（A 补充，人眼）

```bash
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro
```

门：mesh 无丢失、装配无错位、拖动各关节顺滑。提示：§2.4 的 mock+RViz 会话可兼活体目检。

### 2.4 防线验收（B/C）——自动 bed 为准，现场版建立直觉

**自动路径（数字账，进报告）**：

```bash
bash test/fault_injection.sh --robot piper
bash test/fault_injection.sh --robot piper --chain mujoco
```

- `--robot piper|xarm7` 手动选机型（阈值账按机型，关节集按 机型×链 自动装配）
- 门：逐场景 PASS（mock F1-F6；mujoco + F7 mimic，仅 piper）。**gz 链不跑 = 设计使然**（防线在我们插件里）

**现场版（RViz 手动复现）**——起链，等两行 "Configured and activated"：

```bash
ros2 launch unistackbot_bringup control.launch.py robot:=piper use_rviz:=true
```

命令模板（9 关节列全；mode:1=CSP；手指值=±0.5×gripper；**每个终端先 source 工作区**）：

```bash
ros2 topic pub --once /joint_stream_controller/command unistackbot_interface/msg/JointCommand \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, gripper, gripper_joint1, gripper_joint2], mode: 1, position: [P1, P2, P3, P4, P5, P6, G, G1, G2], velocity: [0,0,0,0,0,0,0,0,0], effort: [0,0,0,0,0,0,0,0,0]}"
```

| 场景 | position 注入值 | RViz 预期现象 |
|---|---|---|
| F1 断流保持 | `[1.0, 0.5, -1.0, 0.3, -0.2, 0.4, 0.03, 0.015, -0.015]`，到位后静置 10s+ | 纹丝不动 |
| F3 限位外 | 首值改 `3.0`（>joint1 限位 2.618） | 精确停在 2.618，不越界 |
| F2 NaN | 九个值全改 `.nan` | 零位移；后续合法命令仍响应 |
| F5 超速 | 先回全零，再发首值 `2.5` | 匀速段存在，耗时 ≈ 0.5s（精确判定靠录包） |
| F6 断流受控减速 | 模板改 `-r 50`、首值 `2.4`，臂动后 Ctrl-C 发流终端 | **不到 2.4**——减速滑行后停住 |
| F7 mimic | 末三值 `[0.08, 0.04, -0.04]` | 手指同步 = ±0.5×gripper |

辅助核验（JSB 乱序，按名看）：`ros2 topic echo /joint_states --once`。
F2 的 `.nan` 依赖 CLI YAML 解析，报解析错误则以自动 bed 为准。收场：Ctrl-C launch，残留按定位树清。

### 2.5 服务语义（B）——自动 smoke 为准，手动验关键三条

```bash
bash test/smoke_sim_control.sh        # 自动: 十项断言 (自起链, 先收掉其他链)
```

手动三条（链活着时，现场理解"为什么必须 pause"）：

```bash
ros2 service call /sim_control/pause std_srvs/srv/Trigger
ros2 service call /sim_control/set_joint_state unistackbot_interface/srv/SetJointState \
    "{name: [joint1], position: [0.5]}"    # pause 下瞬移生效
ros2 service call /sim_control/resume std_srvs/srv/Trigger
# 反向: 不 pause 直接瞬移 → 被激活控制器每拍拉回
```

### 2.6 RT 基准（E，约 30 分钟；先收掉其他链）

```bash
bash test/rt_chain_bench.sh piper_first_0922 --robot piper
bash test/rt_chain_bench.sh piper_first_0922 --robot piper --chain mujoco
```

门：mock p99 7.5µs / max 70µs；mujoco max 508µs = 预算剪枝签名。超线即停，按定位树查。
（脚本自带分步终端进度；CM 目标从 ee_state 当前位姿动态生成，机型无关。）

### 2.7 数据与场景（F，三链 × 双空间录制——G4 一致性门的数据来源）

顺序执行，一次一套链，收链后再换下一链。**每链 2+1 演示**（三命令域覆盖, 设计 §16.2 透传原则）：

- **演示 1 关节空间（两种率域）**：`demo_motion.py` → JS 点流（路径点从 URDF 限位推算，
  同机型三链同命令流）。默认 `--hz 50` = **关节慢流域**（率失配 → JS **ruckig 档** C2 填充，
  链上默认；A/B 实测同轨迹 dv/拍 平滑 ~100×）；加跑 `demo_joint_fullrate`（C++）=
  **关节满速域**（自校准 = update_rate; ruckig 对微小目标步 ≈ 直通——python `--hz 500`
  只是名义值故满速档用 C++）。
  满速档要点：rclcpp wall 定时器下发；频率自校准用**消息 stamp 差分中位数**（到达时刻差会被
  轮询量化偏差低读，实测 480.5 假象 vs stamp 校准 500.0 精确）；结束时自报名义/实际频率
  （mock 实测 99.9% 达成率）——**判读：达成率 ≥99% 为过，显著偏低查链负载/DDS**
- **演示 2 笛卡尔空间**：`demo_cartesian.py --pattern sweep` → 切 CM → 抬升+前伸部署 → r=span
  圆周整圈 → 上浮/回心 → 收回落零 → 切回（base/tip 读自 CM 自身参数，机型无关）。三 pattern 按用途选：
  - `sweep`（大幅，**录制默认**）：部署 20cm + r=10cm 圆周 16 点连续 + 收回，~12s，EE 总程 ~1.6m——
    曲线信息量最大；参数 `--lift/--fwd/--span/--speed`，腿时长=距离/speed，发布率 `--stream-hz`
    （与 CM 500Hz 控制解耦，只定切分粒度）
  - `axes`（多点位测试）：抬升基准 + 8 向 2cm 逐点回基准——可达性覆盖 + 诚实拒绝路径；
    需要可达性数据时可加跑
  - `up`（最小演示）：+z 往返 3cm，`--traverse` 插值流（=0 单发点到点）
  - **点位设计纪律**：大幅运动必须先"部署"——piper 探针实测零位邻域仅 +z 可行，部署位
    (z+200,x+200) 邻域 ±150mm 球 6/6 全可解；换机型先跑工作空间探针再定 `--lift/--fwd/--span` 默认
- **演示 3 终点+负路径（§16.6 改版, 2026-09-24）**：OTG 门已退役（架构并入 CM 输出级）。
  终点语义 = CM 原生能力（`--once` 单发即完整合法用法, 内部一次 IK + OtgStream C2）;
  负路径 = `--pattern reject` 不可达弹幕（远超臂展/穿底等）逐点断言诚实拒
  （UNREACHABLE/LIMIT_CONFLICT 两类结果码都要有镜头）+ 恢复目标必须照常收敛
  （拒绝不污染链路状态; 上层不完美是常态, 拒绝路径也是验收面）。判读：诚实拒 N/N、
  恢复收敛 err ≈ 账地板、单发终点到位 err ≈ 账地板、命令流频率 ≈ update_rate

先起录包再跑演示（慢流/满速/末端/OTG 门四路命令流都进包——`/joint_stream_controller/command`
是关节慢流+满速+OTG 门共用入口，`/cartesian_motion_controller/target` 是末端流）；**仿真链必须带
/clock**（跨链按仿真时间对齐的前提）。

**链 1/3 mock（本阶段必录）**——按序执行，每步等上一步收尾：

```bash
# ⓪ 残留检查 (2026-09-23 实锤: 三条链的 robot_state_publisher 同活 → demo 读到别链
#    的 URDF 变体 → 关节表错 → JS 因"未列全命令关节"静默丢消息 → demo 发流臂不动。
#    supervisor_node 也要查——2026-09-23 实锤 24 个孤儿积累整天无人发现, 因残留检查
#    模式里没它; 它本身 SIGINT 正常退出, 孤儿全来自旁路 launch 的收场方式)
pgrep -af "robot_state_publisher|ros2_control_node|supervisor_node" | grep -v grep   # 必须=空
#    清理用 [x] 括号防 pgrep -f 自匹配 (实锤: 模式含自身命令行会把清理命令自己杀死, exit 144):
for p in $(pgrep -f "[s]upervisor_node|[r]os2_control_node|[r]obot_state_publisher"); do kill $p; done
ros2 daemon stop                                                     # 清 DDS 缓存的陈旧参与者

# ① 起链 —— spawner 竞态防御 (2026-09-23 定稿, 官方依据 ros2_control issue #2071):
#    launch 已收拢为 2 个 spawner 进程(激活组列表 + inactive 组, 官方 example_13 模式)
#    + --service-call-timeout 30 + 延迟 2s。等 3 行 "Configured and activated" (~4-6s)。
#    若仍见 "Failed to configure" = 机器极慢, 等 5s 手动补拉: ros2 run controller_manager
#    spawner joint_state_broadcaster joint_stream_controller ee_state_broadcaster
ros2 launch unistackbot_bringup control.launch.py robot:=piper use_rviz:=true

# ② 起录包 (四路命令流+状态全进包; 先于一切演示, 时间基准)
ros2 bag record -s mcap /joint_states /joint_stream_controller/command \
    /cartesian_motion_controller/status /cartesian_motion_controller/target /tf \
    -o /tmp/piper_first_mock

# ③ 演示 1a 关节慢流 (50Hz 点流, ~16s) —— 等脚本退出 (断流减速收尾) 再下一条;
#    结束行自报名义/实际达成率 (rclpy 低频域校准旁证)
ros2 run unistackbot_demo demo_motion.py

# ④ 演示 1b 关节满速 (C++ 透传域, 6s) —— 结束行自报达成率, ≥99% 为过
ros2 run unistackbot_demo demo_joint_fullrate --ros-args -p duration:=6.0

# ⑤ 演示 2 末端位姿流 (sweep, ~12s) —— 等脚本退出 (臂已落回起点)
ros2 run unistackbot_demo demo_cartesian.py --pattern sweep

# ⑥ 演示 3 终点+负路径 (§16.6 改版 2026-09-24: OTG 门退役, 终点语义=CM 原生能力——
#    每条消息=最新终点, --once 单发即完整合法用法; 脚本自动切 CM, 结束切回 JS)
#    终点冒烟可选: ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped \
#      "{header: {frame_id: base_link}, pose: {position: {x: 0.056, y: 0.0, z: 0.413}, orientation: {w: 0.7373, x: 0.0, y: 0.6756, z: 0.0}}}"
ros2 run unistackbot_demo demo_cartesian.py --pattern reject

# ⑧ 收尾: Ctrl-C 停录包(终端 2) → Ctrl-C 收链(终端 1; 不用 pkill) → 等 ~30s 租约
#    → 下一条链前重跑 ⓪ (残留检查是每链 ⓪ 步, 不是只有第一条链做)
```

**链 2/3 mujoco**——步骤同 mock，差异：headless（录制不渲染）+ 带 /clock：

```bash
# ① 起链 (headless=录制链不渲染防污染; 若复用默认 GUI 先显式关) —— 等 3 行 "Configured and activated"
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper headless:=true

# ② 起录包 (mujoco 必带 /clock —— 跨链按仿真时间对齐)
ros2 bag record -s mcap /joint_states /joint_stream_controller/command \
    /cartesian_motion_controller/status /cartesian_motion_controller/target /tf /clock \
    -o /tmp/piper_first_mujoco

# ③ 演示 1a 关节慢流 —— 等脚本退出再下一条
ros2 run unistackbot_demo demo_motion.py

# ④ 演示 1b 关节满速 —— 达成率 ≥99% 为过
ros2 run unistackbot_demo demo_joint_fullrate --ros-args -p duration:=6.0

# ⑤ 演示 2 末端位姿流 (sweep) —— 物理链稳态 0.7-1.1mm, 骑 1mm 线的腿判"稳态未达容差"属预期
ros2 run unistackbot_demo demo_cartesian.py --pattern sweep

# ⑥ 演示 3 终点+负路径 (§16.6 改版 2026-09-24: OTG 门退役, 终点语义=CM 原生能力——
#    每条消息=最新终点, --once 单发即完整合法用法; 脚本自动切 CM, 结束切回 JS)
#    终点冒烟可选: ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped \
#      "{header: {frame_id: base_link}, pose: {position: {x: 0.056, y: 0.0, z: 0.413}, orientation: {w: 0.7373, x: 0.0, y: 0.6756, z: 0.0}}}"
ros2 run unistackbot_demo demo_cartesian.py --pattern reject

# ⑧ 收尾: 停录包 → 收链 → 等租约再起下一条
```

**链 3/3 gz**（gripper 不动 = 已知回归，曲线里属预期）——步骤同 mock，差异：先 gz_clean、gui 关：

```bash
# ⓪ 清残留 (单独执行! 它的 pkill 模式会误杀含 "ros2 launch" 字样的宿主命令行)
ros2 run unistackbot_gazebo gz_clean.sh

# ① 起链 (gui 关防渲染污染) —— 等 3 行 "Configured and activated"
ros2 launch unistackbot_gazebo ign.launch.py robot:=piper gui:=false

# ② 起录包 (带 /clock)
ros2 bag record -s mcap /joint_states /joint_stream_controller/command \
    /cartesian_motion_controller/status /cartesian_motion_controller/target /tf /clock \
    -o /tmp/piper_first_gz

# ③ 演示 1a 关节慢流 —— 等脚本退出再下一条
ros2 run unistackbot_demo demo_motion.py

# ④ 演示 1b 关节满速 —— 达成率 ≥99% 为过
ros2 run unistackbot_demo demo_joint_fullrate --ros-args -p duration:=6.0

# ⑤ 演示 2 末端位姿流 (sweep) —— 手指不耦合为已知回归, 曲线里属预期
ros2 run unistackbot_demo demo_cartesian.py --pattern sweep

# ⑥ 演示 3 终点+负路径 (§16.6 改版 2026-09-24: OTG 门退役, 终点语义=CM 原生能力——
#    每条消息=最新终点, --once 单发即完整合法用法; 脚本自动切 CM, 结束切回 JS)
#    终点冒烟可选: ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped \
#      "{header: {frame_id: base_link}, pose: {position: {x: 0.056, y: 0.0, z: 0.413}, orientation: {w: 0.7373, x: 0.0, y: 0.6756, z: 0.0}}}"
ros2 run unistackbot_demo demo_cartesian.py --pattern reject

# ⑧ 收尾: 停录包 → 收链 → **gz_clean 再清一次** (ign 服务器常驻留孤儿) → 等租约
```

**频率矩阵（选做，独立于上面 A-G 录制流程）**——控制端低频 {50,100,200} × 总线 {500,1000}：

```bash
# 一格一跑 (~40s: 清场→起链→录包→demo→指标→结果行自动追加 rate_matrix_<机型>_<链>.md)
bash test/rate_matrix_bench.sh <标签> --robot piper --chain mock --bus-hz 500 --cmd-hz 50
#    --chain mujoco 同款; --domain cartesian 可选; gz 链不参与 (bus_hz 忽略, 三链分工)
```


**症状速查**：demo 发流臂不动 → 先查 ⓪ 残留（demo 打印的关节表带不带 `+ mimic [...]` 是
快速指纹：mock=piper 9 命令关节带 mimic 尾，mujoco=7 无——与链型不符即分裂脑）；再查
`ros2 control list_controllers`（JS active?）与 `/joint_states` 是否有新帧。

门（每链 2+1 演示）：曲线人检——演示 1 正弦平滑无抖动（慢流=demo_motion ruckig 填充、满速=
demo_joint_fullrate 透传，同路径两档对比）；演示 2
看到"部署 → 圆周连续扫描 → 收回落零"全程流式平滑，status 误差单调收敛无震荡；演示 3
OTG 门加减速平滑（对照恒速段）、到位 ≈ 账地板、命令流 ≈ update_rate。**判读三条**：① 解析解稳态
误差地板 = 归一化账（piper ≈0.089mm，预期非 bug，DLS 无此账）；② 诚实拒绝（UNREACHABLE=1/
LIMIT_CONFLICT=4）是解析解的正确行为——脚本记"拒"跳过并在汇总行给出 N/M（零位邻域 ±x/±y 拒绝
属预期; OTG 门对不可达终点 WARN 拒绝保持）；③ 收敛只认发布后的新 status（脚本已内置清缓存 +
target_pose 对账，防在途旧帧假收敛）。收场不用 pkill，精确收链。

### 2.8 归档（G）

按模板 `docs/sim/acceptance_report_template.md` 手填，存
`test/results/acceptance_piper_<日期>.md`，连同 E 阶段基线文档一起入库。

## 3. 引擎化（后置：手动流程两台机型走通后再做）

```bash
bash test/run_acceptance.sh <robot> [--with-rt]
```

- 编排 §2.1-2.8，复用现有工具退出码与输出，**零验证逻辑**；任一门挂即停并定位到阶段
- 报告自动填充 `acceptance_report_template.md` 同构内容 → `results/acceptance_<robot>_<日期>.md`
- 退出码 0/非 0（CI-ready）

## 4. 工具需求（由流程位置倒推）

| 优先 | 工具 | 服务的阶段 | 状态 |
|---|---|---|---|
| 1 | run_acceptance.sh 流程引擎 + 报告 | G | 待开发（§3） |
| 2 | 跨链对比器（同命令流三链 diff） | F | 待开发 |
| 3 | 扰动注入 CLI（wrench 脉冲/阶跃） | C 进阶 | 待开发 |
| 4 | /sim_control/diagnostics 计数器话题 | B/C 长跑 | 待开发 |
| 5 | control_recorder 录制节点 | F 完整版 | 缓建 |
| 6 | 即时故障注入 CLI（ad-hoc） | 流程外 | 待开发 |

## 5. 复用性验收（本资产的完成标准）

1. piper 首跑全绿，报告入库（流程自验证）
2. xarm7 第二跑零流程改动全绿（首次复用实测；**尚未跑过完整 A-G**）
3. 未来新机型：只备机型资产 + 登记表，一条命令出报告

**已知边界**：流程覆盖"链对命令流的响应"；算法在环由批次 6 消费者样例节点覆盖；
传感器闭环与多形态保持挂账；真机就绪归 G6（`sim_environment_and_test_plan.md` §5.5/§10）。
OTG 门（2026-09-23）暂为 §2.7 手动项——**尚未进 verify_robot 自动断言**（mujoco 链验证与
rt_chain_bench WCET 对比待补后收编）；OTG 门与 CartesianShaper 的设计依据见
`architecture/hardware_framework_design.md` §16。

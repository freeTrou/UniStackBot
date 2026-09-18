# unistackbot_controller

控制器集成包：**CM 控制器插件**（笛卡尔流式运动控制）+ ROS 工具节点。算法库（urdf_fk/dls_ik/种子库）住 `unistackbot_algorithm`（2026-09-17 拆分），控制器只消费类型。

## 能力（P1.3, 2026-09-17）

- `urdf_fk.hpp/cpp`（纯 C++ 库，无 roscpp）：`init(urdf, base, tip)` 显式错误流 → `fk(q)→CartesianPose` / `jacobian(q)→6×n 行主序` / 限位与链序关节名查询。建链用 kdl_parser（与 RSP 同源，TF 对拍机器精度一致）；限位自 urdfdom 提取。RT 安全：fk/jacobian 零堆分配（计数器实证）
- `fk_tool`（`ros2 run`）：FK 命令行工具，调试与 TF 对拍用
- `test_urdf_fk.cpp`（g++ 直编）：零位手算真值（strict，xarm7）+ 雅可比 vs 有限差分（<1e-11）+ FK 幂等 + RT 零分配

## 验证

```bash
bash unistackbot_algorithm/urdf_fk/test/run_urdf_fk_test.sh xarm7 link7 link_base strict   # 单测 37 断言
bash unistackbot_algorithm/urdf_fk/test/run_urdf_fk_test.sh piper link6 base_link          # 机型无关性
bash test/check_fk_tf.sh xarm7                                                      # sim 对拍: TF vs FK 10 位姿, 实测 ~4e-13
```

TF 对拍原理：robot_state_publisher 是独立实现（kdl_parser 建树 + 树序 FK），同 URDF 同 q 两套代码，任何坐标系级错误都逃不过——**这是 FK 正确性的工程裁判**。

## 数值 IK（P1.4, 2026-09-17）

- `dls_ik.hpp/cpp`：DLS（Eigen SVD 阻尼伪逆，λ 自适应）+ 零空间二级目标（限位中心吸引 + 边界斥力）+ 种子阶梯（流式种子 → mt19937 均匀重启 40 次）+ 分支粘性（重启解距种子 >1.5 rad 视为跳变拒绝输出）+ 失败不改输出。消费 `RobotCommand.redundancy`（LOCK_JOINT 精确锁定；ARM_ANGLE 偏置构型上诚实拒绝 `UNSUPPORTED`）
- `ik_tool`（`ros2 run`）：位姿 → 关节解 + FK 回代误差自证
- 验证（`test/run_ik_test.sh`）：真值对拍（`ik_oracle_xarm7.txt`，ssik 离线生成 + 自检指纹）+ 轨迹连续性（101 点圆弧全过，增量 0.063<0.15）+ 对抗表 + 端到端（IK 解→sim 瞬移→TF 实测偏差 0.0）

**C 项收官（2026-09-17）**：冷启动 99.5%（600 样本 v2 Oracle，关节正推全姿态）。关键修复链：SolveMode 接口解耦（STREAMING 粘性 1.5 / COLD_START 无粘性+四分支代表种子）、boxed DLS（迭代 clamp 进线搜索，2/90→59/90）、Wampler λ、收敛邻域信任步。完整方法论与换臂 SOP 见 `docs/ik_validation_playbook.md`。迭代内 clamp 与线搜索组合、收敛后限位检查（绝不 clamp 伪装——FK 回代断言当场抓住）。

## CartesianMotionController（P1.5, 2026-09-18 双链验收）

IK 算法上环的宿主：`cartesian_motion_controller/` 自包含文件夹（hpp+cpp+plugin xml）。

**对外契约（冻结）**：
- `~/target` `geometry_msgs/PoseStamped`（值通道，**reliable+KeepLast(1)**，base 系，入口四元数归一化）
- `~/control` `unistackbot_interface/CartesianControl`（事件通道：TRACKING/HOLD，transient_local）
- `~/status` `unistackbot_interface/msg/CartesianMotionStatus`（位姿误差/收敛/min_sigma/结果码，20Hz + INACTIVE 门闩）
- 关节反馈不新增：`/joint_states`（JSB）是关节空间单一事实源

**三条防线**：① update() 内流式 IK 墙钟预算（500µs，实测 max 11µs）② 连续失败 3 拍 → worker 低优线程 COLD_START（自建 UrdfFk+DlsIk 实例——KDL 不跨线程，种子库线程内加载即预热），结果经 SpLatest seq 对账回灌 ③ 安全层独立于 IK：NaN 门 → 限位 clamp → 步长饱和（URDF max_velocity/update_rate 单一事实源）。失败处置 = 逐拍事实码 + NEAR_SINGULAR 停烧预算（give_up，新目标自动重启）；无降级状态机（对标 MoveIt Servo，策略归编排层/OTG 门）。

**激活预热**：四档日志（cached_tid 首syscall）+ 可达/不可达假解 + RealtimePublisher 首拍——首触成本全留非 RT 阶段。

**验收数据**（双机型双链）：mock/gz E2E 收敛 5e-8；WCET p50 1.5µs / p99 4.8µs / max 87µs（预算 2000µs）；隔离测量 update 路径 12s **零缺页**；双模式长程 11/12（唯一失败=启动期 action 未就绪，竞态非控制）。

**已知边界**：折叠零位直发远目标是流式域边界（σ≈0.002 深奇异）——JTC 预摆位后一切正常，或等 worker 冷启动（piper 无种子库时阶梯可能不足）；加速度界（一行）挂真机前清单（Ruckig 已选型）；策略（锁存/联动/看门狗动作）挂编排层账。

## 工具节点

- `fk_tool` / `ik_tool`：算法调试 CLI
- `ik_demo_node`：RViz 拖动演示（走 JTC；CM 就绪后可改发 `~/target`）

## 范围裁决（2026-09-17, 取代 2026-09-09 裁决）

本包承载**通用基础设施 + 算法工程**：

- OTG ingest gate —— 外部命令的入口安全链成员（真机接入前落地）
- 基于 URDF 的 FK/IK（已落地：`urdf_fk` + `dls_ik` + 种子库）
- 算法控制器（自研算法工程落地本包：如 Cartesian IK 运动控制器、后续运动学/控制算法）
- 算法控制器的插件模板

> 2026-09-09 的"只放基础设施、形态算法永不进本包、保持空壳"裁决**废止**。
> 双通道接入（外部进程 ingress+OTG / CM 插件）语义不变，仅对外部团队算法成立。

完整边界分析见 `docs/hardware_framework_design.md` §14（§14.4 有本次修订记录）。

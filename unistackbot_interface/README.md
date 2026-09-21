# unistackbot_interface

公共接口定义包：跨包 / 跨仓库共享类型的**单一事实源**（ROS msg/srv/action + 纯 C++ 共享契约头），供算法团队的外部仓库直接依赖。

## 现状

**第二批契约已落地（2026-09-17）**——反馈/指令帧 + 分层结果码（命名与分层均经用户裁决）：

- `robot_feedback.hpp`：`RobotFeedback` 反馈帧——关节三态 + **模式/状态两个正交维度**（`RobotMode`：NONE/CSP/CSV/CST/MIT，0x6061 语义；`DeviceState`：OFFLINE/DISABLED/READY/OPERATIONAL/FAULT，statusword 的形态盲抽象——**故障是状态不是模式**）+ `fault_mask` 轴位掩码。设备"错误"住反馈帧。`seq` 写次数递增 / `stamp_s` steady_clock 域
- `robot_command.hpp`：`RobotCommand` 指令帧——命令模式词汇对齐 **CiA402 周期同步族 + 行业 MIT**（`NONE`/`CSP`/`CSV`/`CST`/`MIT`，默认 NONE=零初始化即安全；**MIT 模式**=Mini Cheetah 执行器命令格式（电机行业通用名，控制理论即关节阻抗式）：τ=kp·e+kd·ė+τ_ff，协作二期柔顺与 RL policy 输出的标准形态，ki 因 windup 风险不留；PROFILE/IPM/HOMING 的不收录理由写在头文件边界声明）/掩码/`max_delta` 步长限幅 + `RedundancyPreference`（`PRESERVE`/`LOCK_JOINT<i>`/`ARM_ANGLE(ψ)`，冗余显式化）
- `ik_result.hpp` / `sim_result.hpp`：**分层结果码**（`IkResult` 4 码 / `SimResult` 5 码，各配 `*_result_message()` 单一事实源）。裁决原则：**错误跟着产生它的层走；设备状态跟着反馈流走；公共失败码取消**；总线层结果等 hardware 落地随需另立
- `joint_capacity.hpp`：`kMaxJoints` 唯一定义（原住 sim_control_contract，上收）
- 命名对照：设计文档语境 RobotStateSnapshot/JointCmd → 落地更名 RobotFeedback/RobotCommand

**第三批：控制器在线消息（2026-09-18）**：

- `JointCommand.msg`：关节点流命令（CSP/CSV/CST/MIT 四模式 + `joint_names` 对名；消费者 reliable+KeepLast(1)）
- `CartesianControl.msg`：CM 事件通道（TRACKING/HOLD）
- `CartesianMotionStatus.msg`：CM 状态遥测（误差/min_sigma/结果码/stream_stale，20Hz）
- 挂账：`EeState.msg` 待旋转表示决策后加入（加并行字段不破坏消费者）

测试（g++ 直编，不进 colcon）：`g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion -Iinclude test_contract.cpp -o /tmp/test_interface && /tmp/test_interface`

**`/sim_control` 契约曾居本包（2026-09-16），2026-09-17 回迁 `unistackbot_sim_control`**——契约随其主人（统一仿真控制层）与两个实现同居；本包回归**纯机器人级类型包**（结果码是类型故 `SimResult` 留下；`kMaxJoints` 因反馈/指令帧共用亦留）。实现归实现的家（gz 后端在 gazebo），类型归类型的家，契约归契约的家。

## 放什么

- `master.hpp` 的 `RobotStateSnapshot` / `JointCmd` 等共享契约头（P0.2 后续）
- RL ingress 命令 schema
- state 出口消息

判据：**两包以上（或跨仓库）要用的类型**才进来；包内私有类型留在各自包里。

## 挂账：帧容量运行时化（路径 1，已裁决暂缓，2026-09-17）

**目标**（用户需求）：帧尺寸 = init 时从配置读的关节数 N——多少关节定义多大，无任何写死容量，人形 43 直接上，运行时零分配。

**硬约束**（四选三）：帧尺寸运行时读 / RT 零分配 / 无锁组件（sp_latest、sp_ring，TSAN 认证资产）不动 / 代码无容量常数——不可兼得。已核实的组件事实：两者均有 `trivially_copyable` 硬断言；`sp_latest::read()` 每次拷贝构造临时量（vector 载荷 = 每读一次分配，RT 不可接受）。

**已裁决的实施方案（路径 1，暂缓执行）**：
- 契约层：`RobotFeedback`/`RobotCommand` 数组 → `std::vector` + `init(n)`（等长赋值零分配——标准保证源 size ≤ 目标 capacity 时复用缓冲；用全局 new 计数器测试实证）；删除 `joint_capacity.hpp`
- 交换层：sim_control 内部 `SimCommand`/`PlantSnapshot` 保持 POD，容量常数包内化为 `kSimWireJoints`（注释含人形日"改数+重编"升级路径）；两个无锁组件零改动
- 连带：sim_control 对 interface 的依赖（仅为 kMaxJoints）消失——依赖图简化为 gazebo → sim_control，interface 完全独立
- 工作量 ~2 小时，零 TSAN 风险

**触发条件**（满足其一时重启）：① P1.3/P1.4 的真实交换形态落地（消费者出现，契约改动免费窗口关闭前）；② 人形/高关节构型立项；③ 跨进程共享内存交换成为需求（届时连带解决）。

**现状**：`kMaxJoints = 16`（interface/joint_capacity.hpp，与 SimControlHardware URDF 上界 ≤16 一致）；契约帧为定长 POD + `joint_count` 运行时字段；内部缓冲已是 init 恰好 resize 的等长 vector。

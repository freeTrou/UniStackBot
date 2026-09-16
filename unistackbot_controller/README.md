# unistackbot_controller

运动控制层：**形态盲运动学库**（URDF → KDL 链 → FK + 雅可比）+ 算法控制器骨架。库不含任何机型知识——机型只是 URDF 数据。

## 能力（P1.3, 2026-09-17）

- `urdf_fk.hpp/cpp`（纯 C++ 库，无 roscpp）：`init(urdf, base, tip)` 显式错误流 → `fk(q)→CartesianPose` / `jacobian(q)→6×n 行主序` / 限位与链序关节名查询。建链用 kdl_parser（与 RSP 同源，TF 对拍机器精度一致）；限位自 urdfdom 提取。RT 安全：fk/jacobian 零堆分配（计数器实证）
- `fk_tool`（`ros2 run`）：FK 命令行工具，调试与 TF 对拍用
- `test_urdf_fk.cpp`（g++ 直编）：零位手算真值（strict，xarm7）+ 雅可比 vs 有限差分（<1e-11）+ FK 幂等 + RT 零分配

## 验证

```bash
bash unistackbot_controller/test/run_urdf_fk_test.sh xarm7 link7 link_base strict   # 单测 37 断言
bash unistackbot_controller/test/run_urdf_fk_test.sh piper link6 base_link          # 机型无关性
bash test/check_fk_tf.sh xarm7                                                      # sim 对拍: TF vs FK 10 位姿, 实测 ~4e-13
```

TF 对拍原理：robot_state_publisher 是独立实现（kdl_parser 建树 + 树序 FK），同 URDF 同 q 两套代码，任何坐标系级错误都逃不过——**这是 FK 正确性的工程裁判**。

## 数值 IK（P1.4, 2026-09-17）

- `dls_ik.hpp/cpp`：DLS（Eigen SVD 阻尼伪逆，λ 自适应）+ 零空间二级目标（限位中心吸引 + 边界斥力）+ 种子阶梯（流式种子 → mt19937 均匀重启 40 次）+ 分支粘性（重启解距种子 >1.5 rad 视为跳变拒绝输出）+ 失败不改输出。消费 `RobotCommand.redundancy`（LOCK_JOINT 精确锁定；ARM_ANGLE 偏置构型上诚实拒绝 `UNSUPPORTED`）
- `ik_tool`（`ros2 run`）：位姿 → 关节解 + FK 回代误差自证
- 验证（`test/run_ik_test.sh`）：真值对拍（`ik_oracle_xarm7.txt`，ssik 离线生成 + 自检指纹）+ 轨迹连续性（101 点圆弧全过，增量 0.063<0.15）+ 对抗表 + 端到端（IK 解→sim 瞬移→TF 实测偏差 0.0）

**已知限制（挂账调参）**：冷启动成功率强依赖种子构型（流式上一解 101/101≈100%；中心种子对"朝下姿态"可达带仅 ~50%）——TRAC-IK 的 SQP 是业界对此的答案，我们的对应提升路径=停滞检测+重启策略调参 / 限位 SQP，挂账。迭代内不 clamp（边界踏步实测更差），收敛后限位检查（越界=该种子失败，绝不 clamp 伪装——clamp 伪装被 FK 回代断言当场抓住，2026-09-17）。

## 算法控制器骨架（待 P1.4 后实例化）

状态接口→`RobotFeedback`、`RobotCommand`→命令接口的薄适配骨架；OTG 摄入门（安全链）另立项。


运动控制 / 运动学解算层（rclcpp、geometry_msgs、nav_msgs、tf2）。

## 现状

**空骨架**（`src/placeholder.cpp`），尚未开始实现。

## 范围裁决（2026-09-09）

本包**只放通用基础设施控制器**：

- OTG ingest gate —— 外部命令的入口安全链成员
- 基于 URDF 的 FK
- 算法控制器的插件模板

形态相关算法（步态 / WBC / ZMP / 轮式运动学）**不进本包**，由算法团队的独立仓库承载，经两条一等通道接入：

1. 外部进程：ingress + OTG
2. CM 插件：框架提供模板

完整边界分析见 `docs/hardware_framework_design.md` §14。

# unistackbot_algorithm

纯算法库（2026-09-17 自 `unistackbot_controller` 拆出，范围裁决见该包 README）：**形态盲运动学解算 + 数值 IK**。**核心库零 ROS / 零 nlopt**（无 rclcpp/controller_interface），外部仓库（算法团队/RL 训练侧/ARM 交叉编译）只依赖核心库即可复用；TRAC-IK 是可选子件（方案 C，2026-10-08），见下。

## 内容（目录 = 每算法一个文件夹，后续新算法各起一个，互不交叉）

| 文件夹 | 说明 | 测试 |
|---|---|---|
| `urdf_fk/` | URDF → KDL 链 → FK + 雅可比 + 限位 + `chain()` 只读访问；实例不可跨线程共享（见头文件契约）| `bash urdf_fk/test/run_urdf_fk_test.sh <robot> <tip> <base> [strict]`（拓扑必填无默认）；TF 对拍 `../test/check_fk_tf.sh` |
| `dls_ik/` | 数值 IK：boxed DLS + 种子阶梯 + 墙钟预算 + 奇异遥测 + 可选种子库；继承 `IkSolver` 接口（RT-capable：零 malloc 可进任意线程；数值解默认已让位 trac_ik，本类=RT 场景/离线调试 fallback）| `bash dls_ik/test/run_ik_test.sh <robot>` |
| `ik_solver/` | **IkSolver 抽象接口**（CM 可插拔求解层，5 硬契约在 hpp，契约 4=调用域纪律两类实现自声明）。新求解器 = 实现接口 + yaml `ik_solver:` 参数 + playbook 六阶段 | 由实现方测试覆盖 |
| `trac_ik_solver/` | **TracIk 适配层 + vendored TRAC-IK**（traclabs rolling，BSD-3，回调线程专用——内部起线程+分配，禁入 RT 环）。编入独立可选子库 `unistackbot_trac_ik`（rclcpp+nlopt 只挂子库），`UNISTACKBOT_BUILD_TRAC_IK=OFF` 可关（交叉编译核心库时） | AB 工具账见 `trac_ik_solver.hpp` 头注 |
| `base_kinematics/` | **底盘运动学可插拔层**（body twist ↔ 执行通道闭式双向变换，urdf_fk 同族纯函数；消费者=BaseTwistController；参考实现 diff 已入，n_steer/pass_through 按底盘域阶段 B/D 进） | `g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I.. base_kinematics.cpp test_base_kinematics.cpp -o /tmp/t_bk && /tmp/t_bk`（base_kinematics/ 目录下，纯 std 零依赖） |
| `dls_ik/test/` 工具链 | 种子库生成/增量 + Oracle 基准；库资产在 `unistackbot_description/arms/<robot>/ik/` | — |

验证方法论 SOP（六阶段 + 参数审计 + 换臂流程）：`docs/guides/ik_validation_playbook.md`。

## 纪律

- 依赖单向：`unistackbot_algorithm → unistackbot_interface`；**永不依赖** controller/controller_manager 生态
- **核心库零 ROS / 零 nlopt**（方案 C，2026-10-08，交叉编译考量）：rclcpp+nlopt 只进可选子库 `unistackbot_trac_ik`（vendored 上游特例，用户裁决不剥离上游代码；`UNISTACKBOT_BUILD_TRAC_IK=OFF` 时核心库纯 Eigen+KDL，ARM/RT 侧随便交叉编——OFF 时勿编 controller）
- 测试为 g++ 直编（不进 colcon test），见各脚本头注释；与 `unistackbot_common` 组件同型

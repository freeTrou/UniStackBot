# unistackbot_algorithm

纯算法库（2026-09-17 自 `unistackbot_controller` 拆出，范围裁决见该包 README）：**形态盲运动学解算 + 数值 IK**。**核心库零 ROS / 零 nlopt**（无 rclcpp/controller_interface），外部仓库（算法团队/RL 训练侧/ARM 交叉编译）只依赖核心库即可复用；TRAC-IK 与 pluginlib 注册库是可选子件（方案 C，2026-10-08 / 缝③，2026-10-22），见下。

## 内容（目录 = 每算法一个文件夹，后续新算法各起一个，互不交叉）

| 文件夹 | 说明 | 测试 |
|---|---|---|
| `urdf_fk/` | URDF → KDL 链 → FK + 雅可比 + 限位 + `chain()` 只读访问；实例不可跨线程共享（见头文件契约）| `bash urdf_fk/test/run_urdf_fk_test.sh <robot> <tip> <base> [strict]`（拓扑必填无默认）；TF 对拍 `../test/check_fk_tf.sh` |
| `dls_ik/` | 数值 IK：boxed DLS + 种子阶梯 + 墙钟预算 + 奇异遥测 + 可选种子库；继承 `IkSolver` 接口（RT-capable：零 malloc 可进任意线程；数值解默认已让位 trac_ik，本类=RT 场景/离线调试 fallback）| `bash dls_ik/test/run_ik_test.sh <robot>` |
| `analytic_piper/` | piper 球腕 6 轴闭式解析解（IKFast 生成物 vendored，Apache-2.0；`check_spherical_wrist` 结构指纹选拒；再生管线 `gen/`） | `bash analytic_piper/test/run_wrist_check_test.sh` |
| `analytic_stationlite/` | stationlite 三平行轴（Pieper 形式二）闭式解析解（IKFast；指纹 j2∥j3∥j4+正交对选拒；B 线 2026-10-21） | `bash analytic_stationlite/test/run_stationlite_ik_test.sh` |
| `ik_solver/` | **IkSolver 抽象接口**（CM 可插拔求解层，5 硬契约在 hpp，契约 4=调用域纪律两类实现自声明；**对外插件缝**——虚签名变更=破坏性变更）。新求解器 = 实现接口 + 插件注册（自包 `PLUGINLIB_EXPORT_CLASS` + plugin.xml + `pluginlib_export_plugin_description_file`，外部交付零仓库改动）+ playbook 六阶段 | 由实现方测试覆盖 |
| `analytic_common/` | **闭式解析解适配层公共件**（头内联 detail 层：周期折叠+去重+限位过滤+距种子最近；2026-10-26 审核修 F10 抽取——piper/stationlite 双消费者已现，此前逐字复制已漂移） | 随两适配层黄金测试覆盖 |
| `ik_plugins/` | **IkSolver pluginlib 注册库**（缝③ 2026-10-22，ROS-thin 可选子库：4 条 `PLUGINLIB_EXPORT_CLASS`，yaml `ik_solver` 值 = xml class name 裸短名，五机型 yaml 零改动；住 trac_ik 开关块内——按构造依赖 trac_ik） | 装载路径由 CM 活链回归覆盖（`verify_robot.sh`） |
| `trac_ik_solver/` | **TracIk 适配层 + vendored TRAC-IK**（traclabs rolling，BSD-3，回调线程专用——内部起线程+分配，禁入 RT 环）。编入独立可选子库 `unistackbot_trac_ik`（rclcpp+nlopt 只挂子库），`UNISTACKBOT_BUILD_TRAC_IK=OFF` 可关（交叉编译核心库时） | AB 工具账见 `trac_ik_solver.hpp` 头注 |
| `base_kinematics/` | **底盘运动学可插拔层**（body twist ↔ 执行通道闭式双向变换，urdf_fk 同族纯函数；消费者=BaseTwistController；参考实现 diff 已入，n_steer/pass_through 按底盘域阶段 B/D 进） | `g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I.. base_kinematics.cpp test_base_kinematics.cpp -o /tmp/t_bk && /tmp/t_bk`（base_kinematics/ 目录下，纯 std 零依赖） |
| `dls_ik/test/` 工具链 | 种子库生成/增量 + Oracle 基准；库资产在 `unistackbot_description/arms/<robot>/ik/` | — |

验证方法论 SOP（六阶段 + 参数审计 + 换臂流程）：`docs/guides/ik_validation_playbook.md`。

## 纪律

- 依赖单向：`unistackbot_algorithm → unistackbot_interface`；**永不依赖** controller/controller_manager 生态
- **核心库零 ROS / 零 nlopt**（方案 C，2026-10-08，交叉编译考量）：rclcpp+nlopt 只进可选子库 `unistackbot_trac_ik` 与 `ik_solvers` 注册库（vendored 上游特例，用户裁决不剥离上游代码；`UNISTACKBOT_BUILD_TRAC_IK=OFF` 时核心库纯 Eigen+KDL，ARM/RT 侧随便交叉编——注册库同块不构建，controller 本体可编：CM 已 pluginlib 化不直链 trac_ik 头，2026-10-22）
- 测试为 g++ 直编（不进 colcon test），见各脚本头注释；与 `unistackbot_common` 组件同型

# unistackbot_algorithm

纯算法库（2026-09-17 自 `unistackbot_controller` 拆出，范围裁决见该包 README）：**形态盲运动学解算 + 数值 IK**。零控制器生态依赖（无 rclcpp/controller_interface），外部仓库（算法团队/RL 训练侧）只依赖本包即可复用。

## 内容（目录 = 每算法一个文件夹，后续新算法各起一个，互不交叉）

| 文件夹 | 说明 | 测试 |
|---|---|---|
| `urdf_fk/` | URDF → KDL 链 → FK + 雅可比 + 限位；实例不可跨线程共享（见头文件契约）| `bash urdf_fk/test/run_urdf_fk_test.sh <robot> <tip> <base> [strict]`（拓扑必填无默认）；TF 对拍 `../test/check_fk_tf.sh` |
| `dls_ik/` | 数值 IK：boxed DLS + 种子阶梯 + 墙钟预算 + 奇异遥测 + 可选种子库；继承 `IkSolver` 接口（CM 默认实现）| `bash dls_ik/test/run_ik_test.sh <robot>` |
| `ik_solver/` | **IkSolver 抽象接口**（CM 可插拔求解层，5 硬契约在 hpp）。新求解器 = 实现接口 + yaml `ik_solver:` 参数 + playbook 六阶段 | 由实现方测试覆盖 |
| `dls_ik/test/` 工具链 | 种子库生成/增量 + Oracle 基准；库资产在 `unistackbot_description/arms/<robot>/ik/` | — |

验证方法论 SOP（六阶段 + 参数审计 + 换臂流程）：`docs/guides/ik_validation_playbook.md`。

## 纪律

- 依赖单向：`unistackbot_algorithm → unistackbot_interface`；**永不依赖** controller/controller_manager 生态
- 测试为 g++ 直编（不进 colcon test），见各脚本头注释；与 `unistackbot_common` 组件同型

# unistackbot_algorithm

纯算法库（2026-09-17 自 `unistackbot_controller` 拆出，范围裁决见该包 README）：**形态盲运动学解算 + 数值 IK**。零控制器生态依赖（无 rclcpp/controller_interface），外部仓库（算法团队/RL 训练侧）只依赖本包即可复用。

## 内容（目录 = 每算法一个文件夹，后续新算法各起一个，互不交叉）

| 文件夹 | 说明 | 测试 |
|---|---|---|
| `urdf_fk/` | URDF → KDL 链 → FK + 雅可比 + 限位（TF 对拍 ~4e-13）；实例**不可跨线程共享**（KDL 求解器内部暂存，见头文件用法契约）| `bash urdf_fk/test/run_urdf_fk_test.sh <robot> [tip] [base] [strict]`；链级 TF 对拍 `../test/check_fk_tf.sh` |
| `dls_ik/` | 数值 IK：boxed DLS + σ 自适应阻尼 + 种子阶梯（调用方种子 → 库种子 → 四分支 → 随机）+ `timeout_ns` 墙钟预算 + `min_sigma` 遥测 + `NEAR_SINGULAR` 失败分类 + 可选种子库（`loadSeedLibrary`）| `bash dls_ik/test/run_ik_test.sh <robot>`（六层 6000+ 断言）|
| `dls_ik/test/` 工具链 | `gen_seed_library.py`（种子库生成）/ `seed_lib_incremental.py`（覆盖驱动增量）/ `gen_ik_oracle.py`（真值基准）；库资产在 `unistackbot_description/arms/<robot>/ik/` | — |

验证方法论 SOP（六阶段 + 参数审计 + 换臂流程）：`docs/ik_validation_playbook.md`。

## 纪律

- 依赖单向：`unistackbot_algorithm → unistackbot_interface`；**永不依赖** controller/controller_manager 生态
- 测试为 g++ 直编（不进 colcon test），见各脚本头注释；与 `unistackbot_common` 组件同型

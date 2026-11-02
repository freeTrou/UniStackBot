# unistackbot_algorithm_solvers — 算法参考实现 + 插件注册（Pkg 2）

> **定位**（2026-11-01 两包拆分定稿）：算法域参考实现之家。**同仓外部化**——归属臂算法组维护，
> 框架组当租户对待；依赖只指 SDK（`unistackbot_algorithm`）+ 自选库，框架包对本包零改动。
> SDK 契约、接入三步指南、归属终表：`docs/architecture/algorithm_architecture.md`（normative）。
> 算法团队最小开发闭包 = **`unistackbot_algorithm` 一个包**（本包是"我们的参考实现"，不是他们的必需品）。

## 内容地图

| 目录 | 内容 | 注册插件名 | 测试 |
|---|---|---|---|
| `ik_plugins/` | pluginlib 注册面（IK + WB + trac_ik 可选块） | — | — |
| `dls_ik/` | DLS 数值 IK（RT-capable：零 malloc/锁，可进任何线程） | `dls` | `test/run_ik_test.sh [robot]` |
| `analytic_common/` | IKFast 系适配层公共件 | — | （随两机型测试覆盖） |
| `analytic_piper/` | piper 球腕 6 轴闭式解（IKFast 生成 + 结构指纹选拒） | `analytic_piper` | `test/run_wrist_check_test.sh` |
| `analytic_stationlite/` | stationlite 平行轴闭式解（IKFast + 指纹选拒 + 周期折叠） | `analytic_stationlite` | `test/run_stationlite_ik_test.sh` |
| `trac_ik_solver/` | TRAC-IK 适配层（vendored，**回调线程专用**——契约 4b：内部起线程+分配，禁入 RT 环） | `trac_ik` | `test/run_trac_ik_test.sh [robot]` |
| `whole_body/` | 全身/多臂解算：`single_arm_dls`（单臂示例）/ `dualarm_dls`（双臂占位）/ `upper_body_hqp_mos`（mos HQP 适配，见其 README） | `dualarm_dls` | `whole_body/README.md`（实例验证路径） |
| `base_kinematics/` | 底盘运动学（diff + n_steer；BaseVelocityController 编译期消费，工厂自注册） | — | `test_base_kinematics.cpp`（g++ 直编） |

## 依赖与构建

```
依赖: unistackbot_algorithm (SDK) + unistackbot_common (仅 ulog) + Eigen/KDL/urdfdom/kdl_parser/pluginlib
可选: trac_ik 块需 rclcpp + nlopt —— UNISTACKBOT_BUILD_TRAC_IK=ON (默认) / OFF 时本包其余照编
```

```bash
cd UniStackBot_ws && colcon build --symlink-install --packages-select unistackbot_algorithm_solvers
```

选型账（详见各机型 `ik_decision_card.md` 与 CLAUDE.md）：`analytic_*` = 闭式正解；`trac_ik` = 数值默认
（Distance 档保流式连续性）；`dls` = RT 场景/离线 fallback。per-robot yaml 显式声明、无默认值（fail-fast 纪律）。

## 测试（g++ 直编，零 colcon）

| 脚本 | 前置 | 期望 |
|---|---|---|
| `dls_ik/test/run_ik_test.sh [robot]` | `/tmp/verify_<robot>.urdf`（verify_robot.sh 展开）+ oracle | 冷启动高成功率 + 流式步长账（xarm7 参考值见脚本头） |
| `analytic_piper/test/run_wrist_check_test.sh` | 工作区已 source | 指纹正反例 + FK-IK 回代黄金，全部通过 |
| `analytic_stationlite/test/run_stationlite_ik_test.sh` | 工作区已 source | 2026 cases 全过 |
| `trac_ik_solver/test/run_trac_ik_test.sh [robot]` | 同 dls_ik 前置 | AB 分层账全 PASS |

oracle 与种子库可再生：`test/gen_ik_oracle.py` / `test/gen_seed_library.py` / `test/seed_lib_incremental.py`。

## IKFast 再生成管线（analytic_*/gen/）

`gen/generate_in_container.sh`（podman + OpenRAVE）吃 `*_canonical.urdf/dae` 产 IKFast 生成物——
**输出不进 git**（`gen/.gitignore` 挡 `generated/`），正本经手工适配后入库为 `*_ikfast.cpp`。
规范化工序与坑见 `analytic_piper/gen/README.md`；stationlite 为后立范式（gen 目录零输出物）。

## vendor 许可证账

| vendor | 许可 | 来源与 patch |
|---|---|---|
| `trac_ik_solver/vendor/trac_ik/` | BSD-3 | traclabs rolling 快照 + Humble 兼容 patch（urdf::Model→urdfdom parseURDF）；`kdl_tl.cpp`/`dual_quaternion.h`/`math3d.h` 为传递依赖补入 |
| `whole_body/vendor/mos_wbc/` | Lumos | mos WbcSolver 接口头快照，唯一 patch = include 改同目录相对（详见 `whole_body/README.md`） |

## 加一个新实现（三步）

1. 建目录写实现（对照 `dls_ik/` 或 `whole_body/single_arm_dls.hpp` 为范本；失败不改 out、墙钟预算、调用域自声明——契约 5 条硬规则见 SDK `ik_solver.hpp`）
2. `ik_plugins/` 加注册三件：EXPORT_CLASS + plugin.xml + CMake 一行
3. 机型 yaml 改一行选用（`ik_solver:` / `solver:`），启动日志回显激活解

外部团队同款路径（兄弟包或独立 clone 均可）：`docs/architecture/algorithm_architecture.md` §4。

## 维护纪律

- 实现契约在 SDK 头里，本包**永不反向定义**契约词汇（IkResult/Redundancy 已迁 SDK，2026-11-02）
- g++ 直编脚本的路径解析基 = 包根与 SDK 兄弟包相对路径——**两包拆分时曾整批断掉**（2026-11-02 修复），动目录结构须同改脚本
- 求解器实例线程私有；RT-capable 与否必须在类注释自声明

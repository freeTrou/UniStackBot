# unistackbot_products/ —— 产品域（ODE §3.6 一仓两域）

> 2026-10-30 随 Phase 0A 立骨架（首次实使用）。**框架域（UniStackBot 各包）只进机制；
> 机型资产/兼容门面/出货技能住本域**——外部贡献者 PR 不触框架包的物理落点。

| 子域 | 内容 | 首件 |
|---|---|---|
| `mos_legacy/` | mos 合并线的机型与兼容件 | `mos_p20_description`（P20 URDF+meshes，Phase 0A） |
| `wheeled_family/` | 轮臂家族五变体（预留，`docs/architecture/wheeled_arm_family.md`） | — |
| `mos_compat/` | MosCompat 门面 + /sdk ROS 面节点（Phase 2 落） | — |

约定：机型包命名 `<machine>_description`；CODEOWNERS 随第一个外部维护者进来时补。

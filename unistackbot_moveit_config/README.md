# unistackbot_moveit_config · MoveIt2 接入配置

> M1 (2026-10-09) piper 试点落地。MoveIt = 任务级规划器租户: 规划避障自主运动,
> 执行走链上 JTC —— 框架控制器/契约零自研代码 (B 路线, 性能与语义双胜出:
> 执行环内闭环零附加跳 + JTC 原生 hold = 任务域主流语义)。

## 架构位置 (三命令域 → 四)

| 命令域 | 命令源 | 执行器 | 状态 |
|---|---|---|---|
| 流式关节 (遥操/键控/demo) | topic 点流 | JS (自研 ruckig) | 默认 active |
| 流式笛卡尔 (遥操/RL/VLA) | topic 点流 | CM (自研 OtgStream) | inactive 切换 |
| **任务级规划执行 (MoveIt)** | **action (整轨迹)** | **JTC (社区标准件)** | **inactive 切换** |

三执行器认领同一批臂关节接口, 任意时刻至多一个 active (switch_controllers 既有机制)。
两域断流语义分治 (2026-10-08 裁决): 流式域=受控减速刹停 (0c); 任务域=JTC 停在最后点 (主流)。

## 用法

```bash
# 终端1: 链 (JTC 已随链 inactive 注册)
ros2 launch unistackbot_bringup control.launch.py robot:=piper
# 终端2: move_group + RViz (MotionPlanning 面板拖目标交互)
ros2 launch unistackbot_moveit_config moveit_piper.launch.py
# 程序化 E2E (规划→切JTC→执行→切回; E2E 实测 j2 0.6 到位 0.6004):
ros2 run unistackbot_moveit_config demo_moveit_plan.py --j2 0.6
```

RViz: Fixed Frame=world; MotionPlanning display 拖末端球标设目标 → Plan。
交互执行需先切 JTC (JS↔JTC 互斥): 拖动规划用 Plan and Display 无需切换,
真执行用 demo 脚本或手动 switch (编排自动化属 M1-4 待做)。

## 文件

- `config/piper.srdf` — 规划组 arm(base_link→link6, CM 链同口径)+gripper; home 状态限位内缩
- `config/piper_kinematics.yaml` — KDL 求解器先行 (M2 换自研 TracIk 包装插件, IK 单一事实源)
- `config/piper_joint_limits.yaml` — velocity=URDF 源; acceleration=机型 a=5 同口径
- `config/piper_ompl_planning.yaml` — RRTConnect + **AddTimeOptimalParameterization 必备**
- `config/piper_moveit_controllers.yaml` — moveit_simple_controller_manager → JTC action
- `config/piper_moveit.rviz` — world 系 + RobotModel/PlanningScene/MotionPlanning
- `scripts/demo_moveit_plan.py` — 程序化规划执行 E2E

## Humble 踩坑记录 (2026-10-09)

1. **JTC type 必须由 spawner `-t` 传**: Humble CM 不认 yaml 段顶/参数内的 type
   (那是 Iron+ 行为); 且 `-t` 对多控制器 spawner 全局 → CM/JTC 必须拆两个 spawner 进程
2. **type 放段顶层直接炸链**: rcl 参数文件语法 "Cannot have a value before ros__parameters"
3. **AddTimeOptimalParameterization 是真名**: 旧文档的 AddTimeParameterization 在
   2.5.10 不存在且**静默拒载** → 轨迹时间戳全零 → JTC "时间不严格递增"拒收
4. URDF/SRDF 字符串参数须包 `ParameterValue(..., value_type=str)` (gz 链同款坑)

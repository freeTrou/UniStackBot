# unistackbot_interface

公共接口定义包：跨包 / 跨仓库共享类型的**单一事实源**（ROS msg/srv/action + 纯 C++ 共享契约头），供算法团队的外部仓库直接依赖。

## 现状

**首批契约已落地（2026-09-16）**——`/sim_control` 统一仿真控制契约：

- `srv/SetJointState.srv`（瞬移关节状态请求/应答）
- `include/unistackbot_interface/sim_control_contract.hpp`（纯 C++ 共享契约头，header-only）：`SimCommand`/`SimCmdType` 命令帧 + `kMaxJoints` 容量常量 + `SimControlServer` 服务门面（五个 `/sim_control/*` 服务的统一创建器，互斥组串行保证队列单生产者）

消费方：`unistackbot_sim_control`（mock 链插件，命令入队交 RT 循环）、`unistackbot_gazebo`（gz 适配器，翻译成 ign 世界服务）——**两个实现只依赖本包，互不依赖**。

消费方 CMake 接法（Humble 实测）：纯 C++ 契约头走 monorepo 直引（`$<BUILD_INTERFACE:...>/../unistackbot_interface/include`，与 unistackbot_common 同型，对外发布前需调整）+ 生成 srv 头经 `ament_target_dependencies(<target> unistackbot_interface)`。

## 放什么

- `master.hpp` 的 `RobotStateSnapshot` / `JointCmd` 等共享契约头（P0.2 后续）
- RL ingress 命令 schema
- state 出口消息

判据：**两包以上（或跨仓库）要用的类型**才进来；包内私有类型留在各自包里。

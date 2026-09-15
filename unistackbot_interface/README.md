# unistackbot_interface

公共接口定义包：跨包 / 跨仓库共享类型的**单一事实源**（ROS msg/srv/action + 纯 C++ 共享契约头），供算法团队的外部仓库直接依赖。

## 现状

**空骨架**：`msg/` `srv/` `action/` `include/` 目录已建，尚无任何类型落地。

## 放什么

- `master.hpp` 的 `RobotStateSnapshot` / `JointCmd` 等共享契约头
- RL ingress 命令 schema
- state 出口消息

判据：**两包以上（或跨仓库）要用的类型**才进来；包内私有类型留在各自包里。

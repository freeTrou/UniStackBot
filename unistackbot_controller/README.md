# unistackbot_controller

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

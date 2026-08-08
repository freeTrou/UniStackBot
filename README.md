# UniStackBot

基于 ROS 2 Humble 的通用移动机器人软件栈，按职责拆分为四个功能包，覆盖从硬件驱动到顶层启动的完整链路。

## 目录结构

```
unistackbot/
├── unistackbot_bringup/        # 顶层启动包
├── unistackbot_controller/     # 运动控制层
├── unistackbot_description/    # 机器人模型描述
├── unistackbot_hardware/       # 硬件抽象层
├── README.md
├── LICENSE
└── .gitignore
```

## 功能包说明

### unistackbot_bringup

顶层启动包，负责拉起整机的各个节点并加载运行参数。包含统一入口的 launch 文件和默认配置，便于一键启动完整系统或其中的子模块（如仅启动底盘、仅启动可视化等）。

### unistackbot_controller

运动控制层。实现底盘运动学解算（差速 / 麦克纳姆 / 全向轮等）、轨迹跟踪、速度平滑以及上层指令（`geometry_msgs/Twist`、`nav_msgs/Odometry` 等）到底层电机命令的转换。向上对接导航与规划，向下对接硬件接口。

### unistackbot_description

机器人模型描述包。存放 URDF/Xacro 文件、可视化配置（RViz）以及 `robot_state_publisher` / `joint_state_publisher` 的启动文件，用于描述机器人的连杆、关节、惯量与传感器外参，供仿真、可视化与 TF 树使用。

### unistackbot_hardware

硬件抽象层。负责与底层驱动（电机控制器、编码器、IMU 等）通信，向上以 `hardware_interface` 插件或自定义话题/服务形式提供统一接口，屏蔽具体硬件差异，使上层控制器与具体设备解耦。

## 构建

```bash
# 在 ROS 2 工作空间根目录
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
# 启动完整系统（占位示例，待 bringup 实现）
ros2 launch unistackbot_bringup unistackbot.launch.py
```

## 许可证

见 [LICENSE](LICENSE)。

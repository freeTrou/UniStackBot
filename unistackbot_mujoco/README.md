# unistackbot_mujoco — MuJoCo 集成层 (第三条控制链)

MuJoCo 仿真链: 真实动力学回归验证, 对照 mock 链 (理想执行器) 与 gz 链 (Gazebo Fortress)。
纯 launch 胶水零 C++ —— 控制器管理器宿主是 `mujoco_ros2_control` 的**定制 ros2_control_node**
(controller_manager + MuJoCo 引擎 + 物理线程 + 渲染同进程), 上游 ros2_control 合入仿真 PR
后可换回标准节点 (临时措施)。双线程架构 (CM RT 线程核1 + MuJoCo 物理线程) 与 SpLatest
通道语义在仿真下保持不变。

## 起链

```bash
ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper
# 可选: headless:=false 拉起 MuJoCo Simulate 渲染窗 (需 DISPLAY); use_rviz:=true
```

与 mock/gz 链同款三件套: `joint_state_broadcaster` + `joint_stream_controller` (active)
+ `cartesian_motion_controller` (inactive 注册, switch_controllers 切换)。控制器配置共用
bringup 的 `<robot>_controllers.yaml` (update_rate 500Hz + RT 线程官方参数)。

命令通道与其他链一致:

```bash
# 关节空间 (注意: mujoco 链手指为 passive, 只列 7 个主关节):
ros2 topic pub -r 50 /joint_stream_controller/command unistackbot_interface/msg/JointCommand \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, gripper], mode: 0, position: [0.5, 1.0, -1.0, 0, 0, 0, 0.05]}"
# 笛卡尔空间 (base 系):
ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped ...
```

## 与其他链的差异

| 项 | mock (SimControlHardware) | gz (GazeboSimSystem) | mujoco (MujocoSystemInterface) |
|---|---|---|---|
| 动力学 | 无 (运动学一阶逼近) | ODE | MuJoCo (implicitfast, timestep 2ms) |
| mimic 手指 | 后端 step() 推导 | 插件 mimic 参数 | MJCF equality (手指 passive, 仅状态接口) |
| JointStream 关节集 | 9 (含手指) | 9 (含手指) | **7 (主关节; 手指由 equality 跟随)** |
| CM 宿主 | standalone ros2_control_node | gz 插件内 | mujoco 定制 ros2_control_node |
| /sim_control | 插件内服务 | side-car 适配器 | 原生四服务 (适配器待接, 见下) |
| 清残留 | mock 双清 | `gz_clean.sh` | 无孤儿进程问题 (普通进程收场) |

## /sim_control 语义 (阶段1 未接适配器)

mujoco 定制节点自带四个服务, 与 `/sim_control` 契约的映射 (适配器后续立项, 先手工用):

| /sim_control | mujoco 原生 | 差异 |
|---|---|---|
| reset | `/mujoco_ros2_control_node/reset_world` | 复位整个世界 (含时间) |
| pause / resume | `/mujoco_ros2_control_node/set_pause` | 单服务布尔参数 |
| step | `/mujoco_ros2_control_node/step_simulation` | |
| set_joint_state | `override_start_position_file` / `set_free_joint_state` | 仅启动期 / 仅自由关节, **运行期关节设置无原生等价** (gz 链同样无) |

另有 `/mujoco_ros2_control_node/apply_external_wrench` (扰动力注入, gz 链没有的能力)。

## 已知坑

- **headless**: apt 0.1.2 无 `MUJOCO_HEADLESS` 环境变量支持 (PR #157 未随发布),
  走 URDF `headless` 硬件参数; 默认无头。GUI 模式需可用 DISPLAY, 收场时渲染线程
  有已知段错误 (GL 析构顺序, demo 同款, 无害——控制器已全部干净关闭)。
- **单链互斥**: 节点与其他链同名 (robot_state_publisher / controller_manager),
  起链前确认无残留 (同 CLAUDE.md 通用规则)。
- **MJCF 资产**: `unistackbot_description/arms/<robot>/mujoco/` (生成/策展/equality
  方向坑见其 README); URDF 惯量/结构改动后需再生成。

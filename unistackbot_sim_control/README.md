# unistackbot_sim_control

统一仿真控制层：**对外**给 ros2_control 提供统一硬件插件，**对内**按后端分类，新仿真平台按同一后端接口扩展。

## 组成

- **`SimControlHardware`**（SystemInterface 插件）—— URDF `<hardware>` 块写死本插件 + `<param name="backend">kinematic</param>`。接口按 URDF 声明镜像导出（effort 恒 0）；动态关节 ≤16，限位 / `max_velocity` / mimic 全来自 `<ros2_control>` 参数。
- **后端**（`sim_backend_factory.hpp` 是唯一认识具体后端类的地方，新仿真 = 子类 + 登记一行）：
  - `kinematic`：理想执行器——限位 clamp + `max_velocity` 饱和的一阶逼近，mimic 关节从源关节推导
  - `threaded`：实时循环独立线程，三通道走 `unistackbot_common` 的无锁原语
- **`/sim_control/*` 服务**（reset / set_joint_state / pause / resume / step）—— 插件进程内自建，命令经 SPSC 无锁队列进实时循环。应答 `success=true` 只代表**已接受**，不代表执行完成。JTC 激活时其保持命令每周期覆盖瞬移，**set_joint_state / reset 需在 pause 下用**。`set_joint_state` 用本包的 `srv/SetJointState.srv`。
- **`sim_control_gz_node`**（可选编译）—— Gazebo 链路的适配器：gz 的 ros2_control 插件是 `GazeboSimSystem`，`/sim_control/*` 改由它承载。pause/resume/step 翻译成 ign 世界服务；`reset`/`set_joint_state` 在 Fortress 无原生等价、直接拒绝（Garden+ 才有）。CMake `QUIET` 探测不到 ignition 库就跳过编译。
- **日志**走 ulog 宏（`ULOG_INFO`/`ULOG_ERROR`）：终端 sink 恒开；URDF 加 `<param name="ulog_file">` 可选开文件 sink。

## 用法

```bash
# mock 链路（kinematic 后端，无仿真器）
ros2 launch unistackbot_bringup piper_control.launch.py

# 回归测试（仓库根，自包含 10 项断言）
bash test/smoke_sim_control.sh
```

## 依赖说明

CMake 直引 `../unistackbot_common`（sp_ring 队列 + ulog 日志；monorepo 内、未 install——对外发布前需调整）。

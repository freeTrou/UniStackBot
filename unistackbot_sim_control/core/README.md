# unistackbot_sim_control

统一仿真控制层：**对外**给 ros2_control 提供统一硬件插件，**对内**按后端分类，新仿真平台按同一后端接口扩展。

## 组成

- **`SimControlHardware`**（SystemInterface 插件）—— URDF `<hardware>` 块写死本插件 + `<param name="backend">kinematic</param>`。接口按 URDF 声明镜像导出（effort 恒 0）；动态关节 ≤16，限位 / `max_velocity` / mimic 全来自 `<ros2_control>` 参数。
- **后端**（`sim_backend_factory.hpp` 是唯一认识具体后端类的地方，新仿真 = 子类 + 登记一行）：
  - `kinematic`：理想执行器——限位 clamp + `max_velocity` 饱和的一阶逼近，mimic 关节从源关节推导
  - `threaded`：实时循环独立线程，三通道走 `unistackbot_common` 的无锁原语
- **`/sim_control/*` 服务**（reset / set_joint_state / pause / resume / step）—— 插件进程内自建，命令经 SPSC 无锁队列进实时循环。应答 `success=true` 只代表**已接受**，不代表执行完成。控制器激活时其保持命令每周期都会覆盖瞬移，**set_joint_state / reset 需在 pause 下用**。`set_joint_state` 用本包的 `srv/SetJointState.srv`。
- **write()/read() 最终防线**（真机驱动同型照抄）: read 侧状态有限性门（非有限保持上一拍）；write 侧 NaN → 限位 clamp → 步长饱和 + 故障计数器。分层语义：控制器层"拒绝"，write 层"clamp"兜底。
- **`/sim_control` 契约**住本包 `include/unistackbot_sim_control/sim_control_contract.hpp`（`SimCommand`/`SimCmdType`/`SimControlServer`；2026-09-17 终局：实现归实现的家、契约归契约的主人）。
- ~~`sim_control_gz_node`~~ 已迁至 `unistackbot_gazebo`（gz 知识归 gz 集成层；本包保留 `/sim_control` 契约与 SimControlServer 脚手架，2026-09-16）
- **日志**走 ulog 宏（`ULOG_INFO`/`ULOG_ERROR`）：终端 sink 恒开；URDF 加 `<param name="ulog_file">` 可选开文件 sink。

## 用法

```bash
# mock 链路（kinematic 后端，无仿真器；robot 必填）
ros2 launch unistackbot_bringup control.launch.py robot:=<机型>

# 回归测试（仓库根，自包含）
bash test/smoke_sim_control.sh     # /sim_control 服务语义深测（piper 关节表，断言含 mimic/限位拒绝/reset）
bash test/fault_injection.sh       # 故障注入测试床：F1 断流 / F2 NaN / F3 限位 / F5 超速 / F6 断流受控减速 / F7 mimic
                                   # （--chain mock|mujoco 可选；write 防线的验收床）
```

## 依赖说明

CMake 直引 `../unistackbot_common`（sp_ring 队列 + ulog 日志；monorepo 内、未 install——对外发布前需调整）。

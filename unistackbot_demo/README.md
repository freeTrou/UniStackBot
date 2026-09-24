# unistackbot_demo · 控制 demo 独立包

> 2026-09-23 从 bringup/scripts 独立成包: demo 是链路契约的**独立外部消费者**,
> 不寄生 bringup/controller。三 demo = 三命令域（设计 §16.2 透传原则）。

## 三命令域

| 命令域 | 程序 | 语言 | 说明 |
|---|---|---|---|
| ① 末端位姿流（< 总线频率） | `demo_cartesian.py` | py | 切 CM 往返/多点位/大幅 sweep；三 pattern 见脚本 docstring |
| ② 关节慢流（< 总线频率） | `demo_motion.py --hz 50` | py | JS **ruckig 档**（链上默认）C2 填充（率失配桥接；A/B 实测同轨迹 dv/拍 平滑 ~100×） |
| ③ 关节满速流（= 控制频率，透传） | `demo_joint_fullrate` | **C++** | ruckig 对满速微小目标步 ≈ 直通（严格透传语义 = hold 档）；**C++ 的理由: python rclpy 发布 500Hz 全是抖动**（100Hz 稳/200Hz 勉强），满速档必须真调度 |

## 用法

```bash
ros2 run unistackbot_demo demo_cartesian.py                 # ① 末端 (默认 up; --pattern axes|sweep)
ros2 run unistackbot_demo demo_motion.py                    # ② 关节慢流 (默认 50Hz)
ros2 run unistackbot_demo demo_joint_fullrate               # ③ 关节满速 (自校准频率)
ros2 run unistackbot_demo demo_joint_fullrate --ros-args -p duration:=6.0 -p hz:=0.0
#   hz=0 (默认) 读 bringup 配置文件 /**.update_rate (单一事实源; -p robot:=<机型> 多配置时必填)
#   bus_hz:= 起链属运行期覆盖, 文件读不到 —— 矩阵测试 1000Hz 档须显式 -p hz:=1000.0
#   结束时如实报告实际达成率 (mock 实测: 校准 500.0Hz (stamp 差分), 定时器下发 99.9% 达成)
```

## 契约消费方式（全部运行期发现，零机型硬编码）

- 关节表/限位/mimic：robot_state_publisher 的 `robot_description` 参数（URDF `<ros2_control>` 块，
  与 JS 消费口径一致；demo 自持解析——外部消费者独立消费契约）
- 频率：`/joint_states` 到达率自校准
- 末端链名：CM 自身参数（`base_link`/`tip_link`）
- 关节命令：`/joint_stream_controller/command`（JointCommand CSP，reliable+KeepLast(1)，列全含 mimic）

## 前置

三链任一已启动（`control` / `ign` / `mujoco` launch）且 `joint_stream_controller` active。
关节 demo 收尾自动走 JS 断流受控减速（链上 yaml `stale_timeout_ms`）。

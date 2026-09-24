# unistackbot_demo · 控制 demo 独立包

> 2026-09-23 从 bringup/scripts 独立成包: demo 是链路契约的**独立外部消费者**,
> 不寄生 bringup/controller。三 demo = 三命令域（设计 §16.2 透传原则）。

## 三命令域

| 命令域 | 程序 | 语言 | 说明 |
|---|---|---|---|
| ① 末端位姿流（< 总线频率） | `demo_cartesian.py` | py | 切 CM 往返/多点位/大幅 sweep/负路径；四 pattern 见脚本 docstring |
| ② 关节慢流（< 总线频率） | `demo_motion.py --hz 50` | py | JS **ruckig 档**（链上默认）C2 填充（率失配桥接；A/B 实测同轨迹 dv/拍 平滑 ~100×） |
| ③ 关节满速流（= 控制频率，透传） | `demo_joint_fullrate` | **C++** | ruckig 对满速微小目标步 ≈ 直通（严格透传语义 = hold 档）；**C++ 的理由: python rclpy 发布 500Hz 全是抖动**（100Hz 稳/200Hz 勉强），满速档必须真调度 |

**大幅快慢变速测试**（2026-09-24 用户需求：>10s 长时 + 大幅 + 有快有慢）——三 demo 各一条：
- ① `--pattern sweep`：部署慢/前伸快 → 圆周 16 点逐点快慢交替 → 上浮快/回心慢 → 收回，全程 `--speed`×每腿系数（慢 0.5/快 1.6），总程 >20s
- ② 默认运动：四段计划 85%→15%→85%→零位（限位区间百分比），快腿 2.0/2.5s 与慢腿 6.0/4.0s 交替，~14.5s
- ③ 默认运动：同②口径四段快慢（`duration:=` 总时长，各段按占比缩放），默认 14.5s

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
- 频率：bringup 配置文件 `/**.update_rate` 直读（单一事实源；多配置 `-p robot:=` 必填；`/joint_states` stamp 差分中位数仅兜底）
- 末端链名：CM 自身参数（`base_link`/`tip_link`）
- 关节命令：`/joint_stream_controller/command`（JointCommand CSP，reliable+KeepLast(1)，列全含 mimic）

## 前置

三链任一已启动（`control` / `ign` / `mujoco` launch）且 `joint_stream_controller` active。
关节 demo 收尾自动走 JS 断流受控减速（链上 yaml `stale_timeout_ms`）。

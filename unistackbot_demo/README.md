# unistackbot_demo · 控制 demo 独立包

> 2026-09-23 从 bringup/scripts 独立成包: demo 是链路契约的**独立外部消费者**,
> 不寄生 bringup/controller。三 demo = 三命令域（设计 §16.2 透传原则）。

## 三命令域

| 命令域 | 程序 | 语言 | 说明 |
|---|---|---|---|
| ① 末端位姿流（< 总线频率） | `demo_cartesian.py` | py | 切 CM 五 pattern: up/axes/sweep/**circle**(连续小步圆周流, IK 解连续性探针)/reject |
| ② 关节慢流（< 总线频率） | `demo_motion.py --hz 50` | py | JS **ruckig 档** C2 填充; **测试工况库** `--pattern broad\|single\|couple\|agile`（③同款） |
| ③ 关节满速流（= 控制频率，透传） | `demo_joint_fullrate` | **C++** | ruckig 对满速微小目标步 ≈ 直通（严格透传语义 = hold 档）；**C++ 的理由: python rclpy 发布 500Hz 全是抖动**（100Hz 稳/200Hz 勉强），满速档必须真调度 |
| ④ 键盘遥操作（交互） | `demo_teleop_key.py` | py | 键态→笛卡尔速度→目标空间积分→CM 目标流; hold-to-move+deadman; 零外部依赖 (termios) |

**大幅快慢变速测试**（2026-09-24 用户需求：>10s 长时 + 大幅 + 有快有慢）——三 demo 各一条：
- **测试工况库 (2026-09-29, ②③ 同款; 代表性点位 = 能暴露问题的工况)**:
  - `--pattern broad`（默认）: 现四段快慢大幅序列 —— 综合回归
  - `--pattern single`: 逐关节单轴扫 90%→10%→home（其余锁 home）—— 单关节隔离诊断（跟踪/限位/振荡, 无耦合干扰）
  - `--pattern couple`: 全关节同相往返 + 相邻反相往返 —— 最大同向耦合惯量 / 哥氏扰动工况
  - `--pattern agile`: 全关节 mid±8% 快速往返 ×10 —— 伺服带宽/振荡探针（stationlite j4 振动工况的复现器）
- ① `--pattern sweep`：部署慢/前伸快 → 圆周 16 点逐点快慢交替 → 上浮快/回心慢 → 收回，全程 `--speed`×每腿系数（慢 0.5/快 1.6），总程 >20s
- ② 默认运动：四段计划 85%→15%→85%→零位（限位区间百分比），快腿 2.0/2.5s 与慢腿 6.0/4.0s 交替，~14.5s
- ③ 默认运动：同②口径四段快慢（`duration:=` 总时长，各段按占比缩放），默认 14.5s

## 用法

```bash
ros2 run unistackbot_demo demo_cartesian.py                 # ① 末端 (默认 up; --pattern axes|sweep|circle)
ros2 run unistackbot_demo demo_motion.py                    # ② 关节慢流 (默认 50Hz)
ros2 run unistackbot_demo demo_joint_fullrate               # ③ 关节满速 (自校准频率)
ros2 run unistackbot_demo demo_joint_fullrate --ros-args -p duration:=6.0 -p hz:=0.0
#   hz=0 (默认) 读 bringup 配置文件 /**.update_rate (单一事实源; -p robot:=<机型> 多配置时必填)
#   bus_hz:= 起链属运行期覆盖, 文件读不到 —— 矩阵测试 1000Hz 档须显式 -p hz:=1000.0
#   结束时如实报告实际达成率 (mock 实测: 校准 500.0Hz (stamp 差分), 定时器下发 99.9% 达成)

ros2 run unistackbot_demo demo_teleop_key.py                  # ④ 键盘遥操作·笛卡尔 (CM 域)
#   键位: W/S前后 A/D左右 R/F升降 | I/K俯仰 J/L偏航 U/O翻滚 (Shift=2×瞬时)
#         -/= 速度档(3档: 精细20/常规50/快速100 mm·s) Space=HOLD乒乓 Q=退出
#   启动自动切 CM, 退出切回 JS + 显式发 HOLD (瞬时冻结); 需真实终端 (stdin tty)

ros2 run unistackbot_demo demo_teleop_joint.py                # ④ 键盘遥操作·关节 (JS 域, 2026-10-08)
#   键位: 一列一关节 Q/W/E/R/T/Y/U=J1..J7 往上限, A/S/D/F/G/H/J=往下限
#         -/= 档位(0.25/0.5/1.0 rad/s) Shift=2× Space=清键急停 Ctrl-C=退出
#   直发 JS (不过 IK: 无奇异/无姿态冻结重构/顶限位=贴住); 关节表自持解析 URDF
#   E2E (r1_7a mujoco): q/a 键控 J1 ±0.8/0.55rad, w 贴限位钉住+HUD■, 264条/11.5s
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

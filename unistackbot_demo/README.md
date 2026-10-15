# unistackbot_demo · 控制 demo 独立包

> 2026-09-23 从 bringup/scripts 独立成包: demo 是链路契约的**独立外部消费者**,
> 不寄生 bringup/controller。分层: 教学件 → 机型控制件 → 通用压力件 (自上而下读)。

## 分层总览

| 层 | 程序 | 定位 |
|---|---|---|
| **教学件**（机型无关） | `minimal_joint.py` / `minimal_cartesian.py` | 链路契约的最小正确用法, 5 分钟上手入口 |
| **机型控制件**（`scripts/<robot>_arm/`） | `r1_7a_demo_joint.py` / `r1_7a_demo_cartesian.py` | 机型接入参考实现: 完整工况 + 防御细节样板 |
| **通用压力件** | `demo_motion` / `demo_cartesian` / `demo_joint_fullrate` | 测试工况库: 快慢变速/单轴/耦合/带宽探针, 频率矩阵配餐 |
| **交互遥操** | `demo_teleop_key.py` / `demo_teleop_joint.py` | 键盘遥操作 (笛卡尔域/关节域) |
| **数据消费** | `demo_points_playback.py` | 点位资产回放 (录制规范 v1 的 json v1) |

## 为什么切 CM？(接入第一问)

`joint_stream_controller`（JS, 默认 active）与 `cartesian_motion_controller`（CM）
写**同一批关节 position 命令接口**——ros2_control 资源管理规则: 同一命令接口
同时至多一个 active 控制器持有。这是框架强制的安全互斥（杜绝双写竞争），
不是仪式。所以笛卡尔控制 = **切换接管 → 用完切回**，全部 demo 的
`switch_controllers` 调用都在教这件事。切换 ~1s, 换接口级安全。

## 最小接入教学 demo（`scripts/minimal/<形态>/`，按形态分类，2026-10-12 立 / 10-14 归类）

**控制开发人员 5 分钟上手入口**——链路契约的最小正确用法，零机型特化
（关节表解析自 URDF、末端链读自 CM 参数、目标从反馈派生）：

```bash
ros2 run unistackbot_demo minimal_joint.py       # 关节域: 读反馈 → 发一个目标 → 回位 (~110 行)
ros2 run unistackbot_demo minimal_cartesian.py   # 笛卡尔域: 拿反馈 → 切 CM → 单目标 → 切回 (~150 行)
#   目标相对当前位姿派生 (--dz 可负) —— 重复运行向边界累积属预期, 拒绝即错误处理教学
```

两个文件各一条主线、每行注释讲"这条契约怎么消费"——**接入方先读这两个再上手**。

## 机型专属控制 demo（`scripts/<robot>_arm/`，2026-10-12 约定）

每台机器人一个文件夹放**机型接入参考实现**（关节 + 笛卡尔各一），文件名带机型前缀
——install 平铺进 `lib/`（`ros2 run` 不认子目录，重名即冲突），新机器人按此复制。
限位读自链上 URDF（单一事实源），航点为机型常量并在启动期逐点断言。

| 机型 | 关节 demo | 笛卡尔 demo |
|---|---|---|
| r1_7a | `ros2 run unistackbot_demo r1_7a_demo_joint.py` | `ros2 run unistackbot_demo r1_7a_demo_cartesian.py` |

- **r1_7a 关节**: 16 段航点链（基座连续扫/折叠双档/腕组合 A/B）→回近零位，~20s
- **r1_7a 笛卡尔**: 部署→两圈 36 点连续圆周流（快/慢）→收回→回关节零位（err 全程 <1mm；J6 带几何/落回离奇异核 2cm 已内建；IK 错误语义 last_result+min_sigma 人话打印）
- 生命周期纪律样板: 启动切 CM → 退出切回 JS → **收尾回关节零位**（环境留给下一个消费者）

## 通用命令域 demo

| 命令域 | 程序 | 语言 | 说明 |
|---|---|---|---|
| ① 末端位姿流（< 总线频率） | `demo_cartesian.py` | py | 切 CM 五 pattern: up/axes/sweep/**circle**(连续小步圆周流, IK 解连续性探针)/reject |
| ② 关节慢流（< 总线频率） | `demo_motion.py --hz 50` | py | JS **ruckig 档** C2 填充; **测试工况库** `--pattern broad\|single\|couple\|agile`（③同款） |
| ③ 关节满速流（= 控制频率，透传） | `demo_joint_fullrate` | **C++** | ruckig 对满速微小目标步 ≈ 直通（严格透传语义 = hold 档）；**C++ 的理由: python rclpy 发布 500Hz 全是抖动**（100Hz 稳/200Hz 勉强），满速档必须真调度 |
| ④ 键盘遥操作 | `demo_teleop_key.py` / `demo_teleop_joint.py` | py | 笛卡尔域 (CM)/关节域 (JS 直发不过 IK) |
| ⑤ 点位资产回放 | `demo_points_playback.py` | py | 录制规范 json v1 的序列回放 (`--seq @序列名`) |

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
# 教学件 (先跑这两个):
ros2 run unistackbot_demo minimal_joint.py
ros2 run unistackbot_demo minimal_cartesian.py   # [--dz 0.10] 可负自愈

# 机型控制件:
ros2 run unistackbot_demo r1_7a_demo_joint.py
ros2 run unistackbot_demo r1_7a_demo_cartesian.py   # [--lift 0.15] [--fwd 0.15] [--span 0.08]

# 通用压力件:
ros2 run unistackbot_demo demo_cartesian.py                 # ① 末端 (默认 up; --pattern axes|sweep|circle)
ros2 run unistackbot_demo demo_motion.py                    # ② 关节慢流 (默认 50Hz)
ros2 run unistackbot_demo demo_joint_fullrate               # ③ 关节满速 (自校准频率)
ros2 run unistackbot_demo demo_joint_fullrate --ros-args -p duration:=6.0 -p hz:=0.0
#   hz=0 (默认) 读 bringup 配置文件 /**.update_rate (单一事实源; -p robot:=<机型> 多配置时必填)
#   bus_hz:= 起链属运行期覆盖, 文件读不到 —— 矩阵测试 1000Hz 档须显式 -p hz:=1000.0
#   结束时如实报告实际达成率 (mock 实测: 校准 500.0Hz (stamp 差分), 定时器下发 99.9% 达成)

# 遥操 (需真实终端 stdin tty):
ros2 run unistackbot_demo demo_teleop_key.py                  # ④ 键盘遥操作·笛卡尔 (CM 域)
#   键位: W/S前后 A/D左右 R/F升降 | I/K俯仰 J/L偏航 U/O翻滚 (Shift=2×瞬时)
#         -/= 速度档(3档: 精细20/常规50/快速100 mm·s) Space=HOLD乒乓 Q=退出
#   启动自动切 CM, 退出切回 JS + 显式发 HOLD (瞬时冻结)
ros2 run unistackbot_demo demo_teleop_joint.py                # ④ 键盘遥操作·关节 (JS 域, 2026-10-08)
#   键位: 一列一关节 Q/W/E/R/T/Y/U=J1..J7 往上限, A/S/D/F/G/H/J=往下限
#         -/= 档位(0.25/0.5/1.0 rad/s) Shift=2× Space=清键急停 Ctrl-C=退出
#   直发 JS (不过 IK: 无奇异/无姿态冻结重构/顶限位=贴住); 关节表自持解析 URDF

# 点位回放:
ros2 run unistackbot_demo demo_points_playback.py --points-file <v1.json> --seq @<序列名>
#   side/hz 自动从文件 meta 读 (显式 --side/--hz 优先); 详细见 unistackbot_tool README §3/§4
```

## 契约消费方式（全部运行期发现，零机型硬编码）

- 关节表/限位/mimic：robot_state_publisher 的 `robot_description` 参数（URDF `<ros2_control>` 块，
  与 JS 消费口径一致；demo 自持解析——外部消费者独立消费契约）
- 频率：bringup 配置文件 `/**.update_rate` 直读（单一事实源；多配置 `-p robot:=` 必填；`/joint_states` stamp 差分中位数仅兜底）
- 末端链名：CM 自身参数（`base_link`/`tip_link`）
- 关节命令：`/joint_stream_controller/command`（JointCommand CSP，reliable+KeepLast(1)，列全含 mimic）

## 前置

三链任一已启动（`control` / `ign` / `mujoco` launch）且 `joint_stream_controller` active。
关节 demo 收尾停发命令，臂按断流语义（2026-10-10 统一）收敛到最后目标保持。

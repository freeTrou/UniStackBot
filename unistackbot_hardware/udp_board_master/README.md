# unistackbot_udp_board —— Lumos UDP 电控板接入（mos_hal 融入 bus 第四总线）

**状态（2026-10-30, Phase 0B）**：帧契约 BoardProtocol + FakeBoard 虚拟电机板已落地（28 黄金 + RTT 自测全绿）；UdpBoardMaster 主站（MasterBase 实现，ODE 裁决 4 "UDP=第四条总线自写"）进后续批次。

## 帧契约（golden 单一事实源）

| 帧 | 方向 | 尺寸 | 布局 |
|---|---|---|---|
| Control | 主机→板 | **278B** | ts(1)+id(1,0x02)+bus[3]×65(臂/头)+bus_chassis(81) |
| Response | 板→主机 | **474B** | ts(1)+id(1,0x01)+bus[3]×96(臂/头)+bus_chassis(184) |

- **golden = `debug/example_run_frames.md` 字节实录**（mos_hal2.0 @ `origin/feature/single-motor-control`），protocol.h 有漂移在案（ODE 警告），冲突以实录为准；
- `_MotorCmd` 8B 小端位域：enable(1)+kd_ki(10)+kp(11)+tar_tor(12)+tar_vel(15)+tar_pos(15)；
- 位域布局前提：小端 + gcc 位域分配（与板固件/原 HAL 同编译器族），`static_assert` 锁全族尺寸 + 黄金测试双重保险（含 fake_motor.py 的 python 位解释交叉视角对拍）；
- **raw 计数不定标**：tar_vel raw ≈ rad/s×1000（实录），逐关节 direction/增益住 HAL 标定 yaml → 装配层。

## FakeBoard（虚拟电机板，零硬件）

对等移植 `debug/fake_motor.py`，两处增强：① 臂/头电机命令回声（tar→cur，让臂链路端到端可测）；② 纯函数核心（`process`）与 UDP 壳分离。底盘假数据 = 实录 golden 规则（电机 i：vel=1000+i，pos=5000+i×100，tor=i，status=3，error=0）。

## 测试（g++ 直编，零 ROS）

```bash
bash test/run_udp_board_test.sh
# board_protocol: 28 pass, 0 fail   (尺寸族/位域往返/实录锚 540·1122/python 交叉视角/假数据/门禁)
# udp_board RTT: 2000/2000 frames ok, p50=110us p99=126us max=406us  (本机回环; 对照总线 200Hz=5ms 周期)
```

## 后续批次

UdpBoardMaster（MasterBase 第四实现：UDP 泵/板状态机 board_safety/RTT 遥测）——挂 Phase 3 前置，真板接入等电控组 UDP_DLY_TIME 回答（ODE 外部依赖三问）。

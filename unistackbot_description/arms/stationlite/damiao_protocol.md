# 达妙(DM)电机 CAN 协议提取 —— stationlite 真机接入规格

> 2026-10-21 提取,同日**官方资料仓补齐后修订**(`https://gitee.com/kit-miao/damiao` → `/home/work/project/damiao/`)。来源:①官方手册 DM-J4310-2EC **V1.2 版说明书 V1.4**(2026-09-14,stationlite j4-6/夹爪,startouchlib 引用版)+ DM-J4340-2EC **V1.1 版说明书 V1.3**(2026-08-13,j1-3;旧 PDF `~/下载/达妙电机协议.pdf` = 4310 **V1.0 已停产**)②startouchlib 源码 `damiao_motor/` 逐字节 codec。**用途 = 自研 `unistackbot_protocol` 的 `dm_motor_core` 内芯规格输入 + 甲骨文对拍依据;本仓不引厂商代码**。
> 三方闭环:手册 ↔ SDK 源码 ↔ 回环 mcap,冲突处已标注。

## 1. 总线与 ID 拓扑

| 项 | 值 | 来源 |
|---|---|---|
| 总线 | 经典 CAN(非 FD)1Mbps,每臂一条:can0/can1 | mcap 文件名 + `start_can.sh` |
| **臂 CAN ID** | **j1-j6 send = 0x01-0x06,recv = 0x11-0x16** | 正式接口 `interfaces_0825fasttest.cpp:3687-3688` 写死(example 同值) |
| **夹爪 CAN ID** | **DM4310 @ send 0x07 / recv 0x17** | 同上 `:3692` |
| 参数/广播帧 CAN ID | **0x7FF**(参数读写 + refresh + 存储参数) | `create_query_param_command` |
| stationlite 电机 | j1-j3 = **DM4340** / j4-j6 = **DM4310**(4 处源码一致) | `interfaces_0825fasttest.cpp:3683-3686` 等 |
| 内核接口参数 | `restart-ms 100`(bus-off 自恢复)+ `txqueuelen 1000` | `start_can.sh` |

## 1b. 域约定与 motor_signs(codec 接口语义,2026-10-21 冻结)

- **CAN 帧上的 q/dq/tau = 电机域原样**(带电机自身方向),关节域 = `motor_signs[i] × 电机域`(±1 双向同乘,`interfaces_0825fasttest.cpp:809-810` 的 `motor_to_joint/joint_to_motor`)
- stationlite `motor_signs = [1,-1,-1,-1,-1,1]`(j2-j5 电机正方向 ≠ URDF 关节正方向,住 `robot_kinematics.yaml`)
- **自研分层定案:`dm_motor_core` 保持字节↔电机域,signs 换算归上层机型配置**(真机 yaml per-joint)——协议词汇零出包,signs 属装配配置非协议

## 1c. 启动/使能时序(DmSafety 握手设计输入,源码 `initial_startouch` + 3899-3915)

1. `init_arm_motors`(类型+ID 表)→ 夹爪 clrErr → 切 MIT 模式(写 RID=10)→ 读 KT/Imax/TMAX(力位 Nm 换算基准,TMAX 优先于 KT×Imax)
2. **`set_timeout_all(0)`——厂商整机禁用了驱动器看门狗**(RID=9 TIMEOUT 写 0;疑因 400Hz 流 p95 10ms 抖动会误触发通讯丢失退使能)。**我们的裁决方向:保留看门狗**(框架哲学"驱动器看门狗=安全终点"),timeout 设大于主站抖动上界——接入时验证抖动实际分布再定值
3. 使能握手:`enable_all` → recv 5ms 超时 → 逐电机查 error_code==1 → **失败重试 3 次、间隔 2ms**
4. 紧急停 = 控制环内 `disable_all`;控制频率 `kControlHz=400.0`(mcap 印证)
5. MIT 模式挂重力/科氏/惯量前馈三开关(`enable_gravity/coriolis/inertia_compensation`,mcap τ_ff 47% 承载即此)

## 2. 定点换算(全协议数值轴)

```
编码 double→uint:  x = clamp(x, min, max);  uint = trunc((x-min)/(max-min) * (2^bits - 1))   # 注意: 向零截断 (非四舍五入; SDK static_cast 语义, 甲骨文实证)
解码 uint→double:  val = uint/(2^bits - 1) * (max-min) + min
```
注意分母是 `(1<<bits)-1`(全幅编码,不是 2^bits)。源码 `double_to_uint`/`uint_to_double`(`dm_motor_control.cpp:282-293`)。

## 3. 电机限值表(定点换算的量程来源)—— 三个数字三个语义

`MOTOR_LIMIT_PARAMS`(SDK 协议映射上界)vs **手册物理值**(V1.3/V1.4 新手册)vs **URDF effort**(厂商整机安全档):

| 型号(板本) | SDK tMax(协议) | 手册额定/峰值(物理) | 减速比 | 峰值相电流 | URDF effort(整机) |
|---|---|---|---|---|---|
| **DM4340 V1.1**(j1-3) | 28 | **12 / 40 Nm** | **40:1** | 19.85A | **27** |
| **DM4310 V1.2**(j4-6+夹爪) | 10 | **3.5 / 12.5 Nm** | 10:1 | 20A | **7**(J4)/ 12.5(J5/J6) |
| DM4310 V1.0(停产,旧 PDF) | 10 | 3 / 7 Nm | 10:1 | 9.8A | — |

- **旧 PDF 是停产 V1.0**——此前"峰值 7"的结论基于它;**现行 V1.2 物理峰值 12.5Nm**(URDF J5/J6 effort=12.5 由此而来,不是厂商填错)。J4 的 7 是整机保守档(≈V1.0 值)。
- **语义链**:协议 tMax = MIT 定点映射上界(可 RID=23 改);手册峰值 = 电机物理能力;URDF effort = 厂商整机安全档。**我们的 write 层安全钳位沿用 URDF effort 值(27/27/27/7/12.5/12.5)——厂商整机口径,最保守且已验证**。
- SDK `MOTOR_LIMIT_PARAMS` 其余型号(3507/6006/8006/8009/10010/DMH/DMG)全表见 `dm_motor_constants.hpp:100-114`,本机不用。
- **TMAX 可被参数覆盖**:反馈解码时若电机参数表读到 RID=23(TMAX)则优先用(`dm_motor_control.cpp:114-117`)——跨机型对拍先统一 TMAX 语义再比字节。
- MIT 模式衍生(手册明示):**kp=0 且 kd≠0 时给 v_des = 匀速转动**(速度模式的实现法)。

## 4. 控制帧(stationlite 全走 MIT,余两种备查)

### 4.1 MIT 模式(CAN ID = send_id,8 字节)

```
D0      D1      D2      D3          D4      D5      D6          D7
q>>8    q&FF    dq>>4   (dq&F)<<4   kp&FF   kd>>4   (kd&F)<<4   tau&FF
                        |kp>>8                       |tau>>8
位宽:   q=16bit  dq=12bit  kp=12bit  kd=12bit  tau=12bit
量程:   q∈[-pMax,pMax]  dq∈[-vMax,vMax]  kp∈[0,500]  kd∈[0,5]  tau∈[-tMax,tMax]
```
源码 `pack_mit_control_data`(`dm_motor_control.cpp:146-166`)。**kd=0 被手册明示禁止**(位置控制会震荡甚至失控)。

### 4.2 POS_VEL 模式(CAN ID = send_id + 0x100,8 字节)

`float position(LE) ×4B + float max_velocity(LE) ×4B`——IEEE754 直传,无限点。源码 `pack_posvel_control_data`。

### 4.3 POS_FORCE 模式(CAN ID = send_id + 0x300,8 字节)

`float position(LE) ×4B + u16 velocity(×100 LE) + u16 current_limit(归一化 [0,1]×10000 LE)`。源码 `pack_posforce_control_data`。

## 5. 特殊命令帧(CAN ID = send_id,8 字节 = `FF FF FF FF FF FF FF <cmd>`)

| cmd | 语义 |
|---|---|
| 0xFC | **使能** |
| 0xFD | **失能** |
| 0xFB | **清故障** |
| 0xFE | **设当前位置为零位**(标定用,慎发) |

源码 `create_enable/disable/clrErr/set_zero_command`。使能/失能/清错的时序节奏(间隔/重试)住上层,库内无约束——接入时按 mcap 或上层 SDK 补。

## 6. 参数帧(CAN ID = 0x7FF)

| 帧型 | D0-D1 | D2 | D3 | D4-D7 |
|---|---|---|---|---|
| 查询 | send_id LE | **0x33** | RID | — |
| 写入 | send_id LE | **0x55** | RID | 值(uint32 或 float,LE) |
| refresh(触发单次反馈) | send_id LE | **0xCC** | 0 | 0×4 |

**值类型按 RID 分段**(`is_in_ranges`,`dm_motor_control.cpp:309-312`):RID ∈ {7-10, 13-16, 35-36} 为 **uint32**,其余为 **float**:
- uint32 段:7 MST_ID / 8 ESC_ID / 9 TIMEOUT / 10 CTRL_MODE / 13 hw_ver / 14 sw_ver / 15 SN / 16 NPP / 35 can_br / 36 sub_ver
- float 段:0 UV / 1 KT / 2 OT / 3 OC / 21 PMAX / 22 VMAX / **23 TMAX** / 59 Imax / …(全表 82 项见 `dm_motor_constants.hpp` RID enum)

**控制模式切换 = 写 RID=10**(1=MIT / 2=POS_VEL / 3=VEL / 4=POS_FORCE),帧 `[id_lo, id_hi, 0x55, 0x0A, mode, 0, 0, 0]`。

**0x55 写的是 RAM,掉电丢**(手册明示"立即生效,无法存储")——**持久化需存储参数帧**(SDK 未实现,手册独有):
```
0x7FF: [CANID_L, CANID_H, 0xAA, 0x01]   → 回显同帧 = 成功
```
约束:只在失能模式生效 / 一次性存全部参数 / 耗时最长 30ms / **flash 擦写寿命约 1 万次,禁频繁发**。

## 7. 反馈帧(CAN ID = recv_id,8 字节)

```
D0           D1-D2      D3/D4高      D4低/D5      D6        D7
(ID&0xF)     q 16bit    dq 12bit     tau 12bit    T_MOS℃    T_rotor℃
|(ERR&0xF)<<4
```
- D0 = `(error_code << 4) | motor_id`——ID 与错误码共用首字节(`parse_motor_state_data`)
- q/dq/tau 解码量程 = §3 限值表(TMAX 可被 RID=23 覆盖)
- 温度**直读摄氏整型**,无限点
- 反馈随每条控制帧返回(命令-回显式);另有 0x7FF+0xCC refresh 主动触发

## 8. 状态码表(手册,反馈 D0 高 4 位)—— 完整版含 3/4/5

| 码 | 含义 | 粗态映射建议(自研 DmSafety) |
|---|---|---|
| 0 | 失能(上电默认) | READY |
| **1** | **使能(正常)** | ENABLED |
| **3** | **输出轴校准异常**(V1.3/V1.4 手册新增,旧 V1.0 无) | FAULT(标定类) |
| **4** | **传感器输出异常**(同上) | FAULT(标定类) |
| **5** | **电机编码器校准异常**(同上) | FAULT(标定类) |
| 8 | 超压 | FAULT |
| 9 | 欠压 | FAULT |
| A | 过电流 | FAULT |
| B | MOS 过温 | FAULT |
| C | 线圈过温 | FAULT |
| D | 通讯丢失 | FAULT(总线侧) |
| E | 过载 | FAULT |

SDK 实证:`enabled_ = (error_code == 0x1)`(`dm_motor.cpp:62`)。清故障握手 = 发 0xFB 后等 error 恢复 0(失能态; 手册: 清错不自动使能)——框架侧 NodeSafety 守卫即按 error_raw==0 确认。**硬件排障对照**:绿灯常亮=使能 / 红灯常亮=失能 / 红灯闪=故障(码型见上表)。

## 9. 保护与看门狗(手册 V1.3/V1.4)

| 项 | DM4340 V1.1 | DM4310 V1.2 |
|---|---|---|
| 额定电压 | 24V(48V 版另列) | 24V |
| 驱动(MOS)过温 | 120℃ 退使能 | 120℃ |
| 线圈过温 | 建议 ≤100℃ 退使能(可设定) | 同左 |
| 过压 | 建议 ≤32V(48V 版 60V) | 同左 |
| 欠压 | <15V 退使能 | 同左 |
| 过流 | **系数 0.98(归一化 Imax,可设定)** | 同左 |

- **驱动器自带看门狗**:设定周期内未收到 CAN 指令自动退使能——与宇树同为"驱动器看门狗=安全终点",主站断流语义天然安全
- 双编码器:输出轴单圈绝对(掉电不丢)+ 电机侧增量;MIT 的 q 域 = 输出侧,与 URDF 关节角同域,减速比(4340=40:1、4310=10:1)已内含

## 10. stationlite 整机实测档案(can0 mcap 2026-10-20)

- 增益:kp=[300,490,490,65,45,45] kd=[5,5,5,2,2,2](j2/j3 kp=490、j1-3 kd=5 顶格/近顶;5 反向 motor_signs=[1,-1,-1,-1,-1,1] 住 `robot_kinematics.yaml`)
- 跟踪基线:运动段 err p50≈120mrad(纯跟随滞后,corr(err,v)=−0.96);静止段≈0(j3 承 6Nm 重力变形 12mrad)
- τ_ff 能量占比 47%(厂商重力/科氏前馈在跑)
- error_code 全程=1(使能),温度 28-34℃
- cycle ~400Hz(p50 2.51ms);response_latency 字段为跨时钟域产物,**不可作时序对齐**

## 11. 自研内芯落位(unistackbot_protocol)—— 范围裁决(2026-10-21)

**用户纪律:只实现"用得到 + 已验证"的部分;增量暂缓、未验证不做。**

### 第一版实现(已验证 = mcap 全程在跑/SDK 在用/手册在案三方齐)

- **MIT 控制帧**(§4.1)——整机唯一控制模式,mcap 全程实证
- **反馈帧**(§7)——mcap 全程实证
- **特殊命令帧**:使能 0xFC / 失能 0xFD / 清故障 0xFB(SDK 在用)
- 定点换算(§2)+ 状态码 0/1 的粗态语义(mcap 全程实证;8-E 故障码进 DmSafety 查表映射,无需额外逻辑)
- 甲骨文 = startouchlib `pack_mit_control_data`/`parse_motor_state_data` 逐字节对拍(宇树 demo 先例);非有限输入一票拒绝;增益界 [0,500]/[0,5] 钳位语义与宇树 kp/kd 先验同款
- 限值/增益全部从**我们的机型 yaml** 配置(单一事实源是我们自己的配置,不读电机参数)

### 暂不实现(用到再加,加了必须带验证)

| 项 | 为什么暂缓 |
|---|---|
| POS_VEL(0x100)/VEL/POS_FORCE(0x300)帧 | SDK 有但整机不走(mcap 无此流量),未验证 |
| 参数帧 0x33/0x55 读写 | 限值走 yaml 不读电机参数;上电不需要 |
| 存储参数帧 0xAA | SDK 都未实现,纯手册增量 |
| 设零位 0xFE | 标定 SOP 专用工具,不进运行时内芯(零位标定时另验) |
| 故障码 3/4/5 专门处理 | V1.3 手册新增,mcap 未出现过——码表登记即可 |
| kp=0 匀速衍生 | 用不到 |

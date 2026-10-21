# stationlite(FastTouch V2)真机接入资料卡

> 2026-10-20 立项。**接入裁决:厂商资料只参考,代码全部按本仓四轴分层写**(protocol 内芯 / statemachine / bus / master),不引 startouchlib 代码——验收 = 框架包 git diff 为零。SDK 的 `pack_mit_control_data` 仅作字节级对拍甲骨文(宇树 demo 先例)。

## 机型事实(三方实证:手册 ↔ SDK 源码 ↔ 回环 mcap)

| 项 | 值 | 来源 |
|---|---|---|
| 电机 j1-j3 | DM4340(MIT tMax 28Nm) | print_joints.cpp / zero_calibrate.cpp |
| 电机 j4-j6 | DM4310-2EC(额定 3Nm/峰值 **7Nm**,减速比 10:1,双编码器输出单圈绝对) | 手册 V1.0 |
| 总线 | can0/can1 每臂一条,经典 CAN(达妙 1Mbps),MIT 帧格式 | mcap 命名 + 手册 |
| q 域 | 输出侧(减速比内置),与 URDF 关节角同域,零换算 | 手册 |
| error_code | **1=使能态非错误**;故障码 8 超压/9 欠压/A 过流/B MOS 过温/C 线圈过温/D 通讯丢失/E 过载 | dm_motor.cpp:62 + 手册 |
| MIT 增益界 | kp∈[0,500] kd∈[0,5] 各 12bit 定点;**kd 不可为 0(手册明示震荡)** | dm_motor_control.cpp:148 + 手册 |
| 整机增益档案 | kp=[300,490,490,65,45,45] kd=[5,5,5,2,2,2](j2/j3 kp、j1-3 kd 顶格) | mcap 实测 |
| motor_signs | [1,-1,-1,-1,-1,1](j2-j5 反向;**帧=电机域,signs 双向同乘归上层配置**——域约定 2026-10-21 冻结) | robot_kinematics.yaml + fasttest:809-810 |
| CAN ID | 臂 j1-6 send 0x01-06/recv 0x11-16,**夹爪 0x07/0x17**;can0/can1 每臂一套 | fasttest:3687-3692 写死 |
| 保护阈值 | 驱动 120℃/线圈 100℃/过压 32V/欠压 15V/过流 9.8A;**驱动器自带看门狗**(超时自动退使能) | 手册 |
| 真机跟踪基线 | 运动段 err p50≈120mrad(纯跟随滞后,corr(err,v)=−0.96;静止段≈0) | can0 mcap 20261020 |
| 前馈结构 | τ_ff 能量承载 47%(厂商重力/科氏前馈在跑) | can0 mcap |

**安全钳位纪律**:用手册值(DM4310=7Nm),不用协议 TMAX=10(TMAX 是定点映射上界非物理限)。

## 实现计划(2026-10-21 定稿:A+B 两线贯穿,真机最后)

**结构:阶段一全部零硬件(六批自由穿插)→ 阶段二真机窗口(一次到位)。**

两条纪律贯穿全程:**厂商资料只参考代码全自研**(四轴分层,验收=框架包 git diff 为零);**只实现"用得到+已验证"**——增量暂缓、未验证不做(分档见 `damiao_protocol.md` §11)。配置单一事实源 = 我们的 yaml(限值=URDF effort 口径/增益=mcap 档案/signs per-joint,不读电机参数)。

### 阶段一:零硬件(可并行/穿插,全部离线可验收)

| 批 | 内容 | 验收 |
|---|---|---|
| **A1 协议内芯** | `unistackbot_protocol` 加 `dm_motor_core`:MIT 控制帧+反馈帧+使能/失能/清故障帧 codec(§11 分档);契约类型照 unitree_im(电机域原样,signs 不进协议) | g++ 直编测试绿:甲骨文逐字节对拍(startouchlib pack/parse)、定点换算直算((2^bits)−1 全幅)、非有限拒收、增益界 [0,500]/[0,5]、ID\|ERR 拆分、全码表 0/1/3-5/8-E |
| **A2 状态机** | `unistackbot_statemachine` 加 `DmSafety`:码表→粗态映射、使能/断使能意图、清故障握手(3 重试×2ms,请求生命周期自持照 Cia402) | 77 cases 同款深度,经工厂/基类指针测 |
| **A3 SocketCAN 主站** | `canfd_master` 骨架落地(经典 CAN 模式):open/read/write+泵线程+逐节点时隙(6+1 电机@400Hz)+quick_stop/fault_clear 总线化+看门狗参数化(默认保守值);restart-ms/txqueuelen 入装配文档 | **vcan0 虚拟总线 E2E**:dm 帧→vcan→回读对拍、时隙节拍、断流语义 |
| **A3.5 标定预读** | 读 `zero_calibrate.cpp` 提取零位标定 SOP 文档化 | SOP 入本 README |
| **B1 IK 指纹+推导** | `check_parallel_axes` 结构指纹(j2/j3/j4 轴平行最小二乘);Pieper 形式二闭式解推导(解族/分支/退化);预读厂商 `analytic_ik.hpp`(交叉真值+分支约定,参考不搬码) | 指纹正反例测试+推导稿过目 |
| **B2 IK 实现** | `analytic_stationlite`(照 analytic_piper 组织):全解枚举→限位过滤(kLimitEpsRad 微容差)→距 seed 最近解;结果码语义同款 | 编译零警告+契约单测 |
| **B3 IK 验证+切换** | FK 回代黄金测试+厂商 analytic 交叉对拍+oracle 生成+`run_ik_test.sh` 机型参数化+playbook 六阶段;yaml `trac_ik→analytic_stationlite` 切换 | 三链 verify 全绿+AB 对拍无精度回归 |

依赖:A1→A2→A3 顺序;B1→B2→B3;**A/B 两线并行自由**。A 线不等 B(trac_ik 顶得住),B 线独立交付。

### 阶段二:真机窗口(A4+A5,需真机在场)

| 批 | 内容 | 验收 |
|---|---|---|
| **A4 真机接入件** | xacro 真机分支(piper 先例)+ 真机控制器 yaml(增益=mcap 档案、限位=URDF effort 口径、signs per-joint)+ SystemInterface 插件(write 层三门防线照抄)+ 上电 SOP(零位确认/使能/板版核对 RID=13) | 低增益小幅 JS 点流冒烟;**挂账实测定案:J2 下限(−0.209→−0.366?)/看门狗 timeout** |
| **A5 验收+基线对拍** | 真机链 verify(JS 点流+CM 切换+安全面:断流停车/超限拒收/NaN 拒收)+ record_verify 真机包 | 跟踪账对拍 mcap 基线(~120mrad 量级)、频率账 ≥400Hz、acceptance 真机登记、**框架包 git diff=0(铁律)** |

## URDF 对拍结论(2026-10-21,厂商 FastTouchV2.SLDASM.urdf vs 本仓 arms/stationlite)

- **运动学结构逐关节一致**:axis(J1-J6 = z/y/y/y/z/x)、origin xyz/rpy(24/117、−265、265/62、78/−2.9、25.5 mm,rpy 全零)毫米级同参——**零位/方向错位风险解除**;motor_signs 的 5 反向是电机侧符号,不涉 URDF 层
- **effort/velocity/上限全一致**(27/27/27/7/12.5/12.5 与 3+3 电机布局互证;J5/J6=12.5 高于 DM4310 手册峰值 7,厂商口径存疑但两侧相同不构成接入差异)
- **唯一出入:J2 下限** 厂商 −0.366(真机软件限位即此值,SDK 从 URDF 直读无二次覆盖)vs 我们 **−0.209**(保守 9°,安全方向)。裁决待做:全工作空间 → 对齐 −0.366;保持保守 → 记档即可
- 右臂与左臂完全对称同参

## 厂商资料地图

- **协议提取文档(本仓自研规格输入):`damiao_protocol.md`**——帧格式/定点换算/限值表/RID 参数表/状态码/保护阈值全提取,三方实证(手册↔SDK↔mcap)
- **达妙官方资料仓**:`https://gitee.com/kit-miao/damiao` → `/home/work/project/damiao/`(2026-10-21 clone)——选型手册 2026-09-23 版 + 全型号手册子模块(4340 V1.3/4310 V1.4 等各版本齐全,含 2D/3D 图纸、性能曲线、测试数据)+ SDK/固件/上位机子模块
- 源码:`/home/work/project/startouchlib`(达妙驱动/解析 IK/动力学前馈/奇异保护;IK 可作交叉真值)
- 手册:`~/下载/达妙电机协议.pdf` = **停产 V1.0,已被官方仓 V1.4 取代**(峰值 7 → V1.2 物理峰值 12.5)
- 回环数据:`~/下载/startouch_joint_cycles_can{0,1}_*.mcap`(JointCycle 全字段)
- 厂商配置:`startouchlib/config/`(FastTouchV2.SLDASM.urdf 395 行 + robot_kinematics.yaml)

## 开放问题(2026-10-21 第二轮盘点后,已大幅收口)

已关闭(详见 damiao_protocol.md):~~DM4340/DM4310 V1.2 手册~~(官方仓)、~~CAN ID 分配~~(正式接口写死:臂 0x01-06/0x11-16+夹爪 0x07/0x17)、~~motor_signs 作用点~~(帧=电机域,signs 归上层配置)、~~使能时序~~(3 重试×2ms)、~~URDF 对拍~~(结构一致,仅 J2 下限差异)、~~接入形态~~(**统一 SocketCAN**,2026-10-21)。

仍开放:
1. **J2 下限对齐裁决**:厂商 −0.366 vs 我们 −0.209(mcap 实证真机在跑厂商值,q_target 到 −0.2256 越过我们限位)
2. 供电规格(24V,双臂 12+2 电机峰值预算)
4. 真机电机板版核对(RID=13 hw_ver,真机在线一行)
5. 零位标定流程(`zero_calibrate.cpp` 本机可读,待提取)
6. mcap response_latency 组装(startouch_sdk 仓,跨时钟域产物不可作时序对齐)
7. **看门狗设计决策**:厂商整机 `set_timeout_all(0)` 禁用了驱动器看门狗(疑 400Hz 抖动误触发);我们方向=保留+调 timeout 值,接入时以抖动实测定
8. IK 交叉真值(analytic_ik,IK 立项时)/ can1 右臂 mcap(非阻塞)

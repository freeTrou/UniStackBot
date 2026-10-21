# unistackbot_protocol — 协议 codec 轴

> 2026-09-24 开包当日两定形：①用户裁决"每个功能包职责单一"——协议独立成包，不放
> interface；②**拆轴**——状态翻译轴分家到姊妹包 `statemachine/`，本包瘦身为**纯
> codec 轴**（字节 ↔ 物理量）。同容器同款组织（父类+子类+工厂）。
> 设计依据：`docs/architecture/real_hardware_architecture.md` §2.1/§2.2/§2.3。

## 职责（单一：codec——字节 ↔ 物理量）

| 内容 | 说明 |
|---|---|
| `ProtocolCore` 父类 | 最小 3 方法：framespec / encode / decode；错误人话化/慢通道编解码为延后项（见架构 §2.3 加法地图）；特殊命令已落首项：`NodeMode::kClear` 清故障 |
| 契约类型 | `NodeCommand`（中立 MIT 五元组+看门狗位+`cmd_raw`[CiA402=controlword, unitree 忽略]）/ `NodeFeedback`（通用物理量+遥测+raw 错误码+`state_raw`[CiA402=statusword, unitree 恒 0]）/ `FrameSpec` / `NodeMode`（kIdle/kRun/kStop/kClear…）——协议词汇零出包 |
| `unitree_im` 子类 | 宇树 IM 系（6010/6014/5010 通用）：宇树字序 CRC32 变体、定点五系数、钳位、非有限输入一票拒绝；ratio 参数化（机器配置注入，不进代码）；**增益先验 `kPriorKp=60.0`/`kPriorKd=0.5`**（2026-10-10 采纳厂商 r1-7a_sdk 例程值, 常量住本包供机型 yaml/状态机显式引用, 内芯不自动填充; kp/kd 负刚度钳 0, 上界=定点 int 界按实际 ratio 推导, 不设厂商 410.7 那种常数钳）；**`kClear` 清故障帧**（mode 6 + tor_raw=-256 魔数内芯强制, 调用方 tau 不参与） |
| 工厂 | `createProtocolCore(name)` + 静态注册表——**加协议 = 注册表加一行**；未知名字 nullptr + `availableProtocolCores()`（无默认纪律） |

## 边界（不放什么）

- **状态翻译/节点安全机** → 姊妹包 `unistackbot_statemachine`（NodeSafety 轴; 本包只供
  统一类型与增益先验常数）
- **总线交换契约**（MasterBase/CmdFrame/StateFrame）→ 姊妹包 `unistackbot_bus`
- **interface 包的 RobotCommand/RobotFeedback**——本包零 ROS 零外部依赖
  （package.xml 无 depend），g++ 直编即测

## 依赖方向

```
unistackbot_protocol (本包: 纯 std, 零依赖)
        ▲
unistackbot_statemachine ← unistackbot_bus ← (骨架子类 + SystemInterface 适配)
```

文件名 = 主类 snake_case：protocol_core.hpp / unitree_im_core.hpp / protocol_factory.cpp。

## 已落地 (2026-09-24, 切片1步1+2)

```
include/unistackbot_protocol/
  protocol_core.hpp    ProtocolCore 父类 + NodeCommand/NodeFeedback/FrameSpec/NodeMode
  unitree_im_core.hpp  UnitreeImCore 子类 + 纯函数 encodeFrame/decodeFrame/crc32
  protocol_factory.hpp 工厂
src/
  unitree_im_core.cpp  实现 (宇树字序 CRC32/定点五系数/钳位)
  protocol_factory.cpp 静态注册表
```

## 测试

**273 cases 全绿**（g++ 直编零 ROS）——厂商 demo 源码作甲骨文（test/oracle_unitree_demo/,
test-only，不进生产代码）：

- encode 方向与厂商 `buildControlPacket` **逐字节比对**（216 组参数——CRC 宇树字序变体
  一锤定音）
- decode 方向自造帧喂厂商解析器互校（厂商用自己的 CRC 校验并接受，全部字段比对）
- CRC 表前 40 项 golden / 非 38/3 减速比复式记账（6010=32 / 6014=12.666 / 5010=16）
- 拒收路径（坏头/坏CRC/坏长度/非法 ratio/非有限输入/广播 ID）/ 钳位 / 工厂+基类指针多态
- **增益语义与先验**（2026-10-10）：kp/kd 负刚度拒收钳 0；先验 60/0.5 的期望字节
  **数学直算**（r=32/16 双 ratio 自证换算），不调厂商代码对拍；上界=定点界按实际
  ratio 推导（厂商 410.7 是 38/3 基准比产物，勿照抄）
- **kClear 清故障帧**（2026-10-10）：mode 6 + tor_raw=-256（LE: f[4]=0x00,f[5]=0xFF），
  看门狗位随 wd 开关（f[2]=0x63/0xE3 @id=3），CRC 全帧重算验证

```bash
cd unistackbot_hardware/protocol/test
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_unitree_im.cpp \
    oracle_unitree_demo/MotorProtocol.cpp ../src/unitree_im_core.cpp ../src/protocol_factory.cpp \
    -I../include -Ioracle_unitree_demo -o /tmp/test_unitree_im && /tmp/test_unitree_im
```

注： 厂商甲骨文头须最先 include（其自带 constexpr M_PI 与 glibc 宏冲突，顺序敏感）。

## dm_motor 内芯（2026-10-21，stationlite 真机接入 A1）

达妙 DM 系（第二个协议内芯）：MIT 帧 + 反馈帧 + 使能/失能/清故障特殊帧，
**一个子类覆盖全型号**（DmMotorType=限值档位，工厂 dm4310/dm4340 两 key 同类）。
范围纪律=只实现用得到+已验证（POS_VEL/参数帧/存储帧暂缓，分档见
arms/stationlite/damiao_protocol.md §11）。域约定：帧=电机域，motor_signs 归上层。
NodeMode 加性词汇 `kEnable`（达妙独立使能帧；unitree 视同 kRun 别名）。

**175 cases 全绿**（g++ 直编零 ROS）——startouchlib codec 作甲骨文
（test/oracle_startouch/，Apache-2.0 提取副本，逐行保持厂商语义）：
encode 逐字节对拍（两型号×零/半程/顶格/负值/越界钳位）/ 反馈帧 decode 互校
（反馈布局 D0=ID|ERR 与控制帧不同，oracle 定点构造）/ 特殊帧字节 FF×7+cmd /
全幅定点直算（零点码字 0x7FFF 非 0x8000）/ 全码表 0/1/3-5/8-E / 拒收 /
工厂多态 / unitree kEnable 别名回归。

```bash
cd unistackbot_hardware/protocol/test
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_dm_motor.cpp \
    oracle_startouch/oracle_startouch.cpp ../src/dm_motor_core.cpp ../src/protocol_factory.cpp \
    ../src/unitree_im_core.cpp -I../include -Ioracle_startouch -o /tmp/test_dm_motor && /tmp/test_dm_motor
```

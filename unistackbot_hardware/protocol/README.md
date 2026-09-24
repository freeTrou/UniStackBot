# unistackbot_protocol — 协议 codec 轴

> 2026-09-24 开包当日两定形：①用户裁决"每个功能包职责单一"——协议独立成包，不放
> interface；②**拆轴**——状态翻译轴分家到姊妹包 `statemachine/`，本包瘦身为**纯
> codec 轴**（字节 ↔ 物理量）。同容器同款组织（父类+子类+工厂）。
> 设计依据：`docs/architecture/real_hardware_architecture.md` §2.1/§2.2/§2.3。

## 职责（单一：codec——字节 ↔ 物理量）

| 内容 | 说明 |
|---|---|
| `ProtocolCore` 父类 | 最小 3 方法：framespec / encode / decode；状态翻译/错误人话化/慢通道编解码均为延后项（见架构 §2.3 加法地图） |
| 契约类型 | `NodeCommand`（中立 MIT 五元组+看门狗位）/ `NodeFeedback`（通用物理量+遥测+raw 错误码）/ `FrameSpec` / `NodeMode`——协议词汇零出包 |
| `unitree_im` 子类 | 宇树 IM 系（6010/6014/5010 通用）：宇树字序 CRC32 变体、定点五系数、钳位、非有限输入一票拒绝；ratio 参数化（机器配置注入，不进代码） |
| 工厂 | `createProtocolCore(name)` + 静态注册表——**加协议 = 注册表加一行**；未知名字 nullptr + `availableProtocolCores()`（无默认纪律） |

## 边界（不放什么）

- **状态翻译** → 姊妹包 `unistackbot_statemachine`（StateTranslator 轴）
- **总线交换契约**（MasterBase/BusCommand/BusState）→ 姊妹包 `unistackbot_bus`
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

**256 cases 全绿**（g++ 直编零 ROS）——厂商 demo 源码作甲骨文（test/oracle_unitree_demo/,
test-only，不进生产代码）：

- encode 方向与厂商 `buildControlPacket` **逐字节比对**（216 组参数——CRC 宇树字序变体
  一锤定音）
- decode 方向自造帧喂厂商解析器互校（厂商用自己的 CRC 校验并接受，全部字段比对）
- CRC 表前 40 项 golden / 非 38/3 减速比复式记账（6010=32 / 6014=12.666 / 5010=16）
- 拒收路径（坏头/坏CRC/坏长度/非法 ratio/非有限输入/广播 ID）/ 钳位 / 工厂+基类指针多态

```bash
cd unistackbot_hardware/protocol/test
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_unitree_im.cpp \
    oracle_unitree_demo/MotorProtocol.cpp ../src/unitree_im_core.cpp ../src/protocol_factory.cpp \
    -I../include -Ioracle_unitree_demo -o /tmp/test_unitree_im && /tmp/test_unitree_im
```

注： 厂商甲骨文头须最先 include（其自带 constexpr M_PI 与 glibc 宏冲突，顺序敏感）。

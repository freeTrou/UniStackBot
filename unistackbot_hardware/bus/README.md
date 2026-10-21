# unistackbot_bus — 总线主站轴

> 2026-09-24 立轴（用户裁决：总线父类与协议/状态机同款组织——父类+子类+工厂，
> 同居 `unistackbot_hardware/` 容器）。当前为接口定形 + 空注册表诚实占位。

## 职责（单一：总线交换契约）

| 内容 | 说明 |
|---|---|
| `MasterBase` 父类 | master.hpp 五要素代码化（9 方法），**交互类别完整**：①周期交换 PDO 形（publish_cmd/take_state，RT 路径）②**慢通道事务 SDO 形**（read_param/write_param——2026-09-24 用户纠偏补类别：协议层六组件"④慢通道 mailbox"的父类投影；非 RT 上下文专用，无能力协议返 kUnsupported）③即发命令（quick_stop / **fault_clear**——2026-10-10 清故障闩锁：全节点请求→泵发协议清帧→error=0 确认→FAULT 粘性解除；无 FAULT 时自熄，新故障须重新请求）+ 生命周期（start/stop）+ 段级粗态（state）——内部泵线程/时隙/transport 零可见 |
| `MasterConfig` | 段装配配置（**statemachine**=安全机工厂名[必填无默认，主站 start 期经工厂自建 per-node 实例——**状态机是总线内部组件**]/endpoint/rate_hz/节点表+每节点 ratio；无默认纪律，非法即 start 拒绝） |
| 交换类型**自持**（`CmdFrame`=vector<NodeCommand> / `StateFrame`=vector<NodeSnapshot>，DynamicSpLatest 装配期定容——2026-10-21 方案 R-a，kMaxNodes 容量常量退役）——不引用 interface 包的 RobotCommand/RobotFeedback（ROS 包），零 ROS 链保持 |
| 工厂 | `createMaster(name)` + **运行期注册制**（注册表无内置条目——各实现包在组合根调 `registerToMasterFactory()` 经 `registerMaster()` 注入；"serial"=SerialMaster 已落地注册，ethercat/canfd 按加法纪律各一行） |

## 依赖方向

```
interface ← protocol ← statemachine ← bus ← (骨架子类 SerialMaster...)
                                      ↑ SystemInterface (ROS 适配层, 唯一碰 ROS 的总线路径)
```

## 测试

g++ 直编（零 ROS）9 cases：抽象性+交互类别存在性 static_assert / 空注册表契约 / 交换类型定长界。

```bash
cd unistackbot_hardware/bus/test
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_master_base.cpp \
    ../src/master_factory.cpp ../../protocol/src/unitree_im_core.cpp ../../protocol/src/dm_motor_core.cpp \
    ../../protocol/src/protocol_factory.cpp ../../statemachine/src/unitree_im_safety.cpp \
    ../../statemachine/src/cia402_safety.cpp ../../statemachine/src/dm_safety.cpp \
    ../../statemachine/src/node_safety_factory.cpp \
    -I../include -I../../protocol/include -I../../statemachine/include \
    -o /tmp/test_master_base && /tmp/test_master_base
```

## 主站稳定性契约 (2026-10-21, "主站写好后几乎不改"定调)

bus 包三区, 改动纪律不同——**每次 bus 包 diff 必须能归入三区之一, 落冻结区需架构账**:

| 区 | 内容 | 纪律 |
|---|---|---|
| **冻结区** | 主站逻辑(泵循环/时隙调度/安全闩锁/SpLatest 交换) | grep 不到协议词汇; 改动需架构评审+全回归 |
| **加法区** | 包类型(CmdFrame/StateFrame/NodeSnapshot/MasterConfig+NodeSpec/ParamStatus/意图接口) | 只加不改语义(传感器数值槽将来走这里) |
| **配置区** | 工厂注册表 | 加一行=新总线类型 |

主站免疫约定: ①协议知识永不进主站(组合模型 per-node 自建); ②**encode 返回 0=本拍无帧, 泵跳过该节点**(传感器槽型分叉免疫, 契约住 protocol_core.hpp); ③慢通道真实现时**参数帧组装严禁在主站内**——进 ProtocolCore 加法(参数帧 codec 接口, 挂起触发器=第一个真实现: 达妙 0x33/0x55 或 CiA402 SDO); ④装配逻辑私有于各主站 start——**提炼共享挂起, 触发器=CanMaster 落地**(最小集纪律: 第二个消费者出现才动; 2026-10-21 曾预提炼 master_assembly.hpp 后按此纪律回归删除); ⑤意图三件套(enable/disable/quick_stop)语义边界: 意图=协商流(状态机守卫+协议确认+请求自清), 急停=原子旗全段立即, publish_cmd.mode=数据流不作使能隐蔽通道。

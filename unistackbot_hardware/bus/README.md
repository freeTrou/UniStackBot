# unistackbot_bus — 总线主站轴

> 2026-09-24 立轴（用户裁决：总线父类与协议/状态机同款组织——父类+子类+工厂，
> 同居 `unistackbot_hardware/` 容器）。当前为接口定形 + 空注册表诚实占位。

## 职责（单一：总线交换契约）

| 内容 | 说明 |
|---|---|
| `MasterBase` 父类 | master.hpp 五要素代码化（9 方法），**交互类别完整**：①周期交换 PDO 形（publish_cmd/take_state，RT 路径）②**慢通道事务 SDO 形**（read_param/write_param——2026-09-24 用户纠偏补类别：协议层六组件"④慢通道 mailbox"的父类投影；非 RT 上下文专用，无能力协议返 kUnsupported）③即发命令（quick_stop / **fault_clear**——2026-10-10 清故障闩锁：全节点请求→泵发协议清帧→error=0 确认→FAULT 粘性解除；无 FAULT 时自熄，新故障须重新请求）+ 生命周期（start/stop）+ 段级粗态（state）——内部泵线程/时隙/transport 零可见 |
| `MasterConfig` | 段装配配置（**statemachine**=安全机工厂名[必填无默认，主站 start 期经工厂自建 per-node 实例——**状态机是总线内部组件**]/endpoint/rate_hz/节点表+每节点 ratio；无默认纪律，非法即 start 拒绝） |
| `BusCommand`/`BusState` | 交换类型**自持**（protocol 包 Node 类型定长数组 kMaxNodes=16）——不引用 interface 包的 RobotCommand/RobotFeedback（ROS 包），零 ROS 链保持 |
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
    ../src/master_factory.cpp -I../include -I../../protocol/include -I../../statemachine/include \
    -o /tmp/test_master_base && /tmp/test_master_base
```

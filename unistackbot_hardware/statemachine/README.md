# unistackbot_statemachine — 节点安全状态机轴

> 2026-09-24 拆轴开包（用户裁决：状态机与协议同款组织、分居两包、同居
> `unistackbot_hardware/` 容器）。设计依据：`docs/architecture/real_hardware_architecture.md`
> §2.1（状态机=翻译到唯一中立机，绝不并集）/§2.2。
> **2026-10-10 重构归位**：StateTranslator（无状态 map_state）与 NodeSafety（有状态
> 具体类）双机并存退役，定形为 **父类中立骨架 + 协议子类 + 工厂**（同 protocol/bus
> 组织）。硬约束（用户裁决）：**状态机是总线内部组件**——主站 start 期经工厂自建
> per-node 实例，住泵 RT 循环（每时隙喂反馈、内联消费命令生成），非上层驱动。

## 职责（单一：节点级安全状态机）

| 内容 | 说明 |
|---|---|
| `NeutralState` | 四态粗机 + OFFLINE（kUnknown/READY/ENABLED/QUICK_STOP/FAULT）——**粗态是框架对上的承诺**；细分格（STANDBY/瞬态终态）与影子机为延后项 |
| `NodeSafety` 父类 | 协议无关骨架：状态词汇、最近反馈缓存、**FAULT 粘性 + 清故障握手**（request → 协议清帧 → confirm）、使能/断使能意图请求、粗态守卫（can_enable/can_clear_fault） |
| `UnitreeImSafety` 子类 | MIT 协议语义：反馈→粗态映射（故障>超时>模式）、使能命令锚定实测位 + **增益先验 kPriorKp/kPriorKd**（协议轴单一事实源）、断使能=停机帧+看门狗位 |
| `Cia402Safety` 子类 | CiA402 (IEC 61800-7-200) 官方七态机：statusword 解码→**原生层一等遥测**（native_state/statusword）→粗态投影；**三步使能链**（0x06→0x07→0x0F，泵每时隙拿链上下一步，序列性住机器内）；断使能=Disable Voltage(0x00)；两层词汇的完整样板 |
| 工厂 | `createNodeSafety(name)` + 静态注册表——加协议安全机 = 加一行；无默认纪律；key 与协议同一把（"unitree_im"/"cia402"），主站从 `MasterConfig.statemachine` 读 |

## 边界（不放什么）

- **codec**（帧编解码）→ 姊妹包 `unistackbot_protocol`（本包依赖它，仅消费
  NodeFeedback/NodeCommand 统一类型与增益先验常数）
- **宿主逻辑**（stale 判 Unknown/worst-of 聚合/升降级策略）→ 总线骨架——快照层的
  陈旧覆盖是信念撤销（不改状态机内部），本包状态机只管协议语义的推进
- **ROS**——零依赖（文件级纯度，g++ 直编可证）

## 依赖方向

```
unistackbot_protocol ← unistackbot_statemachine (本包) ← unistackbot_bus ← 骨架
```

## 测试

**77 cases 全绿**（node_safety 37 + cia402 40；g++ 直编零 ROS；经工厂创建、基类指针驱动——多态即骨架用法）：
工厂（未知名拒绝/清单）/ 四态推进 + FAULT 粘性 / 清故障握手（request→confirm，
error 未清时 confirm 无效）/ 使能守卫与锚定（防跳变：用最新实测位 + 先验增益）/
断使能命令 / 意图请求守卫（OFFLINE/FAULT 拒绝、故障自动清使能请求）。

```bash
cd unistackbot_hardware/statemachine/test
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_node_safety.cpp \
    ../src/unitree_im_safety.cpp ../src/cia402_safety.cpp ../src/node_safety_factory.cpp \
    -I../include -I../../protocol/include -o /tmp/test_safety && /tmp/test_safety
# CiA402 七态/链测试:
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_cia402_safety.cpp \
    ../src/unitree_im_safety.cpp ../src/cia402_safety.cpp ../src/node_safety_factory.cpp \
    -I../include -I../../protocol/include -o /tmp/test_cia402 && /tmp/test_cia402
```

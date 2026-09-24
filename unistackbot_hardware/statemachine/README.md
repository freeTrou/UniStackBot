# unistackbot_statemachine — 状态翻译轴

> 2026-09-24 拆轴开包（用户裁决：状态机与协议同款组织、分居两包、同居
> `unistackbot_hardware/` 容器）。设计依据：`docs/architecture/real_hardware_architecture.md`
> §2.1（状态机=翻译到唯一中立机，绝不并集）/§2.2。

## 职责（单一：统一反馈 → 中立状态）

| 内容 | 说明 |
|---|---|
| `NeutralState` | 四态粗机（kUnknown/READY/ENABLED/QUICK_STOP/FAULT）——**粗态是框架对上的承诺**；细分格（STANDBY/瞬态终态）与影子机为延后项 |
| `StateTranslator` 父类 | `map_state(NodeFeedback) → NeutralState`（观察方向；plan 动作序列延后——等 CiA402 三步使能链） |
| `UnitreeImTranslator` 子类 | unitree 位语义翻译，优先级 **故障 > 超时 > 模式** |
| 工厂 | `createStateTranslator(name)` + 静态注册表——加翻译器 = 加一行；无默认纪律 |

## 边界（不放什么）

- **codec**（帧编解码）→ 姊妹包 `unistackbot_protocol`（本包依赖它，仅消费 NodeFeedback
  统一类型——所有协议 decode 的共同产物，翻译器与具体协议解耦）
- **中立机的宿主逻辑**（stale 检测/worst-of 聚合/升降级策略）→ 总线骨架 health——
  本包只做纯翻译
- 零 ROS 依赖

## 依赖方向

```
unistackbot_protocol ← unistackbot_statemachine (本包) ← unistackbot_bus ← 骨架
```

## 测试

**9 cases 全绿**（g++ 直编零 ROS；直构 NodeFeedback 结构体，与 codec 解耦）：
四态映射 + 优先级（故障>超时>模式）/ 工厂（按名创建/未知拒绝/列表）/ 基类指针多态
（骨架 health 将来就这么用）。

```bash
cd unistackbot_hardware/statemachine/test
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_unitree_im_translator.cpp \
    ../src/unitree_im_translator.cpp ../src/state_translator_factory.cpp \
    -I../include -I../../protocol/include -o /tmp/test_translator && /tmp/test_translator
```

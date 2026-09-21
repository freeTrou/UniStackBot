# stale_watch — 陈旧看门狗 (0c)

值通道消费者的"上游还活着吗"判定原语。配合 `sp_latest` 的 `seq()` 零拷贝轮询:
RT 线程每控制周期 `tick(seq)` 一次, 只数 seq 多少拍没变—— 无 syscall、无时钟、无分配。

## 状态机

```
IDLE ──首条消息──> LIVE ──阈值拍无变化──> STALE
 ^                    ^                      │
 └── reset() ─────────┴──────seq 恢复变化────┘
```

- **IDLE**: 从未见过 seq 变化。无流 = 待命, 不是故障 (控制器激活后等首条命令)。
- **LIVE**: 流活着。seq 变化 = 收到**消息** (不是值变化)——上层重复发同一目标仍算活着。
- **STALE**: 流曾活, 已连续 `stale_cycles` 拍无变化 = 上游断流。

## 契约

- `stale_cycles = 0` → 永不 STALE (功能关闭)。
- ms → cycles 换算用 `get_update_rate()` 同源: `cycles = timeout_ms * hz / 1000`。
- 只在 RT 消费线程使用 (单线程假设, 与 SpLatest 读者契约一致)。
- `reset()` 在 `on_activate` 调用——重激活不继承断流态。
- 消费策略 (断流→受控减速) 住调用方控制器, 本类只做判定。设计依据
  `docs/hardware_framework_design.md` §6.1: "过期 → 保持 + 受控减速, 与命令断流同路径,
  不新增安全机制"。

## 消费者

- `unistackbot_controller` JointStreamController (`~/command` 流, hold 档线性减速 /
  ruckig 档 OTG 刹停)
- `unistackbot_controller` CartesianMotionController (`~/target` 流, 关节空间线性减速 +
  `status.stream_stale` 置位; 默认关——`--once` 单发目标是合法用法)

## 测试

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wconversion test_stale_watch.cpp -o test_stale_watch && ./test_stale_watch
```

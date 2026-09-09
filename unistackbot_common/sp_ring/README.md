# SpscRing · SPSC 无锁环形队列 (事件通道)

> 组件库第二个成员, 自 `unistackbot_sim_control/sim_command_queue.hpp` 迁入并通用化
> (域类型 SimCmdType/SimCommand 留在 sim_control, 泛型队列进本库)。

## 1. 原语家族 (按语义选型, 一名即一语义)

| 组件 | 语义 | 满溢行为 |
|---|---|---|
| SpLatest | 最新 **1** 个 (值通道) | 覆盖 |
| **SpscRing (本组件)** | **逐条必达** (事件通道, 单写单读) | **拒新保旧** (push 返回 false, 显式暴露) |
| mpsc_ring | 多写单读, **丢旧保新** (最新 N 条: 日志/滑动窗口/音视频环) | 逐出最旧, push 恒成功 |

单写场景要丢旧保新时可直接用 mpsc_ring (CAS 税在日志类频率下可忽略)。

容量按"突发深度 + 余量"定, 不靠大容量兜底——队列满 = 速率不变量被破坏 = 病理事件;
`size()` 观测占用, 常驻 > 容量一半 = 消费跟不上的预警线。

## 2. 契约

- SPSC: 单写者/单读者, 由使用架构保证 (如五服务挂同一互斥组 → 串行 → 单生产者)
- `push`: 满返回 false, 重试/丢弃语义归调用方; `pop`: 空返回 false
- 元素: 可平凡拷贝、不含指针 (`static_assert`); 容量: 2 的幂 (`static_assert`)
- 游标单调递增: 无符号自然回绕 + 2 的幂容量, 取模恒正确 —— 32 位平台同样成立
  (回绕一次 ≈ 49.7 天 @1kHz, 运算正确性不受影响, 无需保护)

## 3. 机制

双游标环形缓冲: 推入写 `buffer_[tail % Capacity]` 后 release 发布 `tail`;
`pop` acquire 确认后取 `buffer_[head % Capacity]`, release 推进 `head`。
满/空判定用游标差值, 无符号减法天然处理回绕。

**丢旧模式的并发要点** (与 SpLatest 同源的 seqlock 式自检):
逐出最旧 = 生产者 CAS 前移 `head_` —— `head_` 自此有两个潜在写者(消费者与逐出), 全走原子 RMW;
消费者"拷贝 → acquire fence → 验 head 未动 → CAS 推进", 被驱逐途中的拷贝必然被检出并放弃。
内存序配对关系逐行注释在实现处; 游标 `alignas(64)` 防伪共享。

## 4. 测试 (test_sp_ring.cpp, 规范 42 门槛)

1. **功能**: 空队列 / 满(容量+1 拒) / FIFO 严格序 / 容量 1 边界
2. **并发完整性压测** (核心): 生产者 100 万条(满则忙等重试), 消费者校验严格递增 +1
   —— **不丢、不重、不乱序**; 收尾断言"恰好收满且队列空"(无幽灵元素)
3. 多核 + 目标平台(ARM)同 SpLatest 待办

## 5. 典型用法

### 命令队列 (拒新保旧 = `try_push`)

```cpp
// 生产者 (服务回调, 非 RT): 失败显式透传给请求者
if (!ring.try_push(cmd))
{
	message = "queue full";    // 请求者在场, 由它决定重试或放弃
	return false;
}

// 消费者 (RT 循环): 每拍排空, 不积压
SimCommand cmd;
while (ring.pop(cmd))
{
	apply(cmd);
}
```

### 遥测/日志 (丢旧保新 = `push_overwrite`)

```cpp
ring.push_overwrite(sample);              // 恒成功, 最旧被无声挤出
Entry e;
while (ring.pop(e)) { ship(e); }          // 消费者拿最新 N 条, 按序
```

### 容量定式

`容量 = 突发深度 + 余量`；"不丢"的真正保证是**消费速率 > 生产速率**（稳态），
容量只吸收瞬时突发。健康系统里 `try_push` 的 false 永远不出现、`size()` 常驻远低于容量——
非零/逼近容量即病理预警，不是需要"处理"的正常工况。

## 6. 边界与不适用

| 边界 | 说明 |
|---|---|
| 单写单读 | 多逻辑生产者经互斥组汇聚（标准手法）；真拓扑多写多读等 `mpmc_ring/` 未来组件（Vyukov 每槽 seq 蓝图） |
| 定容 2 的幂 | 不扩容；越界行为由策略表达（拒 / 逐），不是动态缓冲 |
| 全接口无阻塞 | 零等待零睡眠——RT 热路径安全；需要"等空位"的场景请用 OS 队列（信号量/msgq） |
| 元素 POD | 含指针/非平凡类型 `static_assert` 拒收 |
| 不持久化 | 拒（满）与逐（旧）都不落盘——要"绝对不丢"请落盘/落网，内存环给不了这个承诺 |
| 丢旧模式 pop 重试 | 极端无间隔连发逐出下消费者可能连续重试，终止由生产者工作总量有限保证 |
| 32 位游标回绕 | ~49.7 天 @1kHz 回绕一次，无符号回绕运算天然正确（游标无特殊值语义），无需保护 |
| 一实例一推法 | `try_push` 与 `push_overwrite` 不混用（内存安全，语义不可预测） |
| 与 SpLatest 分工 | 只要最新 1 个 → SpLatest；逐条必达或最新 N 条 → 本组件 |

## 7. 迁移记录

- 泛型 `SpscRing<T, Capacity>` ← `sim_command_queue.hpp`; 新增元素级
  `static_assert(is_trivially_copyable)` (原先在 SimCommand 上, 现下沉到队列本体)
- `kMaxJoints` 抽至 `unistackbot_common/contract.hpp` (全框架单一事实源, sim_control 经
  using-declaration 保持原用法)
- 域类型 `SimCmdType`/`SimCommand`/`kQueueCapacity` 留在 sim_control (域语义不进通用库)

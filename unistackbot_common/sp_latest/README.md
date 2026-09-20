# SpLatest · 双缓冲覆盖写/取最新原语

> 组件库第一个成员。名字沿自 `docs/linux_rt_guide.md` §2.5 与 `docs/hardware_framework_design.md`
> 中"交换原语(覆盖写+取最新)"的既有命名,与 `SpscRing` 同族(两者将来同居本库)。
> 状态:**设计定稿,待实现**。

## 1. 定位

线程间"最新值"交换原语——写者永远不被读者阻塞地覆盖写,读者拿到最近一次完整快照,
序号(seq)随数据走以支撑陈旧检测看门狗。

| 是 | 不是 |
|---|---|
| SPSC(单写单读) 最新值语义 | FIFO 队列(那是 SpscRing) |
| 覆盖写:写者永不等读者 | 丢失检测:中间值被覆盖是**语义**不是 bug |
| POD 专用、零依赖、纯 C++17 头文件 | ROS 组件、动态内存、异常流 |

典型落点:主站线程 state_ex → controller_manager、宿主 RT ↔ plant 线程快照、
任何"控制参考值/状态快照"的下发与回传。

## 2. 语义契约

- **覆盖写**:写者 `publish()` 零等待(wait-free,三次原子操作),读者慢/不在也无碍
- **取最新**:读者 `read()` **单遍直线代码**(WCET = 一次拷贝,无重试循环)——
  检测到写入中或撕裂直接返回 false、out 保持旧值,调用方沿用旧值即标准降级;
  读者永不阻塞、永不拿到半份新半份旧的数据
- **seq**:每次 publish 自增;读者可零拷贝轮询 `seq()` 做**陈旧看门狗**
  (N 拍不变 = 上游停发,主站设计"seq 看门狗"的数据源)
- **单写者**:由使用架构保证(如"五服务挂同一互斥组"),注释声明,无运行期检查
- **POD 契约**:`static_assert(is_trivially_copyable)`,禁指针成员(跨线程裸传的安全边界)

## 3. API

```cpp
template <typename T>
class SpLatest
{
public:
	SpLatest();                             // 构造为"从未发布"零态 (seq=0, 不做任何事)
	void init(const T & initial);           // 装配期带初值 (seq=2 起算; 必须先于并发访问)

	void publish(const T & data);                       // 写者(唯一): 覆盖写
	bool read(T & out, uint64_t & seq_out) const;       // 读者(唯一): 取最新快照
	uint64_t seq() const;                               // 零拷贝看门狗轮询
};
```

生命周期: 构造(零态) → [init(带初值, 装配期)] → publish/read。装配期 = 单线程、
无并发访问的阶段(与宿主 on_init/on_configure 同期), init 不得与活跃线程并发调用。

`read` 返回值:总是处理结果——true=拿到 seq_out 对应的完整快照;
false=尚无数据(seq==0)或撕裂重试耗尽(设计上几乎不可达),out 保持原值。
错误走返回值(规范 27),无异常。

## 4. 内部机制:双缓冲 + seq 宣告(seqlock 式)

```
写者 publish:                          读者 read:
  target = 1 - current_                  s0 = seq_ (acquire)
  seq_.fetch_add(1,acq_rel) → 奇数(宣告)   0 或奇数? → false(沿用旧值)
                                         idx = current_ (acquire)
  slots_[target] = data                  tmp = slots_[idx]
  current_.store(target, release)        [seq_cst 全屏障]
  seq_.fetch_add(1, release) → 偶数(完成)  s1 = seq_ (acquire)
                                         s0 == s1 ? 成功 : false(撕裂, 沿用旧值)
```

seq 语义:0 = 从未发布,奇 = 写入中(瞬态),偶 = 稳定可读,单调递增。
**uint64 裁决**:1kHz 下 ~2.9 亿年才绕回,无绕回保护代码;`static_assert(is_always_lock_free)`
把"64 位原子无锁"钉死在编译期(目标平台 x86_64/AArch64 原生支持,拿到 32 位平台直接编译失败)。
**宣告必须先于任何数据字节**——这是压测换来的教训(踩坑记录在头文件 publish 处):
初版"写槽→翻槽→seq++"在写槽进行中 seq 不动,读者判稳却拷到半新半旧。
两处屏障 (宣告的 acq_rel + 读侧 seq_cst 全屏障) 保弱序架构。

**评审修复记录 (第七轮外部评审)**:
- **写者宣告 fence 方向错误 (真缺陷, x86 TSO 掩盖 / ARM 理论撕裂)**: 原 relaxed fetch_add +
  release fence 不约束**后续写槽上移** —— 危害路径比"当次写槽前移"更深一层: **下一次 publish
  的写槽可上移穿过本次宣告序列**, 与"读旧 current 槽"的读者同槽交叠, 撕裂数据通过 s0==s1 校验。
  修复: 第一次 fetch_add 改 acq_rel (acquire 侧挡后续上移), 删多余 fence。
- **读侧 fence 同款单向洞 (与 MpscRing 第六轮同构)**: acquire fence 不挡 tmp 拷贝下移过
  s1 校验 → 升级 seq_cst 全屏障 (双向)。
- 32 位绕回保护、初始化契约、tears/seq 观测: 评审确认无误。

- **为什么几乎零重试**:读者拷贝期间写者写的是**另一槽**;只有写者在一次拷贝窗口内
  连发两拍才会撕裂(写回读者正在拷的槽)——1kHz 写者 + ~百 ns 拷贝,概率 ≈ 0,
  重试上限是纯安全网;撕裂计数器对外可查(它本身就是有价值观测)
- **为什么不是纯 seqlock(单缓冲)**:单缓冲读者撞上写者就重试,重试率高一档;
  双缓冲把这个概率压到统计零,代价是一个 T 大小的槽位
- **为什么不是三缓冲**:三缓冲读者写者都 wait-free,但空闲槽交接复杂;当前规模
  (POD ≤ 数百 B、kHz 级)用不上,留作将来升级路径
- 实测注记:写者**无间隔全速轰击**(远超真实 1kHz)时 miss 率会很高(写者常年处于
  "写入中"),这是工况极端所致;契约下 miss = 沿用旧值,真实频率比下趋近于零

## 5. 边界与已知取舍

- **单遍无重试**(裁决):WCET = 一次拷贝,最坏情况界最简;碰撞在真实速率下 ≈0.03-0.1%/拍,
  miss 沿用旧值对控制回路的代价低于隐藏的 4× WCET——按"实时 = 有界可预测"取舍
- 撕裂/写入中返回 false:调用方沿用旧值即可——对控制回路"用上一拍数据"是标准降级
- **seq 位宽自适应**(评审三轮演进:uint32+补丁 → uint64 → 32 位兼容):64 位平台用 uint64
  无绕回(~2.9 亿年);32 位平台 `if constexpr` 编译期退 uint32 并启用跳 0 保护——两分支均为
  无锁原子(`static_assert` 钉死,拿到异构平台编译失败好过静默退化为锁)。对外 API 恒为 uint64
  (32 位平台零扩展,但 seq 值本身 ~24.9 天绕回——32 位属遗留场景,真机矩阵 x86_64/AArch64 全 64 位)
- `seq==0` = 从未发布,read 返回 false;需要初值语义用带初值构造
- T 尺寸无上限约束,但建议 ≤ 数百 B(拷贝在读者线程内,尺寸×频率要算账)
- ts 不进原语:调用方把 `uint64_t stamp` 放进 T——原语管 seq(递增等价于"新"),
  业务管 ts(物理时间),职责分离

## 6. 测试方案(交付门槛,规范 42)

1. **功能**:单线程发布/读取往返;seq 单调;首拍语义(未发布 read=false / 带初值 true)
2. **撕裂压测**(极端工况):写者无间隔全速轰击,读者连续 read 断言"快照内部一致"——
   千万次零脏数据;预期 miss/tears 高企(写者常年"写入中"),它们是安全边界不是预期值
3. **双通道全双工**(真实工况):**两个 SpLatest**(cmd_ex 控制下行 + state_ex 反馈上行)
   模拟主站↔CM,**两侧等速 1kHz**,144B 大载荷(16 关节),主站回显最近命令做**闭环完整性**
   校验——预期 miss≈0、tears=0、每条命令走完全程。x86 实测:3000 拍,tears=0/0,miss 1/6000
4. **多核跑**:x86 开发机 + 目标平台(ARM Orin/RK3576)各跑一遍(内存序的正确性在弱序架构上才算验完)

## 7. 落位

```
unistackbot_common/sp_latest/
├── README.md        # 本文档
├── sp_latest.hpp    # 实现 (header-only, 模板)
└── test_sp_latest.cpp  # 功能 + 压测 (独立可编译, 不依赖 ROS)
```

## 8. 调用方契约(使用前必读)

### 红线(违反即破坏正确性,编译器不管)

1. **`publish()` 只准一个线程调**(SPSC 之 S)——用架构机制锁死(如服务挂同一互斥组),不靠自觉
2. **同一实例只给一个消费方**
3. **构造 + `init()` 严格先于线程启动;运行期禁止 re-init**
4. **只装"值"不装"事件"**——每条必达的指令用 SpscRing;跳过中间值是覆盖写的语义,不是 bug

### 责任划分(关键认知)

| 层 | 内容 |
|---|---|
| **原语不骗你** | 红线 1-4 守住 → 永无脏数据、永不阻塞、seq 永不失真——数据完整性的充分条件 |
| **你不被沉默骗到** | **上游死亡时 `read()` 仍返回 true**(读的是同一份旧快照)——活体检测 SpLatest 故意不做(职责单一),义务 100% 在调用方 |

### 调用方标准模式(最新值 + 两阶段看门狗)

```cpp
StateEx state{};          // 初值必须是安全值(如 kp=0/enable=false)——首帧前可能被当真数据用
uint64_t last_seq = 0;    // == 0 表示"从未见过数据" = 启动期
uint32_t beats = 0;

void on_cycle()
{
	uint64_t seq = 0;
	const bool got = state_ex.read(state, seq);   // false 时 state 自动保持旧值, 无需分支

	if (last_seq == 0)
	{
		// 启动期: 等首帧, 宽限秒级; 首帧未到绝不进入控制
		if (got) { last_seq = seq; beats = 0; }
		else if (++beats > kStartupLimit) { error("上游未启动"); }
		return;
	}
	// 运行期: 主信号是 seq 冻结(不是 miss), 阈值毫秒级
	beats = (got && seq != last_seq) ? 0 : beats + 1;
	if (beats > kStaleLimit) { enter_safe_state(); }
}
```

### init 占位帧识别 (2026-09-20 补充契约)

`init(初值)` 的初值**不一定是实际反馈**——真机总线上线前是占位值, 仿真后端 init 即真值。
而 init 后读者首次 `read` 会**成功** (seq=2 稳定), 拿到占位数据无从由返回值分辨。
判据由 seq 提供 (组件公开常量 `kInitFrameSeq = 2`):

```
seq_out == 2 (kInitFrameSeq)  → init 占位帧: 真机调用方忽略, 不进控制 (上线前 CM 不动作)
seq_out >= 4                  → 至少经过一次真实 publish, 可用
```

**推荐读取姿势 (2026-09-20 终稿)**——`readLastFrame`: **要么最新帧, 要么上一帧**:
重试成功 → 最新帧; 重试耗尽 (kNone) → out 保持既有值 = 上一帧 (载体是 out 参数本身,
组件零缓存零旧槽推理)。姿势 = 持久接收变量 + 一个 live 历史 (防 init 残留):

```cpp
Feedback fb;  bool live = false;   // 循环外持久
switch (ch.readLastFrame(fb, seq)) {
  case FrameKind::kLive: live = true; /* 用 fb (最新) */ break;
  case FrameKind::kInit: live = false; break;            // 总线未上线占位 → 忽略
  case FrameKind::kNone: if (live) { /* 用 fb (上一帧) */ } break;
}
```

**live 布尔是必需的, 不是可选的** (第五轮外部评审钉死): 没有它, 首次 kNone (从未成功过)
时 fb 是调用方初始化值、kInit 残留时是占位值——两种情况直接用 fb = 把非反馈数据喂进控制,
且编译器/运行时都无报警。照抄清单: 持久接收变量 ✓ + live 门禁 ✓, 缺一不可。

**大载荷物理约束与终测结论 (2026-09-20, 4096B 级)**: read 成功率 ≈ 1 − 拷贝时长/写者间隔。
终测 (双向 1kHz 真实频率, 4096B, 3 次重试): **10019 读 / kNone=0 / tears=19 / 零脏数据** —
大载荷在真实频率域 100% 命中; 全速背靠背写者 (超设计域 100 倍压力) 下 kNone 80% 但**零脏**
(fat-stress 用例, 正确降级到上一帧语义)。报文尺寸与写频率的账: 1kHz 下 4KB 拷贝 1.5µs
仅占间隔 0.15%, 3 次重试后 kNone 概率 ~10⁻⁹/拍, 实测 0。

`read` (严格: 要么本拍最新要么 false) 保留给确需本拍新鲜度判定的特殊场景。
bench 两个消费点 (bus 读 cmd / CM 读 fb) 均为统一姿势的活样例;
CM 忽略占位 fb 则本拍不产 cmd, 总线读 cmd 同样拦占位 — 双向防护, 上线前整链静止。

### 补充规则

- **双向交换 = 两实例**:控制(`cmd_ex`)与反馈(`state_ex`)各持一只,方向相反、看门狗独立
- **看门狗阈值不对称**:反馈断了宽(20-50ms,控制质量退化);命令断了严(5-10ms 即安全态,安全事件)
- **启动顺序任意**:读者先起 = miss 到首拍;写者先起 = 数据候着;唯一硬规则 = 红线 3
- **载荷尺寸算账**:拷贝发生在读者的 RT 线程内,`尺寸 × 频率` 即 RT 预算占用;大块数据只放索引,
  内容走别的通道

## 链路基准: bench_sp_latest_chain.cpp (2026-09-20)

双核双线程 (核1 CM FIFO80 / 核2 总线 FIFO90) read→update→write 闭环模拟, 两条独立 SpLatest 值通道:
`Feedback`(反馈, 32 关节位置/速度/力矩/错误码/母线电压) + `Command`(控制, 位置命令+速度/力矩前馈+模式/使能)。
**报文整型全部定宽** (int64_t/uint8_t, 时间戳 int64_t ns)。CM 相位参数化 (0=同时启动 / 随机=模拟框架激活时序)。

```bash
cd unistackbot_common/sp_latest
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I../rt_tune bench_sp_latest_chain.cpp -o /tmp/bench_chain -pthread
/tmp/bench_chain <bus_period_us> <cm_period_us> <duration_s> [cm_phase_us=-1随机]
```

**终版: 裸通道 + 总线拍内原生顺序, 零同步补丁** (裁决史: readWait 自旋 / seq 等新帧 / 撕裂率监控 / 移相控制律均试作后否决——为 1% 概率的 2% 代价上机制是过度设计):

1. **总线线程 = IgH 原生拍内顺序** (拍首 send → 总线往返 ~100µs → receive → publish fb → read cmd 供下拍)——通道操作天然落在拍首+往返处, 不是补丁, 是主站循环本来的样子;
2. **CM 侧裸 read**: 撞窗 → read false → 沿用旧值一拍, 实测最坏同相 9.6% miss 下 errRMS 与零 miss 差 2% — **晚一帧不重要**;
3. **上游死 = seq 冻结 = StaleWatch 域** (安全语义, 与撞窗无关)。

**三层概率账**: 相位差是启动时一次性决定的常量 (同频不漂); 撞窗带仅 ±100µs±2µs 两处窄缝 (500Hz ~0.4% / 1kHz ~0.8% / 2kHz ~1.6%), 而**同时启动 φ=0 恰是安全相位** (实测零撕裂); 真撞上 = errRMS +2% 无感, 重激活 CM 即重掷。概率低 × 后果小 × 可重掷 → 任何同步机制失去存在理由。回头加等待机制的判据: 实测 errRMS/相位裕度退化 (2kHz+ 力控高带宽场景)。

实测 (极简终版, 30-60s):

| 轮 | 相位 | 撕裂 fb/cmd | miss | errRMS (rad) | bus/cm wake p99 (µs) |
|---|---|---|---|---|---|
| 500Hz ×30s | 同相 0 | 0 / 0 | 0 | 0.1599 | 10.1 / 11.6 |
| 1kHz ×30s | 同相 0 | 0 / 0 | 0 | 0.0870 | 9.6 / 10.5 |
| 2kHz ×30s | 同相 0 | 0 / 0 | 0 | 0.0445 | 9.1 / 10.0 |
| 1kHz ×60s | 随机 497µs | 0 / 0 | 0 | 0.0881 | 8.4 / 10.0 |
| 2kHz ×30s (终版) | 随机 **104µs=真实撞上撞窗带** | 0 / 4 | 15 (0.025%) | 0.0454 (+2%) | 8.4 / 9.7 |

(errRMS 随频率降低增大 = 执行器一阶模型每拍修正 2.5% 的控制带宽账, 与通道无关。)

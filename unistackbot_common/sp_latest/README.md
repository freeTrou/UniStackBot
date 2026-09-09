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

### 补充规则

- **双向交换 = 两实例**:控制(`cmd_ex`)与反馈(`state_ex`)各持一只,方向相反、看门狗独立
- **看门狗阈值不对称**:反馈断了宽(20-50ms,控制质量退化);命令断了严(5-10ms 即安全态,安全事件)
- **启动顺序任意**:读者先起 = miss 到首拍;写者先起 = 数据候着;唯一硬规则 = 红线 3
- **载荷尺寸算账**:拷贝发生在读者的 RT 线程内,`尺寸 × 频率` 即 RT 预算占用;大块数据只放索引,
  内容走别的通道

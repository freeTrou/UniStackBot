# ruckig · 流式 OTG (vendored) + 防御封装

> **来源**: [pantor/ruckig](https://github.com/pantor/ruckig) v0.14.0 (MIT), vendor 于 2026-09-18。
> **不是单头文件**: `include/` + `src/`(13 个 .cpp) 需参与编译。零第三方依赖, C++17。
> **本组件 = 上游源码 + `otg_stream.hpp` 防御封装 + 一处 RT 补丁**。

## 定位

点流整形器 (时间轴工具): 每拍喂"当前状态 + 最新目标 + (v/a/j) 极限" → 输出本拍参考位。
解决**率失配** (慢上层 10-100Hz vs 500Hz 控制) 与**运动整形** (速度/加速度/加加速度全受限)。
**不做路径插值** (形状轴归上层: MoveIt/computeCartesianPath)。

## OtgStream 防御封装 (推荐入口, 不直接用裸 Ruckig)

社区版已知数值风险 (评审清单): `-101/-110/-111` 计算失败、零目标态不稳定、野输入放大。
封装的防御契约 —— `update()` 只有两种结局:

- `Ok`: 输出本拍参考位 (过全部校验)
- `Hold`: **保持上一拍安全位** (不抛不崩), 错误可观测 (`errorCount/lastError`)

防御链: 输入 NaN/Inf 拒绝 → 目标跳变限幅 (`max_target_jump`, 软限防病态跳变) → Ruckig
返回码检查 (Error* 保持) → 输出 isfinite 全查 → 错误自愈 (`otg.reset` + 状态回退保持位
静止, 下拍重新规划)。init 校验限值 (正/有限/<1e9)。

```cpp
unistackbot_common::OtgStream<7> otg;
otg.init(0.002, limits, /*max_target_jump=*/0.5);
otg.reset(current_q);                       // 激活/重激活时
auto r = otg.update(target_q, out_q);       // 每拍; Hold = 保持
```

## RT 补丁 (vendored 修改, 版本前进时重查)

`trajectory.hpp` 的 `state_to_integrate_from`: 原版 `SetIntegrate = std::function` 每次
`at_time` 堆分配 32B (lambda 捕获集超 SBO) → 已模板化消除 (标记 `[UniStackBot RT 补丁]`)。
补丁后实测: 稳态 update **零 malloc**, p50 0.2µs / max ~14µs @7-DOF。

## 测试

```bash
cd unistackbot_common/ruckig
g++ -std=c++17 -O2 -pthread -Wall -Wextra -I include test_ruckig.cpp src/ruckig/*.cpp -o /tmp/test_ruckig && /tmp/test_ruckig
# 库级: 单轴/7轴/率失配重定向/零malloc/计时 (12 断言)
g++ -std=c++17 -O2 -pthread -Wall -Wextra -I include test_otg_stream.cpp src/ruckig/*.cpp -o /tmp/test_otg && /tmp/test_otg_stream
# 封装级: NaN拒绝/跳变限幅/错误自愈/init校验/观测 (22 断言)
```

**测试教训 (防复发)**: ① 封装首版漏了"目标摄入"—— Ruckig 一直追旧目标、恒返 Ok 不动,
被 T8 位置断言当场抓住 (返回码全对 ≠ 在追你的目标); ② 断言的时序裕量按时间最优时长给
(拍数 = 时长/dt × 1.5)。

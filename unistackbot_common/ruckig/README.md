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

## CartesianShaper 笛卡尔位姿流整形器 (2026-09-23, 组合 OtgStream)

社区版 Ruckig 无 SE(3) 插值 (Pro 功能)。本类用两个 OtgStream<3> 组合出笛卡尔整形,
**零新数学** (Ruckig 管平滑, Eigen 管旋转):

- **位置通道**: OtgStream<3> 直接整形 (x, y, z)
- **姿态通道**: 锚点切空间整形 —— `r = log(anchor⁻¹·q)` 映射到 R³ 线性空间后与位置
  通道完全同构; anchor = reset 时姿态 (激活内恒定); 恒定/已到位姿态 r=0 直通

```cpp
unistackbot_common::CartesianShaper s;
CartesianShaper::Limits lim;   // 逐轴语义 (同关节 OTG): 合成速度可达 √3·max_velocity
s.init(0.002, lim);
s.reset(current_pose);                        // 激活/重激活; anchor 在此刻定格
auto r = s.update(target_pose, out_pose);     // 每拍; Hold = 保持 (契约同 OtgStream)
```

契约/观测面/错误码与 OtgStream 完全同款 (Ok/Hold 两结局、Hold 输出恒有效、
updateCount/errorCount/lastError)。**边界**: ① 目标姿态与 anchor 夹角近 π 时
rotation vector 方向退化 (对跖点), 大范围重定向先 reset 换锚; ② 姿态噪声流上层先滤。

**设计教训 (v1 否决案存档)**: "每拍 reset 注入实测剩余角" 的闭环方案不可行 ——
OtgStream `reset()` 强制下拍重算且首个 update 输出 t=0 状态, 每拍 reset = 推进/停滞
交替、速度永远建不起来 (实测 d_theta 交替 6.7e-8/0)。流式整形必须保持底座状态
延续, 切空间正是为此。

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
g++ -std=c++17 -O2 -pthread -Wall -Wextra -I include test_ruckig_stress.cpp src/ruckig/*.cpp -o /tmp/test_stress && /tmp/test_stress
# 压测: 攻击社区版已知数值失败模式 (2026-09-18, 12 断言)
g++ -std=c++17 -O2 -Wall -Wextra -I.. -Iinclude -I/usr/include/eigen3 \
    test_cartesian_shaper.cpp src/ruckig/*.cpp -o /tmp/test_cs && /tmp/test_cs
# CartesianShaper: 双通道到位/直通/消毒/重定向逐轴限/大转角/观测 (2026-09-23, 9 组)
```

**压测结论 (2026-09-18, 5 场景)**: S2 极端跳变 (限幅关闭放野输入) **实测触发 Ruckig 原生
-110 (ErrorExecutionTimeCalculation)** —— 防御层捕获/保持/自愈全链路生效, 证明对"真错误"
有效而非仅合成用例; S4 RL 野流 60s (30000 拍 ±40rad 随机) 零错误零越界 (跳变限幅生效);
S1 零目标态/S3 大数边界/S5 每拍重定向输出恒有限。**测试教训 (防复发)**: ① 封装首版漏了"目标摄入"—— Ruckig 一直追旧目标、恒返 Ok 不动,
被 T8 位置断言当场抓住 (返回码全对 ≠ 在追你的目标); ② 断言的时序裕量按时间最优时长给
(拍数 = 时长/dt × 1.5)。

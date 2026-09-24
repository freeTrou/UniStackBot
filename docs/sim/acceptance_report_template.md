# <robot> 机型验收报告 (<YYYY-MM-DD>)

> 手动首跑版模板；run_acceptance.sh 就位后由引擎自动填写同构报告。

## 环境快照

| 项 | 值 |
|---|---|
| 内核 / cmdline 关键项 | |
| commit | |
| isolcpus / rt_tune_boot / lo multicast | 已设 / 未设 |

## 阶段结果（A-G，对应 sim_validation_pipeline.md §2）

| 阶段 | 结果 | 关键数据 | 证据（日志/文件路径） |
|---|---|---|---|
| A 模型接入（静态指纹 + 可视化目检） | | 形态指纹 ②③ 全过 | |
| B mock 逻辑（verify §3 + fault F1-F6 + smoke） | | PASS=N FAIL=0 | |
| C mujoco 物理（verify 段 + fault --chain mujoco） | | PASS=N FAIL=0 | |
| D gz 集成（verify --with-gazebo 段） | | PASS=N / 桥语义过 | |
| E RT 基准（mock / mujoco） | | p99 / max / 误差，vs 基线无回归 | results/rt_<标签>.md |
| F 数据与场景（demo 录制 + 曲线人检） | | 曲线无异常 | bag 路径 |
| G 汇总归档 | | 本文档 | |

## 结论

- 验收门：G1 冒烟 ✓/✗ · G2 物理 ✓/✗ ·（G3-G6 未开按 §5.5 现状标注）
- 遗留 / 例外说明：

签名：____________

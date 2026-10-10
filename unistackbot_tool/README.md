# unistackbot_tool · 工具程序容器包

> 2026-10-10 立项（用户裁决）: 工程工具程序统一落点。首件 = **数据录制规范 v1** 的实现
> 工具; 后续工具（标定辅助/诊断/批量分析）都进本包。纯脚本零编译, 链路契约的独立消费者。

## 数据录制规范 v1

### 0. 动机（2026-10-09 走廊排障的账）

仿真与真机验证都需要录**关节电机 + 传感器**数据。无规范的代价（当日实锤）:
采样节奏未知只能猜、点序靠命名约定被字典序搅乱（`_100` 排 `_10` 前）、采集贴限位
（j2 压 -0.209 线 0.014°）无标注被当异常排查。规范 v1 一次定死这些。

### 1. 两种数据资产

| 资产 | 载体 | 用途 | 工具 |
|---|---|---|---|
| **连续记录** | mcap (rosbag2) | 时域验证: 跟踪误差/频响/超调, sim↔real 对拍 | `record_verify.sh` |
| **点位资产** | json v1 | 空间轨迹: 回放验证/遥操采样/示教点位库 | `convert_points_v0_v1.py` → `demo_points_playback.py` |

### 2. 连续记录规范

**必录话题**（按角色, 链型无关）:

| 角色 | 话题 | 说明 |
|---|---|---|
| 关节状态 | `/joint_states` | 传感器侧: position/velocity/effort (JSB 发布) |
| 关节命令（上层意图） | `/joint_stream_controller/command` | 上层发的目标点流 |
| 关节命令（实际执行） | `/joint_stream_controller/applied_command` | 插值后全率实际下发 —— **与状态的差 = 跟踪账** |
| 时钟 | `/clock` | 仿真链必须（分析以 header.stamp 为准, 墙钟仅作参考） |

**选录**（按需 `--extra`）: `/ee_state_broadcaster/ee_state`（笛卡尔验证）·
`/cartesian_motion_controller/status`（CM 域）· `/tf`。

**目录与命名**: `<robot>_<chain>_<task>_<YYYYmmdd_HHMMSS>/`
内含 `rosbag2_*.mcap` + `metadata.yaml`（本工具自动生成 sidecar）。

**metadata.yaml 必填**: robot / chain / task / bus_hz / operator / notes /
控制器快照（`ros2 control list_controllers` 输出）。**没有 sidecar 的 mcap 视为
废数据**——三周后没人记得它的采样率。

**纪律**:
- 一段记录一个 task, 不混任务（对拍分母要干净）
- 真机记录前先 `ros2 control list_controllers` 确认控制器态并进 sidecar
- 分析一律用 `header.stamp`（sim 时间）, 不用接收时刻

### 3. 点位资产 json v1

```json
{
  "meta": {
    "format": 1, "robot": "stationlite", "side": "left",
    "joint_names": ["left_joint1", "..6"],
    "hz": 20, "created": "2026-10-10T..", "operator": "..",
    "source": "armtask points.json v0",
    "notes": "transit_l 为回程值 (与去程差 ~11° j3, 数据源已知瑕疵)",
    "limit_margin": {"left_joint2": "77 点贴 -0.209 下限, 噪声 0.014° 级"}
  },
  "points": { "left": { "transit_l": [q1..q6], "cup_drop": [..], ... } },
  "sequences": {
    "corridor_l": ["transit_l", "to_cupdrop_000", "...", "cup_drop",
                    "from_cupdrop_000", "...", "from_cupdrop_126"]
  }
}
```

**v1 硬规则**（v0 的病都治在这）:
1. **序列显式有序**——回放顺序 = `sequences` 数组序, 与点名后缀**无关**
   （命名约定排序是字典序事故的根, 2026-10-09 实锤）
2. **meta 必填**——hz/关节序/来源/瑕疵备注; 限位贴线必须记 `limit_margin`
3. `points` 保留 `{side: {点名: q}}` 结构, v0 文件不转换也可直接喂回放工具
   （回放工具 v0/v1 双兼容; v0 无 sequences 时 `--seq` 用自然排序展开）

### 4. 标准流程

```bash
# 连续记录 (起链后):
ros2 run unistackbot_tool record_verify.sh <task> [--robot x] [--chain y] \
    [--extra /ee_state_broadcaster/ee_state] [--duration 60]
# → ./<robot>_<chain>_<task>_<时间戳>/  (Ctrl-C 或 --duration 收尾)

# 点位资产: v0 → v1
ros2 run unistackbot_tool convert_points_v0_v1.py <in.json> <out.json> \
    [--robot stationlite] [--side left] [--hz 20]

# 回放验证 (消费):
ros2 run unistackbot_demo demo_points_playback.py --points-file <v1.json> \
    --side left --seq "@corridor_l" --hz 20
```

## 工具清单

| 工具 | 语言 | 说明 |
|---|---|---|
| `record_verify.sh` | bash | 一键录标准话题集 + metadata sidecar |
| `convert_points_v0_v1.py` | py | v0 点位 → v1 (补 meta + 自然序 sequences) |

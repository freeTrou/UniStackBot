# unistackbot_tool · 工具程序容器包

> 2026-10-10 立项（用户裁决）: 工程工具程序统一落点。首件 = **数据录制规范 v1.1**
> 的实现工具; 后续工具（标定辅助/诊断/批量分析）都进本包。纯脚本零编译, 链路契约的独立消费者。

## 数据录制规范 v1.1

> v1.1 (2026-10-12): 必录集 +4 (笛卡尔域意图/分段 + 错误板 + EE 真值);
> sidecar +sim_time 字段。话题名同步当日接口定名 (CM target→command, control→pause)。

### 0. 动机（2026-10-09 走廊排障的账）

仿真与真机验证都需要录**关节电机 + 传感器**数据。无规范的代价（当日实锤）:
采样节奏未知只能猜、点序靠命名约定被字典序搅乱（`_100` 排 `_10` 前）、采集贴限位
（j2 压 -0.209 线 0.014°）无标注被当异常排查。规范 v1 一次定死这些。

### 1. 两种数据资产

| 资产 | 载体 | 用途 | 工具 |
|---|---|---|---|
| **连续记录** | mcap (rosbag2) | 时域验证: 跟踪误差/频响/超调, sim↔real 对拍 | `record_verify.sh` |
| **点位资产** | json v1 | 空间轨迹: 回放验证/遥操采样/示教点位库 | `convert_points_v0_v1.py` → `demo_points_playback.py` |
| **effort 质量筛查** | mcap (复用连续记录) | 伺服病理段检测: 翻转率×速度位移失配双判据, 训练数据消费前过筛 | `check_effort_flips.py` |

**effort 质量筛查** (`check_effort_flips.py`, 2026-10-20): MuJoCo 位置伺服在最小惯量构型可进继电器式极限环——effort 满幅高频换向、位置看似静止。此类段落的 effort 观测是仿真病理非物理。双判据 (窗峰翻转率 + 速度-位移失配比) 好/病包双向校准; 高翻转但速度位移一致 = 物理换向, 显式注明。退出码 0 干净 / 1 检出病理, 可作数据管线的 CI 门。病理档案: `arms/stationlite/mujoco/README.md` #7。

### 2. 连续记录规范

**格式裁决 (2026-10-10 用户定调): 连续记录格式唯一定死 mcap。**
- 禁止裸 `ros2 bag record` 采集验证数据 —— 它默认落 sqlite（格式不一致）且无 sidecar
  （废数据）。一律走 `record_verify.sh`（内建 `-s mcap` + sidecar + 命名规范）。

**必录话题**（v1.1, 按角色, 链型无关; 全部常录——不存在的话题录空流, 分析按需读）:

| 角色 | 话题 | 说明 |
|---|---|---|
| 关节状态 | `/joint_states` | 传感器侧: position/velocity/effort (JSB 发布) |
| 关节命令（上层意图） | `/joint_stream_controller/command` | 上层发的目标点流 |
| 关节命令（实际执行） | `/joint_stream_controller/applied_command` | 插值后全率实际下发 —— **与状态的差 = 跟踪账**（存在性受参数门控 `observe_applied_command`） |
| 笛卡尔命令（意图） | `/cartesian_motion_controller/command` | CM 域目标点流 —— **遥操/CM 采集的意图监督信号** |
| 笛卡尔门控（分段） | `/cartesian_motion_controller/pause` | HOLD 事件闩锁流 —— **主动暂停 = 数据分段边界** |
| 末端位姿真值 | `/ee_state` | FK 只读反馈 —— 末端策略特征源 |
| 错误板（质量标签） | `/supervisor/alerts` | SystemAlert 三域+state —— **DEGRADED/FAULT 段的事后剔除依据** |
| 时钟 | `/clock` | 仿真链必须（分析以 header.stamp 为准, 墙钟仅作参考） |

**选录**（按需 `--extra`）: `/cartesian_motion_controller/status`（CM 裁决细节遥测）· `/tf`。

**目录与命名**: `<robot>_<chain>_<task>_<YYYYmmdd_HHMMSS>/`
内含 `rosbag2_*.mcap` + `metadata.yaml`（本工具自动生成 sidecar）。

**metadata.yaml 必填**: robot / chain / task / bus_hz / **sim_time** / operator / notes /
控制器快照（`ros2 control list_controllers` 输出）。**没有 sidecar 的 mcap 视为
废数据**——三周后没人记得它的采样率。

**纪律**:
- 一段记录一个 task, 不混任务（对拍分母要干净）
- 真机记录前先 `ros2 control list_controllers` 确认控制器态并进 sidecar
- 分析一律用 `header.stamp`（sim 时间）, 不用接收时刻
- **跨钟对齐先查 sidecar `sim_time`**: mock 链 stamp=墙钟; 仿真链 stamp=sim 时间,
  与墙钟流速可能不同（如 mujoco 集成层节拍特性, 见 mujoco README 已知特性）——
  与外部墙钟信号（手柄/传感器）对齐前先实测 RTF（/clock 首尾差 ÷ 墙钟时长）
- 训练管线消费前做 uniform resample（策略需固定频率输入; 顺带抹平 stamp 不均匀）

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

**转换器 = 纯搬运, 零加工**（2026-10-12 用户令）: 点位值原样、序列零推导——
v0 命名启发式序列生成与 `--roundtrip` 合成已退役; sequences 只认显式
（v1 输入透传 / CSV 行序记为 `file_order_<side>`）。序列由**采集端**显式给出。

### 4. 标准流程（零参即用，能自动探测的全部自动）

```bash
# ① 连续记录: 起链后一条命令 (robot/chain 自动探测, 8 话题必录自动含)
ros2 run unistackbot_tool record_verify.sh <task>
# → 目录建在当前目录; 定长加 --duration 60; 指定落盘路径加 --out /data/recs;
#   加录 --extra /tf

# ② 点位转换: 纯搬运零加工 (完整参数见脚本 docstring)
ros2 run unistackbot_tool convert_points_v0_v1.py <in> <out>

# ③ 回放验证 (side/hz 自动从 meta 读)
ros2 run unistackbot_demo demo_points_playback.py --points-file <v1.json> --seq @<序列名>
```

自动探测规则：robot = `/robot_description` 的 `<robot name>`；chain = 话题指纹
（`mujoco_actuators`→mujoco / `/stats`→gz / `/clock`→其他仿真 / 无→mock_or_real）；
`sim_time` 同 /clock 探测进 sidecar。显式参数永远优先于探测。

## 工具清单

| 工具 | 语言 | 说明 |
|---|---|---|
| `record_verify.sh` | bash | 一键录标准话题集 + metadata sidecar |
| `convert_points_v0_v1.py` | py | v0 点位 → v1 (补 meta + 自然序 sequences) |

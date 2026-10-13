#!/usr/bin/env bash
# 数据录制规范 v1.1 连续记录器: 标准话题集 (关节域三层 + 笛卡尔域三层 + 错误板 + 时钟)
# + metadata.yaml sidecar。规范见本包 README §2。
# v1.1 (2026-10-12): 必录 +4 (/cartesian_motion_controller/{command,pause},
#   /ee_state, /supervisor/alerts —— 笛卡尔采集意图/分段 + 数据质量标签);
#   sidecar +sim_time 字段 (mujoco 链墙钟≠sim 时间, 跨钟对齐先看 README §2 纪律)。
# 用法: record_verify.sh <task> [--robot r] [--chain c] [--extra topic]... [--duration s]
#   产出: ./<robot>_<chain>_<task>_<时间戳>/{rosbag2_*.mcap, metadata.yaml}
# 纪律: 一段一个 task; mcap 无 sidecar 视为废数据。
set -euo pipefail

TASK="" ROBOT="" CHAIN="" DUR="" OUTDIR=""
EXTRAS=()
while [ $# -gt 0 ]; do
	case "$1" in
		--robot) ROBOT="$2"; shift 2 ;;
		--chain) CHAIN="$2"; shift 2 ;;
		--extra) EXTRAS+=("$2"); shift 2 ;;
		--duration) DUR="$2"; shift 2 ;;
		--out) OUTDIR="$2"; shift 2 ;;
		-*) echo "未知参数 $1"; exit 2 ;;
		*) TASK="$1"; shift ;;
	esac
done
[ -n "$TASK" ] || { echo "用法: record_verify.sh <task> [--robot r] [--chain c] [--extra topic]... [--duration s]"; exit 2; }

# 机型/链自动探测 (零参即用): robot 读 /robot_description 的 <robot name>,
# chain 按话题指纹 (mujoco_actuators→mujoco / /stats→gz / /clock→其他仿真, 无→mock/真机)
TOPS=$(timeout 5 ros2 topic list 2>/dev/null || true)
if [ -z "$ROBOT" ] && echo "$TOPS" | grep -q "^/robot_description$"; then
	# echo 输出是 yaml 字符串 (转义引号 \"), 提取后剥 \ 和 "
	ROBOT=$(timeout 5 ros2 topic echo /robot_description --once 2>/dev/null \
		| grep -o 'robot name=[^>]*' | head -1 \
		| sed 's/robot name=//; s/[\\"]//g' || true)
	# pipefail 下 grep 无匹配=1 会杀脚本 —— || true 兜底, 探测失败=unknown 不致命
fi
SIMTIME="false"
if echo "$TOPS" | grep -q "^/clock$"; then
	SIMTIME="true"
	if echo "$TOPS" | grep -q "mujoco_actuators"; then
		CHAIN="${CHAIN:-mujoco}"
	elif echo "$TOPS" | grep -q "^/stats$"; then
		CHAIN="${CHAIN:-gz}"
	else
		CHAIN="${CHAIN:-sim_unknown}"
	fi
else
	CHAIN="${CHAIN:-mock_or_real}"
fi
# bus_hz: 参数服务读 controller_manager update_rate (单一事实源; 读不到记 unknown)
HZ="unknown"
HZ_QUERY=$(timeout 5 ros2 param get /controller_manager update_rate 2>/dev/null || true)
if [ -n "$HZ_QUERY" ]; then
	HZ=$(echo "$HZ_QUERY" | grep -oE "[0-9]+" | tail -1)
fi

STAMP=$(date +%Y%m%d_%H%M%S)
DIR="${OUTDIR:-.}/${ROBOT:-unknown}_${CHAIN}_${TASK}_${STAMP}"
mkdir -p "$DIR"   # --out 缺省=当前目录; 父目录不存在自动建
OUT="$DIR/verify"

# 标准话题集 (规范 §2 v1.1 必录表: 关节域三层 + 笛卡尔域 + 错误板 + 时钟)
TOPICS=(/joint_states /joint_stream_controller/command
	/joint_stream_controller/applied_command
	/cartesian_motion_controller/command /cartesian_motion_controller/pause
	/ee_state /supervisor/alerts /clock)

echo "== 数据录制 v1 =="
echo "  目录: $DIR"
echo "  必录: ${TOPICS[*]}"
[ ${#EXTRAS[@]} -gt 0 ] && echo "  加录: ${EXTRAS[*]}"
[ -n "$DUR" ] && echo "  时长: ${DUR}s (无 = Ctrl-C 收尾)"

# 收尾钩子: Ctrl-C/超时后补写 sidecar (含控制器快照)
write_sidecar() {
	{
		echo "# 数据录制规范 v1.1 sidecar —— 没有 sidecar 的 mcap 视为废数据"
		echo "format: 1"
		echo "robot: ${ROBOT:-unknown}"
		echo "chain: ${CHAIN:-unknown}"
		echo "task: $TASK"
		echo "bus_hz: $HZ"
		echo "sim_time: $SIMTIME"
		echo "operator: $(whoami)"
		echo "created: $(date -Iseconds)"
		echo "topics: [${TOPICS[*]}, ${EXTRAS[*]-}]"
		echo "notes: ''"
		echo "controllers_snapshot: |"
		timeout 8 ros2 control list_controllers 2>/dev/null | sed 's/^/  /' || echo "  (链已收)"
	} > "$DIR/metadata.yaml"
	echo "sidecar 已写: $DIR/metadata.yaml"
}
trap write_sidecar EXIT

# Humble ros2 bag record 无 --duration (Rolling+) —— timeout 包裹实现定长录制
if [ -n "$DUR" ]; then
	timeout --signal=INT "$DUR" ros2 bag record -s mcap -o "$OUT" \
		"${TOPICS[@]}" ${EXTRAS[@]+"${EXTRAS[@]}"} || true
else
	ros2 bag record -s mcap -o "$OUT" "${TOPICS[@]}" ${EXTRAS[@]+"${EXTRAS[@]}"}
fi

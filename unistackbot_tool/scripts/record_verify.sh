#!/usr/bin/env bash
# 数据录制规范 v1 连续记录器: 标准话题集 (关节状态 + 命令意图 + 实际执行 + 时钟)
# + metadata.yaml sidecar。规范见本包 README §2。
# 用法: record_verify.sh <task> [--robot r] [--chain c] [--extra topic]... [--duration s]
#   产出: ./<robot>_<chain>_<task>_<时间戳>/{rosbag2_*.mcap, metadata.yaml}
# 纪律: 一段一个 task; mcap 无 sidecar 视为废数据。
set -euo pipefail

TASK="" ROBOT="" CHAIN="" DUR=""
EXTRAS=()
while [ $# -gt 0 ]; do
	case "$1" in
		--robot) ROBOT="$2"; shift 2 ;;
		--chain) CHAIN="$2"; shift 2 ;;
		--extra) EXTRAS+=("$2"); shift 2 ;;
		--duration) DUR="$2"; shift 2 ;;
		-*) echo "未知参数 $1"; exit 2 ;;
		*) TASK="$1"; shift ;;
	esac
done
[ -n "$TASK" ] || { echo "用法: record_verify.sh <task> [--robot r] [--chain c] [--extra topic]... [--duration s]"; exit 2; }

# 机型/链探测 (可缺省: 参数没给就从链上读)
if [ -z "$ROBOT" ] || [ -z "$CHAIN" ]; then
	if timeout 5 ros2 topic list 2>/dev/null | grep -q "^/clock$"; then
		CHAIN="${CHAIN:-mujoco_or_gz}"
	else
		CHAIN="${CHAIN:-mock_or_real}"
	fi
fi
# bus_hz: 参数服务读 controller_manager update_rate (单一事实源; 读不到记 unknown)
HZ="unknown"
HZ_QUERY=$(timeout 5 ros2 param get /controller_manager update_rate 2>/dev/null || true)
if [ -n "$HZ_QUERY" ]; then
	HZ=$(echo "$HZ_QUERY" | grep -oE "[0-9]+" | tail -1)
fi

STAMP=$(date +%Y%m%d_%H%M%S)
DIR="${ROBOT:-unknown}_${CHAIN}_${TASK}_${STAMP}"
mkdir -p "$DIR"
OUT="$DIR/verify"

# 标准话题集 (规范 §2 必录表)
TOPICS=(/joint_states /joint_stream_controller/command
	/joint_stream_controller/applied_command /clock)

echo "== 数据录制 v1 =="
echo "  目录: $DIR"
echo "  必录: ${TOPICS[*]}"
[ ${#EXTRAS[@]} -gt 0 ] && echo "  加录: ${EXTRAS[*]}"
[ -n "$DUR" ] && echo "  时长: ${DUR}s (无 = Ctrl-C 收尾)"

# 收尾钩子: Ctrl-C/超时后补写 sidecar (含控制器快照)
write_sidecar() {
	{
		echo "# 数据录制规范 v1 sidecar —— 没有 sidecar 的 mcap 视为废数据"
		echo "format: 1"
		echo "robot: ${ROBOT:-unknown}"
		echo "chain: ${CHAIN:-unknown}"
		echo "task: $TASK"
		echo "bus_hz: $HZ"
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

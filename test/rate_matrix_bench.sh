#!/usr/bin/env bash
# 频率矩阵基准 (2026-09-23): 控制端(命令源)低频 × 总线/CM 频率 的组合测试接口。
# 矩阵: --cmd-hz {50|100|200} × --bus-hz {500|1000} (gz 链不支持 bus_hz, 已拒)
# 用法:
#   bash test/rate_matrix_bench.sh <标签> --robot piper --chain mock \
#       --bus-hz 500 --cmd-hz 50 [--domain joint|cartesian] [--seconds 30]
#   --seconds: demo 运行上限 (demo_motion 一 cycle ~16s, 默认 30 够一整 cycle)
#   --domain joint     (默认) demo_motion.py --hz C   关节慢流 (率失配 → JS ruckig 填充)
#   --domain cartesian        demo_cartesian.py up 往返, 插值目标流 @ --stream-hz C
# 流程: 清场 → 起链(bus_hz 覆盖) → 等控制器激活 → 录包 → 跑 demo → 指标 → 结果行入
#   test/results/rate_matrix_<robot>_<chain>.md (一行一跑, 自行拼矩阵表)
# 注意: 全程不用 pkill 模式匹配 (自杀坑), 精确 PID 清理。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"
TAG="${1:?用法: rate_matrix_bench.sh <标签> --robot <机型> --chain mock|mujoco --bus-hz 500|1000 --cmd-hz 50|100|200}"
shift
ROBOT=""; CHAIN="mock"; BUS_HZ=""; CMD_HZ=""; DOMAIN="joint"; SECONDS_RUN=30
while [ $# -gt 0 ]; do
	case "$1" in
		--robot) ROBOT="$2"; shift 2 ;;
		--chain) CHAIN="$2"; shift 2 ;;
		--bus-hz) BUS_HZ="$2"; shift 2 ;;
		--cmd-hz) CMD_HZ="$2"; shift 2 ;;
		--domain) DOMAIN="$2"; shift 2 ;;
		--seconds) SECONDS_RUN="$2"; shift 2 ;;
		*) echo "未知参数 '$1'"; exit 2 ;;
	esac
done
[ -n "$ROBOT" ] || { echo "缺 --robot"; exit 2; }
case "$ROBOT" in piper|xarm7) ;; *) echo "未知机型 '$ROBOT' (piper|xarm7)"; exit 2 ;; esac
case "$CHAIN" in mock|mujoco) ;; *) echo "未知链 '$CHAIN' (mock|mujoco; gz 链 CM 在插件内不支持 bus_hz)"; exit 2 ;; esac
case "$BUS_HZ" in 500|1000) ;; *) echo "--bus-hz 须为 500|1000 (矩阵定义; 其他值直接改 yaml /**.update_rate)"; exit 2 ;; esac
case "$CMD_HZ" in 50|100|200) ;; *) echo "--cmd-hz 须为 50|100|200 (矩阵定义)"; exit 2 ;; esac
case "$DOMAIN" in joint|cartesian) ;; *) echo "--domain 须为 joint|cartesian"; exit 2 ;; esac

say() { echo "[$(date +%H:%M:%S)] $*"; }
OUT_MD="$REPO_ROOT/test/results/rate_matrix_${ROBOT}_${CHAIN}.md"
BAG="/tmp/rate_matrix_${TAG}"
LAUNCH_LOG="/tmp/rate_matrix_${TAG}_launch.log"

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
say "频率矩阵单元: 标签=$TAG 机型=$ROBOT 链=$CHAIN 总线=${BUS_HZ}Hz 命令源=${CMD_HZ}Hz 域=$DOMAIN"

# ---------- 清场 (ps 精确匹配 + 精确 PID, 勿 pkill) ----------
python3 - <<'PYEOF'
import subprocess, os, signal
out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
for line in out.splitlines()[1:]:
    parts = line.strip().split(None, 1)
    if len(parts) < 2: continue
    pid, args = int(parts[0]), parts[1]
    if args.startswith('/opt/ros/humble/lib/controller_manager/ros2_control_node') or \
       args.endswith('/mujoco_ros2_control/ros2_control_node') or \
       ('sim_control_mujoco_node' in args) or ('supervisor_node' in args) or \
       args.startswith('/usr/bin/robot_state_publisher') or \
       args.startswith('/opt/ros/humble/lib/robot_state_publisher'):
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
PYEOF
ros2 daemon stop >/dev/null 2>&1
sleep 2

# ---------- 起链 (bus_hz 运行期覆盖) ----------
if [ "$CHAIN" = "mock" ]; then
	ros2 launch unistackbot_bringup control.launch.py "robot:=$ROBOT" "bus_hz:=$BUS_HZ" \
		> "$LAUNCH_LOG" 2>&1 &
else
	ros2 launch unistackbot_mujoco mujoco.launch.py "robot:=$ROBOT" headless:=true "bus_hz:=$BUS_HZ" \
		> "$LAUNCH_LOG" 2>&1 &
fi
LAUNCH_PID=$!
say "链拉起中 (launch pid=$LAUNCH_PID, bus_hz:=$BUS_HZ) ..."

# ---------- 等控制器激活 (最长 60s) ----------
READY=0
for i in $(seq 1 60); do
	ACTIVE=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep -c "active")
	if [ "${ACTIVE:-0}" -ge 3 ]; then READY=1; break; fi
	sleep 1
done
if [ "$READY" != "1" ]; then
	say "FAIL: 60s 内控制器未全部激活 (active=$ACTIVE); 保留日志 $LAUNCH_LOG 供排查"
	kill "$LAUNCH_PID" 2>/dev/null
	exit 1
fi
say "控制器已激活 (3 个 active); JS 校准行:"
grep -E "update_rate .*Hz|权威参数|实测中位数" "$LAUNCH_LOG" | tail -4 | sed 's/^/    /'

# ---------- 录包 → 跑 demo → 停录 ----------
# cartesian 域: JS command 话题在 CM 模式下静默 (CM 直接认领接口), 补录 CM 两话题
# 才能对账命令侧频率/跟踪
CM_TOPICS=""
if [ "$DOMAIN" = "cartesian" ]; then
	CM_TOPICS="/cartesian_motion_controller/status /cartesian_motion_controller/target"
fi
rm -rf "$BAG"
ros2 bag record -s mcap /joint_states /joint_stream_controller/command $CM_TOPICS -o "$BAG" >/dev/null 2>&1 &
BAG_PID=$!
sleep 2
say "录制中, 跑 demo (${SECONDS_RUN}s 上限) ..."
if [ "$DOMAIN" = "joint" ]; then
	timeout "$SECONDS_RUN" ros2 run unistackbot_demo demo_motion.py --hz "$CMD_HZ" --cycles 1
else
	timeout "$SECONDS_RUN" ros2 run unistackbot_demo demo_cartesian.py --pattern up \
		--traverse 2.0 --stream-hz "$CMD_HZ"
fi
DEMO_RC=$?
sleep 1
kill -INT "$BAG_PID" 2>/dev/null; sleep 3
kill "$BAG_PID" 2>/dev/null

# ---------- 指标 + 结果行 ----------
say "计算指标 ..."
METRICS=$(python3 "$REPO_ROOT/test/rate_matrix_metrics.py" "$BAG" --bus-hz "$BUS_HZ" --cmd-hz "$CMD_HZ" --domain "$DOMAIN" 2>&1)
RC=$?
if [ $RC -ne 0 ]; then
	say "FAIL: 指标计算失败: $METRICS"
else
	echo "$METRICS"
	mkdir -p "$(dirname "$OUT_MD")"
	[ -f "$OUT_MD" ] || echo "| 标签 | 链 | 总线Hz | 命令Hz | 域 | 命令达成Hz | JS实测Hz | max\|dv\|/拍 (worst关节) | p50\|dv\|/拍 | 末端|Δq|2阶差 max | 备注 |" > "$OUT_MD"
	echo "$METRICS" | grep "^| " | tail -1 >> "$OUT_MD"
	say "结果行已追加 → $OUT_MD"
fi

# ---------- 收场 ----------
kill "$LAUNCH_PID" 2>/dev/null; sleep 3
python3 - <<'PYEOF'
import subprocess, os, signal
out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
for line in out.splitlines()[1:]:
    parts = line.strip().split(None, 1)
    if len(parts) < 2: continue
    pid, args = int(parts[0]), parts[1]
    if args.startswith('/opt/ros/humble/lib/controller_manager/ros2_control_node') or \
       args.endswith('/mujoco_ros2_control/ros2_control_node') or \
       ('sim_control_mujoco_node' in args) or ('supervisor_node' in args) or \
       args.startswith('/usr/bin/robot_state_publisher') or \
       args.startswith('/opt/ros/humble/lib/robot_state_publisher'):
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
PYEOF
ros2 daemon stop >/dev/null 2>&1
say "收场完成 (demo 退出码=$DEMO_RC; 下一格前建议等 10s DDS 租约)"
[ $RC -eq 0 ] && [ $DEMO_RC -ne 124 ] && exit 0 || exit 0

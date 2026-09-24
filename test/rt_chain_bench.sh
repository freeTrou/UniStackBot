#!/usr/bin/env bash
# RT 链路基准测试套件 (2026-09-20; 2026-09-22 机型参数化 --robot + 终端进度提示)。
# 用法: bash test/rt_chain_bench.sh <标签> --robot <piper|xarm7> [--chain mock|mujoco]
#   --chain mock   (默认) → C 节 = 纯算法 WCET (理想执行器)
#   --chain mujoco        → C 节 = 动力学仿真下 WCET (物理线程与 CM RT 线程同机争用的真实代价)
# 前置: colcon build + source; 无残留链; 输出 test/results/rt_<标签>.md (终端有分步进度)
# 注意: 全程不用 pkill 模式匹配 (自杀坑), 精确 PID 清理。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"
TAG="${1:?用法: rt_chain_bench.sh <标签> --robot <piper|xarm7> [--chain mock|mujoco]}"
OUT="$REPO_ROOT/test/results/rt_${TAG}.md"
shift
CHAIN="mock"; ROBOT=""
while [ $# -gt 0 ]; do
	case "$1" in
		--robot) ROBOT="${2:?--robot 需要值: piper|xarm7}"; shift 2 ;;
		--chain) CHAIN="${2:?--chain 需要值: mock|mujoco}"; shift 2 ;;
		*) echo "未知参数 '$1' (支持: --robot <piper|xarm7> --chain mock|mujoco)"; exit 2 ;;
	esac
done
[ -n "$ROBOT" ] || { echo "缺 --robot: bash $0 <标签> --robot <piper|xarm7> [--chain mock|mujoco]"; exit 2; }
case "$ROBOT" in piper|xarm7) ;; *) echo "未知机型 '$ROBOT' (piper|xarm7)"; exit 2 ;; esac
case "$CHAIN" in mock|mujoco) ;; *) echo "未知链 '$CHAIN' (mock|mujoco)"; exit 2 ;; esac
say() { echo "[$(date +%H:%M:%S)] $*"; }

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
say "RT 基准开始: 标签=$TAG 机型=$ROBOT 链=$CHAIN → 报告 $OUT (全程约 12-15 分钟)"

# ---------- 清场 ----------
clean_procs() {
  python3 - <<'PYEOF'
import subprocess, os, signal
out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
for line in out.splitlines()[1:]:
    parts = line.strip().split(None, 1)
    if len(parts) < 2: continue
    pid, args = int(parts[0]), parts[1]
    if args.startswith('/opt/ros/humble/lib/controller_manager/ros2_control_node') or \
       ('ign gazebo' in args) or args.startswith('/usr/bin/ign') or \
       ('sim_control_mujoco_node' in args) or ('robot_state_publisher' in args):
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
PYEOF
}
say "清场 (含孤儿适配器) + 5s 等租约 ..."
clean_procs
sleep 5

# ---------- 环境快照 ----------
KERNEL=$(uname -r)
CMDLINE=$(cat /proc/cmdline)
PREEMPT=$(grep -o "CONFIG_PREEMPT[A-Z_]*=y" /boot/config-$KERNEL 2>/dev/null | head -3 | tr '\n' ' ')
mkdir -p "$REPO_ROOT/test/results"
cat > "$OUT" <<HDR
# RT 基准: $TAG ($(date '+%Y-%m-%d %H:%M'))

| 项 | 值 |
|---|---|
| 内核 | $KERNEL |
| PREEMPT | $PREEMPT |
| cmdline | \`${CMDLINE}\` |
| isolcpus | $(grep -oE "isolcpus=[^ ]*" /proc/cmdline || echo 无) |
| sched_rt_runtime_us | $(sysctl -n kernel.sched_rt_runtime_us) |
| ulimit -l | $(ulimit -l) |

HDR

# ---------- A. 平台层: cyclictest 三档负载 (核1,2) ----------
echo "## A. cyclictest 三档 (核1+核2, FIFO80, 120s/档)" >> "$OUT"
echo "" >> "$OUT"
echo "| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |" >> "$OUT"
echo "|---|---|---|" >> "$OUT"

run_cyclic() {  # $1=负载进程数 $2=档名
  say "[A] cyclictest '$2' 档开始 (120s${1:+, 造载 $1 进程}) ..."
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" start "$1"; sleep 3; fi
  # isolcpus=1,2 下默认亲和 mask 不含隔离核, cyclictest 不突破继承 mask (直接 -a 会 FATAL) → 必须显式 taskset
  R=$(taskset -c 1,2 cyclictest -m -p 80 -D 120s -h 200 -a 1,2 -t 2 -q 2>&1)
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" stop; fi
  sleep 2
  M=$(echo "$R" | grep "Min Latencies"); A=$(echo "$R" | grep "Avg Latencies"); X=$(echo "$R" | grep "Max Latencies")
  [ -z "$M" ] && { echo "- ⚠️ $2 档 cyclictest 无统计输出:" >> "$OUT"; echo "$R" | grep -E "WARN|FATAL|Error" | sed 's/^/  /' >> "$OUT"; M="— —"; A="— —"; X="— —"; }
  echo "| $2 | $(echo $M|awk '{print $4}') / $(echo $A|awk '{print $4}') / $(echo $X|awk '{print $4}') | $(echo $M|awk '{print $5}') / $(echo $A|awk '{print $5}') / $(echo $X|awk '{print $5}') |" >> "$OUT"
  echo "$R" | grep -E "WARN|FATAL" | grep -v "cpu_dma_latency" | sed "s/^/- cyclictest $2: /" >> "$OUT"
  say "[A] '$2' 档完成 (核1 max $(echo $X|awk '{print $4}')µs, 已入报告)"
}
run_cyclic 0  "无负载 (0/28)"
run_cyclic 14 "50% (14进程)"
run_cyclic 25 "90% (25进程)"

# ---------- B. SMI 计数 + 中断分布 + hwlatdetect (固件级) ----------
say "[B] SMI 采样 30s + 中断分布 ..."
if command -v hwlatdetect >/dev/null 2>&1; then
  say "[B] hwlatdetect 60s (需 sudo 免密, 失败自动跳过) ..."
  HW=$(sudo -n hwlatdetect --duration=60 --threshold=5 2>/dev/null | grep -E "Max Latency|Samples" | tr '\n' ' ')
  [ -z "$HW" ] && HW="sudo 需密码, 跳过 (手动: sudo hwlatdetect --duration=60 --threshold=5)"
else
  HW="rt-tests 未装 hwlatdetect"
fi
echo "" >> "$OUT"
echo "## B. SMI 与中断 (30s 采样)" >> "$OUT"
SMI1=$(perf stat -e msr/smi_counter/ -a sleep 30 2>&1 | grep smi | grep -oE "[0-9,]+" | head -1)
echo "- SMI 计数(30s, 全核): ${SMI1:-工具不可用}" >> "$OUT"
IRQ1=$(cat /proc/interrupts | awk '{print $1}' | grep -c ":") 
echo "- IRQ 活跃行数: $IRQ1 (优化后对比: isolcpus+managed_irq 应减少核1/2 命中)" >> "$OUT"
echo "- hwlatdetect(60s, 阈20µs): $HW" >> "$OUT"
echo "- RT 带宽: runtime=$(sysctl -n kernel.sched_rt_runtime_us)us / period=$(sysctl -n kernel.sched_rt_period_us)us" >> "$OUT"

# ---------- C. 链路层: CM 拍间隔 + WCET + E2E (三档负载) ----------
echo "" >> "$OUT"
echo "## C. CM 链路 ($ROBOT/$CHAIN, 三档负载 E2E)" >> "$OUT"
echo "" >> "$OUT"
echo "| 负载档 | WCET p50/p99/max (µs) | 跟踪误差 (mm) | 收敛时间 (s) | 备注 |" >> "$OUT"
echo "|---|---|---|---|---|" >> "$OUT"

say "[C] 起 $ROBOT $CHAIN 链 (15s 等待控制器) ..."
if [ "$CHAIN" = "mujoco" ]; then
  ros2 launch unistackbot_mujoco mujoco.launch.py "robot:=$ROBOT" headless:=true > /tmp/rtb_launch.log 2>&1 &
  LPID=$!
else
  ros2 launch unistackbot_bringup control.launch.py "robot:=$ROBOT" use_rviz:=false > /tmp/rtb_launch.log 2>&1 &
  LPID=$!
fi
sleep 15
for i in 1 2 3; do
  timeout 15 ros2 control switch_controllers --deactivate joint_stream_controller --activate cartesian_motion_controller >/dev/null 2>&1
  [ "$(timeout 8 ros2 control list_controllers 2>/dev/null | grep cartesian | grep -c active)" = "1" ] && break
  sleep 2
done

e2e_one_tier() {  # $1=负载进程数 $2=档名
  say "[C] E2E 档 '$2' 开始 (目标收敛 + WCET 采样) ..."
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" start "$1"; sleep 3; fi
  # 目标跟踪: 机型无关的近目标 —— 当前 EE 位姿 (ee_state, base 系) + 3cm x 偏移,
  # 姿态取当前值 (verify_motion 同款策略, 不写死任何机型的可达位姿)
  TARGET=$(timeout 5 ros2 topic echo /ee_state_broadcaster/ee_state --once 2>/dev/null | python3 -c "
import sys, yaml
d = next(yaml.safe_load_all(sys.stdin), None)
if d is None: sys.exit(1)
p, o = d['pose']['position'], d['pose']['orientation']
p['x'] = round(p['x'] + 0.03, 4)
print('{header: {frame_id: %s}, pose: {position: {x: %s, y: %s, z: %s}, orientation: {x: %s, y: %s, z: %s, w: %s}}}' % (
    repr(d['header']['frame_id']), p['x'], p['y'], p['z'], o['x'], o['y'], o['z'], o['w']))
")
  timeout 8 ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped \
    "${TARGET:?'ee_state 无数据 (链未起齐?)'}" >/dev/null 2>&1
  T0=$(date +%s.%N)
  CONV=""
  for i in $(seq 1 40); do
    S=$(timeout 4 ros2 topic echo /cartesian_motion_controller/status --once 2>/dev/null | grep "converged: true" | head -1)
    [ -n "$S" ] && { CONV=$(echo "$(date +%s.%N) - $T0" | bc); break; }
    sleep 0.3
  done
  # 终态误差
  ERR=$(timeout 4 ros2 topic echo /cartesian_motion_controller/status --once 2>/dev/null | grep position_error | awk '{printf "%.4f", $2*1000}')
  # WCET: 停用触发终报
  timeout 15 ros2 control switch_controllers --deactivate cartesian_motion_controller >/dev/null 2>&1
  sleep 1
  W=$(grep -a "WCET 终报" /tmp/rtb_launch.log | tail -1 | grep -oE "p50=[0-9.]+µs p99=[0-9.]+µs max=[0-9.]+" | head -1)
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" stop; fi
  # 重新激活供下一档
  timeout 15 ros2 control switch_controllers --activate cartesian_motion_controller >/dev/null 2>&1
  sleep 1
  echo "| $2 | ${W:-N/A} | ${ERR:-N/A} | ${CONV:-超时} | |" >> "$OUT"
  say "[C] E2E 档 '$2' 完成 (WCET=${W:-N/A} 误差=${ERR:-N/A}mm)"
}
e2e_one_tier 0  "无负载"
e2e_one_tier 14 "50%"
e2e_one_tier 25 "90%"

kill $LPID 2>/dev/null; sleep 2; clean_procs
echo "" >> "$OUT"
echo "复测: bash test/rt_chain_bench.sh <新标签> --robot $ROBOT --chain $CHAIN; 对比 test/results/ 下两文件。" >> "$OUT"
say "完成 → cat $OUT"
echo ""
echo "===== 完成: $OUT ====="
cat "$OUT"

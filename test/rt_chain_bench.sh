#!/usr/bin/env bash
# RT 链路基准测试套件 (2026-09-20): 平台 cyclictest 三档 + CM 链路层指标。
# 用法: bash test/rt_chain_bench.sh <标签>     如: baseline / lowlat_after
# 前置: colcon build + source; gz_clean 无残留; 输出 test/results/rt_<标签>.md
# 注意: 全程不用 pkill 模式匹配 (自杀坑), 精确 PID 清理。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"
TAG="${1:?用法: rt_chain_bench.sh <标签>}"
OUT="$REPO_ROOT/test/results/rt_${TAG}.md"

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

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
       ('ign gazebo' in args) or args.startswith('/usr/bin/ign'):
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
PYEOF
}
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
| sched_rt_runtime_us | $(sysctl -n kernel.sched_rt_runtime_us) |
| ulimit -l | $(ulimit -l) |

HDR

# ---------- A. 平台层: cyclictest 三档负载 (核1,2) ----------
echo "## A. cyclictest 三档 (核1+核2, FIFO80, 120s/档)" >> "$OUT"
echo "" >> "$OUT"
echo "| 负载档 | 核1 Min/Avg/Max (µs) | 核2 Min/Avg/Max (µs) |" >> "$OUT"
echo "|---|---|---|" >> "$OUT"

run_cyclic() {  # $1=负载进程数 $2=档名
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" start "$1"; sleep 3; fi
  R=$(cyclictest -m -p 80 -D 120s -h 200 -a 1,2 -t 2 -q 2>&1 | grep "^# ")
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" stop; fi
  sleep 2
  M=$(echo "$R" | grep Min); A=$(echo "$R" | grep Avg); X=$(echo "$R" | grep Max)
  echo "| $2 | $(echo $M|awk '{print $4}') / $(echo $A|awk '{print $4}') / $(echo $X|awk '{print $4}') | $(echo $M|awk '{print $5}') / $(echo $A|awk '{print $5}') / $(echo $X|awk '{print $5}') |" >> "$OUT"
}
run_cyclic 0  "无负载 (0/28)"
run_cyclic 14 "50% (14进程)"
run_cyclic 25 "90% (25进程)"

# ---------- B. SMI 计数 + 中断分布 + hwlatdetect (固件级) ----------
if command -v hwlatdetect >/dev/null 2>&1; then
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
echo "## C. CM 链路 (mock xarm7, 三档负载 E2E)" >> "$OUT"
echo "" >> "$OUT"
echo "| 负载档 | WCET p50/p99/max (µs) | 跟踪误差 (mm) | 收敛时间 (s) | 备注 |" >> "$OUT"
echo "|---|---|---|---|---|" >> "$OUT"

ros2 launch unistackbot_bringup control.launch.py robot:=xarm7 use_rviz:=false > /tmp/rtb_launch.log 2>&1 &
LPID=$!
sleep 15
for i in 1 2 3; do
  timeout 15 ros2 control switch_controllers --deactivate joint_stream_controller --activate cartesian_motion_controller >/dev/null 2>&1
  [ "$(timeout 8 ros2 control list_controllers 2>/dev/null | grep cartesian | grep -c active)" = "1" ] && break
  sleep 2
done

e2e_one_tier() {  # $1=负载进程数 $2=档名
  if [ "$1" -gt 0 ]; then python3 "$SCRIPT_DIR/cpu_load.py" start "$1"; sleep 3; fi
  # 目标跟踪: 发一个近目标 (预摆位后), 等 6s 收敛
  timeout 8 ros2 topic pub --once /cartesian_motion_controller/target geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: link_base}, pose: {position: {x: 0.30, y: 0.10, z: 0.40}, orientation: {w: 0.5, x: 0.5, y: 0.5, z: 0.5}}}" >/dev/null 2>&1
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
}
e2e_one_tier 0  "无负载"
e2e_one_tier 14 "50%"
e2e_one_tier 25 "90%"

kill $LPID 2>/dev/null; sleep 2; clean_procs
echo "" >> "$OUT"
echo "复测: bash test/rt_chain_bench.sh <新标签>; 对比 test/results/ 下两文件。" >> "$OUT"
echo ""
echo "===== 完成: $OUT ====="
cat "$OUT"

#!/usr/bin/env bash
# SimControl 统一仿真控制层冒烟测试 (mock 链路)。
# 自包含: 清理 -> 启动 mock 链路 -> 断言 /sim_control/* 全部行为 -> 演示轨迹回归 -> 清理。
# 用法: bash test/smoke_sim_control.sh     (需要已 colcon build 并 source)
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"

# ROS setup 脚本内部引用未绑定变量, source 期间需临时关闭 nounset
set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
[ -f "$HOME/cyclonedds.xml" ] && export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
set -u

# DDS: 优先机器默认配置 ~/cyclonedds.xml (与本工程其余部分保持一致)
[ -f "$HOME/cyclonedds.xml" ] && export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"

SVC="ros2 service call"
SETSTATE_SRV="unistackbot_sim_control/srv/SetJointState"
LOG=/tmp/smoke_sim_control.log
PASS=0
FAIL=0

say()  { echo "[$(date +%H:%M:%S)] $*"; }
ok()   { PASS=$((PASS+1)); say "PASS: $1"; }
bad()  { FAIL=$((FAIL+1)); say "FAIL: $1"; }

# 读取指定关节的 position (通过 /joint_states 一帧)
joint_pos() {
	timeout 5 ros2 topic echo /joint_states --once --no-daemon 2>/dev/null | python3 -c "
import sys
t = sys.stdin.read()
if 'name:' not in t or 'position:' not in t:
    print('nan'); sys.exit()
names = [x.strip('- ') for x in t.split('name:')[1].split('position:')[0].strip().split('\n')]
vals  = [x.strip('- ') for x in t.split('position:')[1].split('velocity:')[0].strip().split('\n')]
d = dict(zip(names, [float(v) for v in vals]))
print(d.get('$1', 'nan'))
"
}

say "清理残留进程"
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1

say "启动 mock 链路"
ros2 launch unistackbot_bringup piper_control.launch.py > "$LOG" 2>&1 &
LAUNCH_PID=$!

# 等控制器激活 (最多 40s)
READY=0
for _ in $(seq 1 40); do
	if grep -q "Configured and activated" "$LOG" && \
		[ "$(grep -c 'Configured and activated' "$LOG")" -ge 2 ]; then
		READY=1
		break
	fi
	sleep 1
done
if [ "$READY" -eq 1 ]; then ok "控制器激活"; else bad "控制器 40s 内未全部激活 (见 $LOG)"; fi

sleep 1   # 等 /joint_states 出数据

# ---- pause 后瞬移 (JTC 激活时保持命令会立刻覆盖瞬移, 必须先冻结) ----
timeout 10 $SVC /sim_control/pause std_srvs/srv/Trigger >/dev/null 2>&1
timeout 10 $SVC /sim_control/set_joint_state $SETSTATE_SRV \
	"{joint_names: [joint1, gripper], positions: [1.0, 0.08]}" >/dev/null 2>&1
sleep 0.5
P=$(joint_pos joint1);  [ "$P" = "1.0" ]  && ok "set_joint_state joint1=1.0 (pause 下保持)"   || bad "joint1=$P (期望 1.0)"
P=$(joint_pos gripper); [ "$P" = "0.08" ] && ok "set_joint_state gripper=0.08" || bad "gripper=$P (期望 0.08)"
P=$(joint_pos gripper_joint1); python3 -c "exit(0 if abs($P-0.04)<1e-6 else 1)" \
	&& ok "mimic 手指推导 0.04" || bad "gripper_joint1=$P (期望 0.04)"

# ---- 超限位拒绝 ----
R=$(timeout 10 $SVC /sim_control/set_joint_state $SETSTATE_SRV \
	"{joint_names: [joint1], positions: [99.0]}" 2>/dev/null)
echo "$R" | grep -qi "success.*false" && echo "$R" | grep -q "out of limits" \
	&& ok "超限位拒绝并返回原因" || bad "超限位未正确拒绝: $R"

# ---- mimic 关节拒绝 ----
R=$(timeout 10 $SVC /sim_control/set_joint_state $SETSTATE_SRV \
	"{joint_names: [gripper_joint1], positions: [0.02]}" 2>/dev/null)
echo "$R" | grep -qi "success.*false" && echo "$R" | grep -q "mimic" \
	&& ok "mimic 关节拒绝" || bad "mimic 未正确拒绝: $R"

# ---- reset 回零 (暂停下无竞争) ----
timeout 10 $SVC /sim_control/reset std_srvs/srv/Trigger >/dev/null 2>&1
sleep 0.3
P=$(joint_pos joint1);  [ "$P" = "0.0" ] && ok "reset 回零" || bad "reset 后 joint1=$P"
P=$(joint_pos gripper_joint1); [ "$P" = "0.0" ] && ok "reset 后 mimic 同步回零" || bad "reset 后 mimic=$P"

# ---- resume: JTC 保持命令为 0, 应维持零位 ----
timeout 10 $SVC /sim_control/resume std_srvs/srv/Trigger >/dev/null 2>&1
sleep 2
P=$(joint_pos joint1); python3 -c "exit(0 if abs($P)<1e-6 else 1)" \
	&& ok "resume 后维持零位" || bad "resume 后 joint1=$P (期望 0.0)"

# ---- 演示轨迹回归 ----
say "演示轨迹回归 (约 12s)"
DEMO=$(timeout 30 ros2 run unistackbot_bringup piper_demo_motion.py 2>&1)
echo "$DEMO" | grep -q "状态码 4" && ok "演示轨迹执行成功" || bad "演示轨迹失败: $DEMO"

# ---- 收尾 ----
kill "$LAUNCH_PID" 2>/dev/null
wait "$LAUNCH_PID" 2>/dev/null
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1

say "结果: PASS=$PASS FAIL=$FAIL"
[ "$FAIL" -eq 0 ]

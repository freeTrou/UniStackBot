#!/usr/bin/env bash
# 机型参数化的链路验证: 静态 URDF → launch 参数化错误路径 → mock 链 JTC 运动到位 → (可选) Fortress 链。
# 与 smoke_sim_control.sh 的分工: smoke 深测 /sim_control 服务语义(当前 piper 专属断言),
# 本脚本横向验证"任意机型三条链全通"——加新机器人后跑一遍即是验收。
# 轨迹目标不写死: 从 URDF <ros2_control> 限位推算 (min + 0.3*(max-min), 恒在限位内)。
# 用法: bash test/verify_robot.sh <robot> [--with-gazebo]
#   例: bash test/verify_robot.sh xarm7 --with-gazebo
set -u

ROBOT="${1:?用法: verify_robot.sh <robot> [--with-gazebo]}"
WITH_GZ=0
[ "${2:-}" = "--with-gazebo" ] && WITH_GZ=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"

# ROS setup 脚本内部引用未绑定变量, source 期间需临时关闭 nounset (与 smoke 同型)
set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
[ -f "$HOME/cyclonedds.xml" ] && export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

PASS=0; FAIL=0
say()  { echo "[$(date +%H:%M:%S)] $*"; }
ok()   { PASS=$((PASS+1)); say "PASS: $1"; }
bad()  { FAIL=$((FAIL+1)); say "FAIL: $1"; }

# ---- 前置: 机型文件存在 (显式错误, 不让后面每步都摔一遍) ----
DESC_SHARE="$(ros2 pkg prefix --share unistackbot_description 2>/dev/null)" || { echo "找不到 unistackbot_description, 先 colcon build + source"; exit 1; }
BRINGUP_SHARE="$(ros2 pkg prefix --share unistackbot_bringup 2>/dev/null)" || { echo "找不到 unistackbot_bringup"; exit 1; }
XACRO="$DESC_SHARE/arms/$ROBOT/urdf/$ROBOT.urdf.xacro"
CYAML="$BRINGUP_SHARE/config/${ROBOT}_controllers.yaml"
[ -f "$XACRO" ] || { echo "未知机型 '$ROBOT': $XACRO 不存在 (可用: $(ls "$DESC_SHARE/arms" | tr '\n' ' '))"; exit 1; }
[ -f "$CYAML" ] || { echo "机型 '$ROBOT' 缺控制器配置: $CYAML"; exit 1; }

# ================= 1. 静态检查 =================
say "== [1] 静态: xacro 展开 + check_urdf =="
URDF=/tmp/verify_${ROBOT}.urdf
if xacro "$XACRO" use_world:=true use_ros2_control:=true > "$URDF" 2>/tmp/verify_xacro_err.txt; then
	ok "xacro 展开成功"
else
	bad "xacro 展开失败: $(cat /tmp/verify_xacro_err.txt | tail -3)"
	exit 1
fi
check_urdf "$URDF" >/tmp/verify_urdf_tree.txt 2>&1 \
	&& grep -q "Successfully Parsed" /tmp/verify_urdf_tree.txt \
	&& ok "check_urdf 解析通过 ($(grep -c 'child' /tmp/verify_urdf_tree.txt) 个子链)" \
	|| bad "check_urdf 失败: $(head -2 /tmp/verify_urdf_tree.txt)"

# 从展开产物提取: yaml 关节集合 / 限位, 并推算 JTC 目标 (min + 0.3*(max-min))
read -r JOINS TARGS <<< "$(python3 -c "
import xml.etree.ElementTree as ET, yaml, sys
lim = {}
for j in ET.parse('$URDF').iter('joint'):
    if j.get('name') is None or j.find('command_interface') is None: continue
    ps = {p.get('name'): p.text for p in j.findall('param')}
    if 'min' in ps and 'max' in ps:
        lim[j.get('name')] = (float(ps['min']), float(ps['max']))
joints = yaml.safe_load(open('$CYAML'))['joint_trajectory_controller']['ros__parameters']['joints']
missing = [n for n in joints if n not in lim]
if missing: print(f'ERROR 关节缺限位: {missing}'); sys.exit(1)
targ = [round(lim[n][0] + 0.3*(lim[n][1]-lim[n][0]), 4) for n in joints]
print(','.join(joints), '[' + ','.join(str(t) for t in targ) + ']')
" 2>&1)"
if [[ "$JOINS" == ERROR* ]]; then
	bad "目标推算失败: $JOINS $TARGS"
	exit 1
fi
ok "目标推算 ($JOINS): $TARGS"

# ================= 2. launch 参数化错误路径 =================
say "== [2] 错误路径: 必填参数 / 未知机型 =="
E=$(timeout 15 ros2 launch unistackbot_bringup control.launch.py 2>&1)
echo "$E" | grep -q "required argument 'robot'" && ok "缺 robot 参数显式报错" || bad "缺参未报错: $(echo "$E" | tail -1)"
E=$(timeout 15 ros2 launch unistackbot_bringup control.launch.py robot:="__none__" 2>&1)
echo "$E" | grep -q "未知机型" && ok "未知机型显式报错并列出可用项" || bad "未知机型未报错: $(echo "$E" | tail -1)"

# ---- 公共: 等双控制器 active ----
wait_controllers() {  # $1=最长等待秒数 $2=log
	for _ in $(seq 1 "$1"); do
		C=$(timeout 5 ros2 control list_controllers 2>/dev/null)
		if echo "$C" | grep -q "joint_state_broadcaster.*active" && echo "$C" | grep -q "joint_trajectory_controller.*active"; then
			return 0
		fi
		[ -s "$2" ] && grep -qE "Error|error while" "$2" && return 2
		sleep 1
	done
	return 1
}

# ---- 公共: 发 JTC 目标并逐关节断言到位 ----
run_goal_and_check() {  # $1=到位容差
	GOAL_YAML="{trajectory: {joint_names: [$JOINS], points: [{positions: [$(python3 -c "print(','.join('0.0' for _ in '$JOINS'.split(',')))")], time_from_start: {sec: 1}}, {positions: $TARGS, time_from_start: {sec: 4}}]}}"
	R=$(timeout 20 ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
		control_msgs/action/FollowJointTrajectory "$GOAL_YAML" 2>&1)
	echo "$R" | grep -q "SUCCEEDED" && ok "JTC goal SUCCEEDED" || { bad "JTC goal 失败: $(echo "$R" | tail -2)"; return 1; }
	sleep 2
	P=$(timeout 6 ros2 topic echo /joint_states --once 2>/dev/null)
	RES=$(echo "$P" | python3 -c "
import sys, yaml
d = next(yaml.safe_load_all(sys.stdin), None)
if d is None: print('ERROR 无 joint_states'); sys.exit()
targ = dict(zip('$JOINS'.split(','), [float(x) for x in '$TARGS'.strip('[]').split(',')]))
errs = {n: abs(p - targ[n]) for n, p in zip(d['name'], d['position']) if n in targ}
miss = set(targ) - set(d['name'])
if miss: print('ERROR 缺关节', miss); sys.exit()
worst = max(errs.values())
print('OK' if worst < $1 else 'ERR', f'{worst:.4f}')")
	if [[ "$RES" == OK* ]]; then ok "全部关节到位 (最大偏差 ${RES#OK })"; else bad "未到位: $RES"; fi
}

cleanup_launch() {  # $1=launch pid
	[ -n "${1:-}" ] && kill "$1" 2>/dev/null
	[ -n "${1:-}" ] && wait "$1" 2>/dev/null
	ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
}

# ================= 3. mock 链 =================
say "== [3] mock 链: control.launch.py robot:=$ROBOT =="
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
LOG=/tmp/verify_${ROBOT}_mock.log
ros2 launch unistackbot_bringup control.launch.py "robot:=$ROBOT" > "$LOG" 2>&1 &
LPID=$!
if wait_controllers 40 "$LOG"; then
	ok "双控制器 active"
else
	bad "控制器 40s 未激活 (见 $LOG: $(grep -iE 'error' "$LOG" | tail -2))"
	cleanup_launch "$LPID"; exit 1
fi
SVCN=$(timeout 5 ros2 service list 2>/dev/null | grep -c "/sim_control/")
[ "$SVCN" -ge 5 ] && ok "/sim_control 服务齐全 (${SVCN}个)" || bad "/sim_control 服务缺 ($SVCN/5+)"
sleep 1
run_goal_and_check 0.01
cleanup_launch "$LPID"

# ================= 4. Fortress 链 (可选) =================
if [ "$WITH_GZ" -eq 1 ]; then
	say "== [4] Fortress 链: ign.launch.py robot:=$ROBOT gui:=false =="
	ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
	LOG=/tmp/verify_${ROBOT}_ign.log
	ros2 launch unistackbot_gazebo ign.launch.py "robot:=$ROBOT" gui:=false > "$LOG" 2>&1 &
	LPID=$!
	if wait_controllers 90 "$LOG"; then
		ok "双控制器 active (gz_ros2_control 内)"
	else
		bad "控制器 90s 未激活 (见 $LOG: $(grep -iE 'error|warn' "$LOG" | tail -3))"
		cleanup_launch "$LPID"; exit 1
	fi
	timeout 6 ros2 topic echo /clock --once >/dev/null 2>&1 && ok "/clock 仿真时间在流" || bad "/clock 无数据"
	read -r RTFV PAUSED <<< "$(timeout 6 ros2 topic echo /stats --once 2>/dev/null | python3 -c "
import sys, yaml
d = next(yaml.safe_load_all(sys.stdin), None)
print(f'{d[\"real_time_factor\"]:.3f} {d[\"paused\"]}' if d else 'NONE True')")"
	python3 -c "exit(0 if float('$RTFV') > 0.1 else 1)" && [ "$PAUSED" = "False" ] \
		&& ok "RTF 监控可用 (real_time_factor=$RTFV, 未暂停)" || bad "/stats 异常: rtf=$RTFV paused=$PAUSED"
	SVCN=$(timeout 5 ros2 service list 2>/dev/null | grep -c "/sim_control/")
	[ "$SVCN" -ge 5 ] && ok "/sim_control 服务在 gz 链由适配器承载 (${SVCN}个)" || bad "/sim_control 服务缺 ($SVCN/5+)"
	run_goal_and_check 0.05
	cleanup_launch "$LPID"
fi

say "结果: PASS=$PASS FAIL=$FAIL (log: /tmp/verify_${ROBOT}_*.log)"
[ "$FAIL" -eq 0 ]

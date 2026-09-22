#!/usr/bin/env bash
# 机型参数化的链路验证: 静态 URDF → launch 参数化错误路径 → 链上运动 (JointStream 流 + CM 切换)
# → 可选 gz / mujoco 链。
# 2026-09-21 重写: JTC action 断言退役 (链上已无 JTC), 改 verify_motion.py (JS 50Hz 流 + CM 收敛)。
# 与 smoke_sim_control.sh 的分工: smoke 深测 /sim_control 服务语义(当前 piper 专属断言),
# 本脚本横向验证"任意机型×链全通"——加新机器人后跑一遍即是验收。
# 轨迹目标不写死: 从各链渲染的 URDF <ros2_control> 限位推算 (min + 0.3*(max-min), 恒在限位内);
# 关节集随链变 (mujoco/gz 的 piper 手指 passive → 7 关节, mock 9 关节)。
# 用法: bash test/verify_robot.sh <robot> [--with-gazebo] [--with-mujoco]
#   例: bash test/verify_robot.sh xarm7 --with-mujoco
set -u

ROBOT="${1:?用法: verify_robot.sh <robot> [--with-gazebo] [--with-mujoco]}"
WITH_GZ=0; WITH_MJ=0
for a in "${@:2}"; do
	case "$a" in
		--with-gazebo) WITH_GZ=1 ;;
		--with-mujoco) WITH_MJ=1 ;;
		*) echo "未知参数 $a"; exit 2 ;;
	esac
done

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
tally() {  # 汇总 verify_motion.py 的输出行
	local out="$1"
	while IFS= read -r line; do
		case "$line" in
			PASS:*) ok "${line#PASS: }" ;;
			WARN:*) say "WARN:${line#WARN: }" ;;
			FAIL:*) bad "${line#FAIL: }" ;;
			*) [ -n "$line" ] && say "RAW: $line" ;;   # 未识别输出显形 (崩溃栈/环境行, 不计数)
		esac
	done <<< "$out"
}

# ---- 前置: 机型文件存在 (显式错误, 不让后面每步都摔一遍) ----
DESC_SHARE="$(ros2 pkg prefix --share unistackbot_description 2>/dev/null)" || { echo "找不到 unistackbot_description, 先 colcon build + source"; exit 1; }
BRINGUP_SHARE="$(ros2 pkg prefix --share unistackbot_bringup 2>/dev/null)" || { echo "找不到 unistackbot_bringup"; exit 1; }
XACRO="$DESC_SHARE/arms/$ROBOT/urdf/$ROBOT.urdf.xacro"
CYAML="$BRINGUP_SHARE/config/${ROBOT}_controllers.yaml"
[ -f "$XACRO" ] || { echo "未知机型 '$ROBOT': $XACRO 不存在 (可用: $(ls "$DESC_SHARE/arms" | tr '\n' ' '))"; exit 1; }
[ -f "$CYAML" ] || { echo "机型 '$ROBOT' 缺控制器配置: $CYAML"; exit 1; }
# 机型验收登记表 (§5.7 定义判据的登记值单一事实源): 形态指纹 + 容差/mimic/软关节收口于此
AYAML="$BRINGUP_SHARE/config/${ROBOT}_acceptance.yaml"
[ -f "$AYAML" ] || { echo "机型 '$ROBOT' 缺验收登记表: $AYAML (建表见 piper_acceptance.yaml 同型)"; exit 1; }
IFS='|' read -r SUBCHAIN_EXP JM_EXP JI_EXP JJ_EXP TJS_MOCK TCM_MOCK TJS_GZ TCM_GZ TJS_MJ TCM_MJ SOFT_GZ MIM_PAIRS <<< "$(python3 - "$AYAML" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
fp, tol = d.get('fingerprint') or {}, d.get('tolerance') or {}
jb = fp.get('joints_by_chain') or {}
print('|'.join([
    str(fp.get('subchain_count', -1)),
    ','.join(jb.get('mock') or []), ','.join(jb.get('ign') or []), ','.join(jb.get('mujoco') or []),
    str(tol.get('js_mock', 0)), str(tol.get('cm_mock', 0)),
    str(tol.get('js_gz', 0)), str(tol.get('cm_gz', 0)),
    str(tol.get('js_mujoco', 0)), str(tol.get('cm_mujoco', 0)),
    (d.get('soft_joints') or {}).get('gz') or '',
    (d.get('mimic') or {}).get('pairs') or '',
]))
PY
)"

# ================= 1. 静态检查 =================
say "== [1] 静态: xacro 展开 + check_urdf =="
URDF=/tmp/verify_${ROBOT}.urdf
if xacro "$XACRO" use_world:=true use_ros2_control:=true > "$URDF" 2>/tmp/verify_xacro_err.txt; then
	ok "xacro 展开成功"
else
	bad "xacro 展开失败: $(cat /tmp/verify_xacro_err.txt | tail -3)"
	exit 1
fi
if check_urdf "$URDF" >/tmp/verify_urdf_tree.txt 2>&1 && grep -q "Successfully Parsed" /tmp/verify_urdf_tree.txt; then
	NCHAIN=$(grep -c 'child' /tmp/verify_urdf_tree.txt)
	if [ "$NCHAIN" -eq "$SUBCHAIN_EXP" ]; then
		ok "check_urdf 解析通过 + 形态指纹② ($NCHAIN 子链 = 登记值)"
	else
		bad "形态指纹②: 子链数=$NCHAIN ≠ 登记值 $SUBCHAIN_EXP (结构漂移: 查 xacro 或更新登记表)"
	fi
else
	bad "check_urdf 失败: $(head -2 /tmp/verify_urdf_tree.txt)"
fi

# 各链关节集+目标推算: <ros2_control> 带 position 命令接口的关节 (与 JointStream 解析口径一致)。
# 目标: 有限位关节 = min + 0.3*(max-min); mimic 关节 (mock 链手指, 无 min/max) =
# multiplier×主关节目标 + offset —— JS 契约要求消息列全命令关节, mimic 也得给值。
# 用法: derive <use_gazebo 值>; 单行输出 "j1,j2 0.1,0.2"
# (单行! read <<< 只消费首行 —— 两行输出的第二行会被静默丢弃, 2026-09-21 实测踩坑)
derive() {
	xacro "$XACRO" use_world:=true use_ros2_control:=true "use_gazebo:=$1" > /tmp/verify_${ROBOT}_$1.urdf 2>/dev/null
	python3 -c "
import xml.etree.ElementTree as ET, sys
lim, mim, order = {}, {}, []
for j in ET.parse('/tmp/verify_${ROBOT}_$1.urdf').iter('joint'):
    if j.get('name') is None or j.find('command_interface') is None: continue
    if (j.find('command_interface').get('name') != 'position'): continue
    name = j.get('name')
    order.append(name)
    ps = {p.get('name'): p.text for p in j.findall('param')}
    if 'min' in ps and 'max' in ps:
        lim[name] = lim.get(name) or (float(ps['min']), float(ps['max']))
    if 'mimic' in ps:
        mim[name] = (ps['mimic'], float(ps.get('multiplier', '1')), float(ps.get('offset', '0')))
if not lim: print('ERROR 无命令关节'); sys.exit(1)
targ = {n: round(lo + 0.3*(hi-lo), 4) for n, (lo, hi) in lim.items()}
for n, (master, k, off) in mim.items():
    if master in targ: targ[n] = round(k*targ[master] + off, 4)
names = [n for n in order if n in targ]
print(','.join(names), ','.join(str(targ[n]) for n in names))" 2>&1
}
read -r JOINS TARGS <<< "$(derive false)"
if [[ "$JOINS" == ERROR* ]]; then bad "mock 目标推算失败: $JOINS"; exit 1; fi
ok "mock 关节集目标推算 ($JOINS)"

# 静态判据③: 各形态命令关节集 = 登记值 (结构漂移在这里拦)
jointset_check() {  # $1=use_gazebo 值 $2=登记集(逗号串)
	local actual
	read -r actual _ <<< "$(derive "$1")"
	if [[ "$actual" == ERROR* ]]; then bad "形态指纹③[$1]: 推算失败 ($actual)"; return; fi
	[ "$actual" = "$2" ] && ok "形态指纹③[$1]: 关节集 = 登记值" \
		|| bad "形态指纹③[$1]: 实际[$actual] ≠ 登记[$2]"
}
jointset_check false "$JM_EXP"
jointset_check ign "$JI_EXP"
jointset_check mujoco "$JJ_EXP"

# CM 链名 (切换检查用, 从 yaml 读)
read -r CM_BASE CM_TIP <<< "$(python3 -c "
import yaml
c = yaml.safe_load(open('$CYAML'))['cartesian_motion_controller']['ros__parameters']
print(c['base_link'], c['tip_link'])")"

# ================= 2. launch 参数化错误路径 =================
say "== [2] 错误路径: 必填参数 / 未知机型 / 未知链 =="
E=$(timeout 15 ros2 launch unistackbot_bringup control.launch.py 2>&1)
echo "$E" | grep -q "required argument 'robot'" && ok "缺 robot 参数显式报错" || bad "缺参未报错: $(echo "$E" | tail -1)"
E=$(timeout 15 ros2 launch unistackbot_bringup control.launch.py robot:="__none__" 2>&1)
echo "$E" | grep -q "未知机型" && ok "未知机型显式报错并列出可用项" || bad "未知机型未报错: $(echo "$E" | tail -1)"
E=$(timeout 15 ros2 launch unistackbot_bringup sim.launch.py chain:=__none__ robot:="$ROBOT" 2>&1)
echo "$E" | grep -q "未知链" && ok "统一入口未知链显式报错" || bad "未知链未报错: $(echo "$E" | tail -1)"

# ---- 公共: 等控制器 (JSB+JS active, CM 已注册) ----
wait_controllers() {  # $1=最长等待秒数 $2=log
	for _ in $(seq 1 "$1"); do
		C=$(timeout 5 ros2 control list_controllers 2>/dev/null)
		if echo "$C" | grep -q "joint_state_broadcaster.*active" && echo "$C" | grep -q "joint_stream_controller.*active"; then
			return 0
		fi
		[ -s "$2" ] && grep -qE "Error|error while" "$2" && return 2
		sleep 1
	done
	return 1
}

# ---- 公共: 一条链的运动验证 (JS 流 + CM 切换) ----
# $1=use_gazebo 值(定关节集) $2=JS容差 $3=CM容差 $4=soft-joints(可空) $5=mimic规格(可空)
run_motion() {
	read -r J T <<< "$(derive "$1")"
	if [[ "$J" == ERROR* ]]; then bad "关节集推算失败 ($1): $J"; return 1; fi
	local SOFT="${4:+--soft-joints $4}"
	local MIM="${5:+--mimic $5}"
	tally "$(timeout 30 python3 "$SCRIPT_DIR/verify_motion.py" js --joints "$J" --targets "$T" --tol "$2" $SOFT $MIM 2>&1)"
	tally "$(timeout 40 python3 "$SCRIPT_DIR/verify_motion.py" cm --base "$CM_BASE" --tip "$CM_TIP" --tol "$3" --timeout 15 2>&1)"
}

# ---- 公共: 收集进程的传递后代 (精确 PID, 勿用 pkill 模式——会误杀宿主) ----
descendants() {  # $1=root pid -> 逐行输出后代 pid
	local frontier="$1" next
	while [ -n "$frontier" ]; do
		next=$(ps -eo pid,ppid --no-headers | awk -v f="$frontier" 'BEGIN{n=split(f,a," "); for(i=1;i<=n;i++) want[a[i]]=1} want[$2]{print $1}')
		echo "$next"
		frontier=$(echo $next)
	done
}

cleanup_launch() {  # $1=launch pid
	# ros2 launch 收到信号后不级联等孙进程: ros2_control_node/robot_state_publisher 实测慢退出 1-2min,
	# 孤儿在场时下一链 spawner 集体 "Failed to find a free participant index" (2026-09-21 4/4 复现实锤,
	# 受控实验: 孤儿在场必挂 / 手动无孤儿 6/6 全活)。故必须等家族真退场再放行下一节。
	[ -n "${1:-}" ] || return 0
	local FAM S alive
	FAM="$(descendants "$1") $1"
	kill -INT "$1" 2>/dev/null
	for _ in $(seq 1 15); do
		alive=""
		for S in $FAM; do
			ST=$(ps -o stat= -p "$S" 2>/dev/null)
			[ -n "$ST" ] && [ "${ST:0:1}" != "Z" ] && alive="$alive $S"  # 僵尸=已死, 勿误判存活
		done
		[ -z "$alive" ] && break
		sleep 1
	done
	for S in $FAM; do  # 仍有存活 -> 精确 PID SIGKILL 兜底
		ST=$(ps -o stat= -p "$S" 2>/dev/null)
		[ -n "$ST" ] && [ "${ST:0:1}" != "Z" ] && kill -KILL "$S" 2>/dev/null
	done
	sleep 2  # 端口/租约落定
	ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
}

# ================= 3. mock 链 =================
say "== [3] mock 链: control.launch.py robot:=$ROBOT =="
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
LOG=/tmp/verify_${ROBOT}_mock.log
ros2 launch unistackbot_bringup control.launch.py "robot:=$ROBOT" > "$LOG" 2>&1 &
LPID=$!
if wait_controllers 40 "$LOG"; then
	ok "控制器就绪 (JSB+JS active)"
else
	bad "控制器 40s 未激活 (见 $LOG: $(grep -iE 'error' "$LOG" | tail -2))"
	cleanup_launch "$LPID"; exit 1
fi
C=$(timeout 5 ros2 control list_controllers 2>/dev/null)
echo "$C" | grep -q "cartesian_motion_controller.*inactive" && ok "CM 以 inactive 注册 (三件套齐)" || bad "CM 未注册为 inactive"
SVCN=$(timeout 5 ros2 service list 2>/dev/null | grep -c "/sim_control/")
[ "$SVCN" -ge 5 ] && ok "/sim_control 服务齐全 (${SVCN}个)" || bad "/sim_control 服务缺 ($SVCN/5+)"
run_motion false "$TJS_MOCK" "$TCM_MOCK"
cleanup_launch "$LPID"

# ================= 4. Fortress 链 (可选) =================
if [ "$WITH_GZ" -eq 1 ]; then
	say "== [4] Fortress 链: ign.launch.py robot:=$ROBOT gui:=false =="
	ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
	LOG=/tmp/verify_${ROBOT}_ign.log
	ros2 launch unistackbot_gazebo ign.launch.py "robot:=$ROBOT" gui:=false > "$LOG" 2>&1 &
	LPID=$!
	if wait_controllers 90 "$LOG"; then
		ok "控制器就绪 (gz_ros2_control 内)"
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
	# 端到端: /sim_control/pause -> 桥接 ControlWorld -> /stats 应真实翻转
	timeout 8 ros2 service call /sim_control/pause std_srvs/srv/Trigger >/dev/null 2>&1
	sleep 1
	P1=$(timeout 6 ros2 topic echo /stats --once 2>/dev/null | grep -m1 "^paused:" | grep -io "true\|false" | head -1)
	timeout 8 ros2 service call /sim_control/resume std_srvs/srv/Trigger >/dev/null 2>&1
	sleep 1
	P2=$(timeout 6 ros2 topic echo /stats --once 2>/dev/null | grep -m1 "^paused:" | grep -io "true\|false" | head -1)
	[ "$P1" = "true" ] && [ "$P2" = "false" ] && ok "/sim_control pause/resume 经桥接真实生效 (true->false)" || bad "pause/resume 未生效: $P1 -> $P2"
	# piper 的 gripper 在 gz 有已知回归 (不响应) —— soft 不计失败 (登记表 soft_joints.gz); xarm7 登记为空
	run_motion ign "$TJS_GZ" "$TCM_GZ" "$SOFT_GZ"
	cleanup_launch "$LPID"
fi

# ================= 5. MuJoCo 链 (可选) =================
if [ "$WITH_MJ" -eq 1 ]; then
	say "== [5] MuJoCo 链: mujoco.launch.py robot:=$ROBOT headless:=true =="
	MJCF="$DESC_SHARE/arms/$ROBOT/mujoco/$ROBOT.xml"
	if [ ! -f "$MJCF" ]; then
		say "跳过: 机型 $ROBOT 无 MJCF 资产 ($MJCF, 生成法见 arms/$ROBOT/mujoco/README.md)"
	else
	ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
	LOG=/tmp/verify_${ROBOT}_mujoco.log
	ros2 launch unistackbot_mujoco mujoco.launch.py "robot:=$ROBOT" headless:=true > "$LOG" 2>&1 &
	LPID=$!
	if wait_controllers 40 "$LOG"; then
		ok "控制器就绪 (mujoco 定制节点内, 500Hz/FIFO80)"
	else
		bad "控制器 40s 未激活 (见 $LOG: $(grep -iE 'error' "$LOG" | tail -2))"
		cleanup_launch "$LPID"; exit 1
	fi
	timeout 6 ros2 topic echo /clock --once >/dev/null 2>&1 && ok "/clock 仿真时间在流" || bad "/clock 无数据"
	# mimic 断言对登记 (§5.7: 被动关节=主关节×multiplier, 对来自登记表 mimic.pairs)
	run_motion mujoco "$TJS_MJ" "$TCM_MJ" "" "$MIM_PAIRS"
	cleanup_launch "$LPID"
	fi
fi

say "结果: PASS=$PASS FAIL=$FAIL (log: /tmp/verify_${ROBOT}_*.log)"
[ "$FAIL" -eq 0 ]

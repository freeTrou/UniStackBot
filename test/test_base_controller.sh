#!/usr/bin/env bash
# test_base_controller.sh —— 底盘域差速夹具 gz 全链 E2E (2026-10-15, 底盘域 A5/A6)。
#
# 夹具: test/fixtures/diffbot (两轮差速 + 脚轮; 不注册机型 —— 框架自测资产)。
# 覆盖: BaseVelocityController 全管道 (cmd_vel→整形→DiffKinematics→velocity 接口)
#       + odom/TF 发布 + 断流超时→零速 + 超限钳位 + 非全向 vy 钳零。
# 断言床: base_controller_bed.py (五场景, PASS/FAIL 行)。
#
# 用法: bash test/test_base_controller.sh [--gui]
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$(dirname "$REPO_DIR")")"   # 仓库嵌套 src/<repo>, ws 根在上两级 (verify_robot.sh 同款)
FIXTURE="$SCRIPT_DIR/fixtures/diffbot"
GUI=0
[ "${1:-}" = "--gui" ] && GUI=1

say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

# 工作区定位 (脚本自定位, 与 verify_robot.sh 同款); ROS setup 引用未绑定变量,
# source 期间临时关闭 nounset (verify_robot.sh 同型坑)
set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash" 2>/dev/null || { say "BAD: 未找到工作区 install (先 colcon build)"; exit 1; }
set -u

# 渲染 URDF (@CONTROLLERS_YAML@ → 绝对路径) —— gz 插件的 <parameters> 只认绝对路径
URDF_RUN=/tmp/diffbot_run.urdf
sed "s|@CONTROLLERS_YAML@|$FIXTURE/diffbot_controllers.yaml|g" "$FIXTURE/diffbot.urdf" > "$URDF_RUN"
# RSP 参数走文件 (CLI -p 传整份 URDF 会撞 RCLInvalidROSArgsError —— rcl 把值当
# ROS 参数语法解析, launch 的 ParameterValue(str) 同因; 2026-10-15 实锤)
RSP_PARAMS=/tmp/diffbot_rsp.yaml
{ echo '/**:'; echo '  ros__parameters:'; echo '    use_sim_time: true'; printf '    robot_description: "%s"\n' "$(sed 's/\\/\\\\/g; s/"/\\"/g' "$URDF_RUN")"; } > "$RSP_PARAMS"

# 清场 (单独执行; 模式安全规则在案)
ros2 run unistackbot_bringup chain_clean.sh >/dev/null 2>&1
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1

WORLD="$(ros2 pkg prefix --share unistackbot_gazebo 2>/dev/null)/worlds/empty_ign.world"
LOG=/tmp/test_base_controller.log
: > "$LOG"

say "起 gz (headless=$([ $GUI = 1 ] && echo false || echo true)) + 桥 + RSP + spawn + spawner"
# gz server 只从 GZ_SIM_SYSTEM_PLUGIN_PATH 找 system 插件 (gz_ros2_control-system 住
# /opt/ros/humble/lib) —— 官方 gz_sim.launch.py 把 LD_LIBRARY_PATH 注入这条路径, 手起
# 脚本须同款 (2026-10-15 实锤: 缺它 = "Failed to load system plugin")
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$LD_LIBRARY_PATH"
GZ_GUI_FLAG=""; [ $GUI = 0 ] && GZ_GUI_FLAG="-s"
ign gazebo -r $GZ_GUI_FLAG "$WORLD" >> "$LOG" 2>&1 &
GZ_PID=$!
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgsClock >> "$LOG" 2>&1 &
BRIDGE_PID=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$RSP_PARAMS" >> "$LOG" 2>&1 &
RSP_PID=$!
sleep 4
ros2 run ros_gz_sim create -file "$URDF_RUN" -name diffbot -x 0 -y 0 -z 0.03 >> "$LOG" 2>&1
sleep 3
timeout 60 ros2 run controller_manager spawner joint_state_broadcaster base_velocity_controller \
	--controller-manager /controller_manager --service-call-timeout 30 >> "$LOG" 2>&1

say "等待控制器 active"
READY=0
for _ in $(seq 1 20); do
	C=$(timeout 8 ros2 control list_controllers 2>/dev/null)
	if echo "$C" | grep -q "base_velocity_controller.*active" && echo "$C" | grep -q "joint_state_broadcaster.*active"; then
		READY=1; break
	fi
	[ -z "$C" ] && ros2 daemon stop >/dev/null 2>&1
	sleep 1
done
if [ $READY = 0 ]; then
	say "BAD: 控制器未激活 (日志 $LOG)"
	tail -20 "$LOG" | cut -c1-180
	kill -INT $RSP_PID $BRIDGE_PID $GZ_PID 2>/dev/null; sleep 3
	ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
	exit 1
fi
say "控制器就绪, 跑五场景断言床"

PASS=0; FAIL=0
while IFS= read -r line; do
	case "$line" in
		PASS:*) PASS=$((PASS+1)); say "$line" ;;
		FAIL:*) FAIL=$((FAIL+1)); say "$line" ;;
		WARN:*) say "$line" ;;
	esac
done < <(timeout 120 python3 "$SCRIPT_DIR/base_controller_bed.py" 2>&1)

say "结果: PASS=$PASS FAIL=$FAIL (log: $LOG)"

# 收场
kill -INT $RSP_PID $BRIDGE_PID $GZ_PID 2>/dev/null
sleep 3
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1
ros2 run unistackbot_bringup chain_clean.sh >/dev/null 2>&1
[ $FAIL -eq 0 ]

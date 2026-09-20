#!/usr/bin/env bash
# 故障注入测试床 (阶段0b, 2026-09-20): mock 链验证 write()/read() 最终防线。
# 场景: F1 断流保持 / F2 NaN命令 / F3 限位外 / F5 超速 (F4 状态跳变由
#       /sim_control set_joint_state 服务测试覆盖, 见 smoke_sim_control.sh)。
# 前置: colcon build + 本机 DDS 配置 (~/cyclonedds.xml)。
# 用法: bash test/fault_injection.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 launch unistackbot_bringup control.launch.py robot:=xarm7 use_rviz:=false > /tmp/fault_launch.log 2>&1 &
LPID=$!
sleep 14
for i in 1 2 3; do
  N=$(timeout 8 ros2 control list_controllers 2>/dev/null | grep joint_stream | grep -c active)
  [ "$N" = "1" ] && break
  timeout 15 ros2 control switch_controllers --activate joint_stream_controller >/dev/null 2>&1
  sleep 2
done
timeout 90 python3 -u "$SCRIPT_DIR/fault_inject_bed.py"
RC=$?
kill $LPID 2>/dev/null
sleep 1
for p in $(ps -eo pid,args | grep "[r]os2_control_node" | awk '{print $1}'); do kill $p 2>/dev/null; done
exit $RC

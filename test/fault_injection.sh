#!/usr/bin/env bash
# 故障注入测试床 (阶段0b, 2026-09-20; 0c 增 F6; 批次3 增 --chain + F7 mimic)。
# 场景: F1 断流保持 / F2 NaN命令 / F3 限位外 / F5 超速 / F6 断流受控减速 / F7 mimic(mujoco)
#       (F4 状态跳变由 /sim_control set_joint_state 服务测试覆盖, 见 smoke_sim_control.sh)。
# 前置: colcon build + 本机 DDS 配置 (~/cyclonedds.xml)。
# 用法: bash test/fault_injection.sh [--chain mock|mujoco]
#   mock   (默认) xarm7 理想执行器 —— 逻辑/防线层
#   mujoco piper 真实动力学 —— 防线在动力学上的表现 + MJCF equality 耦合

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

CHAIN="mock"
[ "${1:-}" = "--chain" ] && CHAIN="${2:?--chain 需要值: mock|mujoco}"

case "$CHAIN" in
  mock)
    # 阈值账 (xarm7): vmax 3.14; F6 目标 5.0 流 1.0s(≈3.1rad) + 刹停 ≤3.1+0.9 → <4.5
    export BED_JOINTS="$(printf 'joint%s,' {1..6})joint7"
    export BED_VMAX=3.14 BED_F6_TARGET=5.0 BED_F6_STREAM=1.0 BED_F6_HI=4.5 BED_F6_LO=2.0
    export BED_MIMIC=""
    ros2 launch unistackbot_bringup control.launch.py robot:=xarm7 use_rviz:=false > /tmp/fault_launch.log 2>&1 &
    LPID=$!
    ;;
  mujoco)
    # 阈值账 (piper): vmax 5.0; joint1 限位 ±2.618 → F6 目标 2.4 流 0.1s(≈0.5rad)
    # + 刹停 ≤0.5+5.0×0.3=2.0 → 区间 (0.8, 2.2); F7 手指 equality ±0.005
    export BED_JOINTS="$(printf 'joint%s,' {1..6})gripper"
    export BED_VMAX=5.0 BED_F6_TARGET=2.4 BED_F6_STREAM=0.1 BED_F6_HI=2.2 BED_F6_LO=0.8
    export BED_MIMIC="gripper:gripper_joint1:0.5,gripper:gripper_joint2:-0.5"
    ros2 launch unistackbot_mujoco mujoco.launch.py robot:=piper headless:=true > /tmp/fault_launch.log 2>&1 &
    LPID=$!
    ;;
  *) echo "未知链 '$CHAIN' (mock|mujoco)"; exit 2 ;;
esac
sleep 18
# 控制器就绪重试 (2026-09-21 批次3): spawner 与 CM 加载竞态时 spawner 会死透
# (load/configure 报 "no controller exists") —— 重试必须补全 load+activate,
# 只 switch 救不回来; 5 轮仍失败则明确失败退出 (死链跑 bed 会产出空洞 PASS, 更糟)。
READY=0
for i in 1 2 3 4 5; do
  C=$(timeout 8 ros2 control list_controllers 2>/dev/null)
  if echo "$C" | grep -q "joint_state_broadcaster.*active" && echo "$C" | grep -q "joint_stream_controller.*active"; then
    READY=1; break
  fi
  for ctl in joint_state_broadcaster joint_stream_controller; do
    echo "$C" | grep -q "$ctl" ||       timeout 20 ros2 control load_controller --set-state active "$ctl" >/dev/null 2>&1
    echo "$C" | grep -q "$ctl.*inactive" &&       timeout 15 ros2 control switch_controllers --activate "$ctl" >/dev/null 2>&1
  done
  sleep 2
done
if [ "$READY" != "1" ]; then
  echo "FAIL: 控制器 5 轮重试未就绪 (启动竞态/DDS 异常, 见 /tmp/fault_launch.log)"
  kill $LPID 2>/dev/null; sleep 1
  for p in $(ps -eo pid,args | grep "[r]os2_control_node" | awk '{print $1}'); do kill $p 2>/dev/null; done
  exit 1
fi
timeout 90 python3 -u "$SCRIPT_DIR/fault_inject_bed.py"
RC=$?
kill $LPID 2>/dev/null
sleep 1
for p in $(ps -eo pid,args | grep "[r]os2_control_node" | awk '{print $1}'); do kill $p 2>/dev/null; done
exit $RC

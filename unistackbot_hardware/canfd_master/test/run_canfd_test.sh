#!/usr/bin/env bash
# CanFdMaster 真机台架 E2E (g++ 直编, 零 ROS; Kvaser can0↔can1 物理环回线)。
# 前置: can0/can1 已配置 UP (bitrate 1M + restart-ms 100)。未配置则打印命令退出。
# vcan 版: ./run_canfd_test.sh vcan0 vcan0 (单接口自打)
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HW="$DIR/../.."
MASTER_IF="${1:-can0}"
MOTOR_IF="${2:-can1}"

for I in "$MASTER_IF" "$MOTOR_IF"; do
	if ! ip link show "$I" up >/dev/null 2>&1; then
		echo "接口 $I 未 UP。配置 (需 sudo, 一次性):"
		echo "  sudo ip link set $I type can bitrate 1000000 restart-ms 100 && sudo ip link set $I up"
		exit 2
	fi
done

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
	"$DIR/test_canfd_master.cpp" \
	"$DIR/../src/canfd_master.cpp" \
	"$HW/protocol/src/dm_motor_core.cpp" \
	"$HW/protocol/src/protocol_factory.cpp" \
	"$HW/protocol/src/unitree_im_core.cpp" \
	"$HW/statemachine/src/dm_safety.cpp" \
	"$HW/statemachine/src/unitree_im_safety.cpp" \
	"$HW/statemachine/src/cia402_safety.cpp" \
	"$HW/statemachine/src/node_safety_factory.cpp" \
	"$HW/bus/src/master_factory.cpp" \
	-I"$DIR/../include" -I"$HW/protocol/include" -I"$HW/statemachine/include" \
	-I"$HW/bus/include" -I"$HW/../unistackbot_common" \
	-pthread -o /tmp/test_canfd_master
/tmp/test_canfd_master "$MASTER_IF" "$MOTOR_IF"

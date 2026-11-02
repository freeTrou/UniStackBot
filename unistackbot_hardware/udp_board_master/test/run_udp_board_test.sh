#!/usr/bin/env bash
# udp_board 帧契约测试 (g++ 直编, 零 ROS; hardware 域先例)
# 用法: bash test/run_udp_board_test.sh
set -euo pipefail
cd "$(dirname "$0")/.."

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread -Iinclude board_protocol.cpp fake_board.cpp \
	test/test_board_protocol.cpp -o /tmp/test_board_protocol
/tmp/test_board_protocol

g++ -std=c++17 -O2 -Wall -Wextra -pthread -Iinclude board_protocol.cpp fake_board.cpp \
	test/test_rtt.cpp -o /tmp/test_udp_board_rtt
/tmp/test_udp_board_rtt

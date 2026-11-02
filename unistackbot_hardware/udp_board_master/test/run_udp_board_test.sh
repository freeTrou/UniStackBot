#!/usr/bin/env bash
# udp_board 主站测试 (2026-11-02 总线/协议拆分后):
#   ① golden 帧测试 → 住 protocol 包 (test/test_board_protocol.cpp, 55 cases)
#   ② RTT 自测 (本包; 主站 socket 往返)
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$DIR/.."
PROTO="$PKG/../protocol"

echo "== [1] 帧契约 golden (住 protocol 包) =="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I"$PROTO/include" \
	"$PROTO/test/test_board_protocol.cpp" "$PROTO/src/board_protocol.cpp" "$PROTO/src/fake_board.cpp" \
	-o /tmp/test_board_protocol
/tmp/test_board_protocol

echo "== [2] RTT (FakeBoard daemon 本地往返) =="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread -I"$PROTO/include" -I"$PKG/include" \
	"$PKG/src/udp_master.cpp" "$PROTO/src/board_protocol.cpp" "$PKG/../statemachine/src/board_safety.cpp" \
	"$PKG/../statemachine/src/node_safety_factory.cpp" "$PKG/../statemachine/src/unitree_im_safety.cpp" \
	"$PKG/../statemachine/src/cia402_safety.cpp" "$PKG/../statemachine/src/dm_safety.cpp" \
	"$PKG/../bus/src/master_factory.cpp" \
	-I"$PKG/../../unistackbot_common" \
	test/test_rtt.cpp -o /tmp/test_udp_board_rtt 2>&1 | head -5
echo "(RTT 用例改写随 M4 —— 当前保留 daemon 独立自测)"

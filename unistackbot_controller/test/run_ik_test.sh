#!/usr/bin/env bash
# dls_ik 五层验证跑批 (无 ssik 依赖——真值读入库文件)。
# 用法: bash test/run_ik_test.sh [robot]   (默认 xarm7; 需先展开 verify URDF)
set -euo pipefail
ROBOT="${1:-xarm7}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(dirname "$PKG_ROOT")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"

URDF="/tmp/verify_${ROBOT}.urdf"
ORACLE="$PKG_ROOT/test/ik_oracle_${ROBOT}.txt"
[ -f "$URDF" ] || { echo "缺 $URDF (先跑 bash test/verify_robot.sh $ROBOT 生成)"; exit 1; }
[ -f "$ORACLE" ] || { echo "缺 $ORACLE (先跑 test/gen_ik_oracle.py)"; exit 1; }

echo "== 编译 =="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion \
	-I"$PKG_ROOT/include" \
	-I/usr/include/eigen3 -I/usr/include -I/opt/ros/humble/include -I/opt/ros/humble/include/kdl_parser -I/opt/ros/humble/include -I"$REPO_ROOT/unistackbot_interface/include" \
	"$PKG_ROOT/test_dls_ik.cpp" \
	"$PKG_ROOT/src/dls_ik.cpp" \
	"$PKG_ROOT/src/urdf_fk.cpp" \
	-L/opt/ros/humble/lib -lkdl_parser -lorocos-kdl -lurdfdom_model -Wl,-rpath,/opt/ros/humble/lib \
	-o /tmp/test_dls_ik

echo "== 运行 ($ROBOT) =="
TIP=link7
[ "$ROBOT" = "piper" ] && TIP=link6
/tmp/test_dls_ik "$URDF" "$ORACLE" "$TIP"

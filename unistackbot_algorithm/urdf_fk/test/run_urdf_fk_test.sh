#!/usr/bin/env bash
# urdf_fk 单测跑批: 展开指定机型 URDF -> 编译测试 -> 运行。
# 零位位置预言的独立手算 (与源码解耦的真值来源):
#   xarm7 q=0, 沿链复合 origin xyz (kinematics yaml 标称):
#     j1 z 0.267 → j2 (rpy -90°x) → j3 y -0.293 → j4 x 0.0525 (rpy 90°x)
#     → j5 x 0.0775, y -0.3425 (rpy 90°x) → j6 (rpy 90°x) → j7 x 0.076, y 0.097 (rpy -90°x)
#   逐段在"旋转后的父系"里平移, 复合结果 = (0.206, 0, 0.1205) —— 与构型判定
#   实测的 j7 轴点一致 (零位时 link7 原点与 j7 轴点重合, rpy 全为 ±90° 互抵)。
# 用法: bash test/run_urdf_fk_test.sh <robot> <tip> <base> [strict]
#   tip/base 必填无默认 (机型拓扑纪律, 2026-09-22): piper 用 link6/base_link, xarm7 用 link7/link_base;
#   strict=零位手算真值断言 (仅 xarm7 有真值)
set -euo pipefail
ROBOT="${1:?用法: run_urdf_fk_test.sh <robot> <tip> <base> [strict]}"
TIP="${2:?用法: run_urdf_fk_test.sh <robot> <tip> <base> [strict]} (机型拓扑无默认值)"
BASE="${3:?缺 base 参数 (例 piper: base_link, xarm7: link_base)}"
STRICT="${4:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"            # urdf_fk/ 算法文件夹
ALGO_PKG="$(dirname "$PKG_ROOT")"              # unistackbot_algorithm 包根 (include 基)
REPO_ROOT="$(dirname "$ALGO_PKG")"             # 仓库根
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"

set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
set -u

SRC="$(ros2 pkg prefix --share unistackbot_description)/arms/$ROBOT/urdf/$ROBOT.urdf.xacro"
URDF=/tmp/fk_test_${ROBOT}.urdf
xacro "$SRC" use_world:=false use_ros2_control:=false > "$URDF" 2>/dev/null

echo "== 编译 =="
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion \
	-I"$ALGO_PKG" \
	"$PKG_ROOT/test_urdf_fk.cpp" \
	"$PKG_ROOT/urdf_fk.cpp" \
	-I/usr/include/eigen3 -I/usr/include -I/opt/ros/humble/include -I/opt/ros/humble/include/kdl_parser -lorocos-kdl -L/opt/ros/humble/lib -lkdl_parser -lurdfdom_model -Wl,-rpath,/opt/ros/humble/lib -o /tmp/test_urdf_fk

echo "== 运行 ($ROBOT) =="
/tmp/test_urdf_fk "$URDF" "$BASE" "$TIP" $STRICT

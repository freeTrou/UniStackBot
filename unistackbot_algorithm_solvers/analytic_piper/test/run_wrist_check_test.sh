#!/usr/bin/env bash
# 球腕指纹判定测试 (g++ 直编, 同 unistackbot_common 组件测试型)。
# 用例: 合成正例/反例 (内置) + piper 真实 URDF 实测 (期望残差 88µm 量级)。
# 前置: unistackbot_description 已构建 (xacro 可用)。
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$DIR/../.."
DESC_SHARE="${UNISTACKBOT_DESC_SHARE:-$(ros2 pkg prefix --share unistackbot_description 2>/dev/null || true)}"
if [ -z "$DESC_SHARE" ]; then
	echo "需要 source 工作区 (ros2 pkg prefix 不可用)"; exit 1
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

xacro "$DESC_SHARE/arms/piper/urdf/piper.urdf.xacro" \
	use_gripper:=false use_ros2_control:=false use_world:=false > "$WORK/piper_arm.urdf"
echo "piper 裸臂已展开: $WORK/piper_arm.urdf"

# 两步编译 (2026-10-26 审核修 C4): 生成物 TU 带 pi_ 系出货 define, 适配层/测试不带
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
	-Wno-ignored-qualifiers -Wno-pedantic -Wno-unused-parameter -Wconversion \
	-DIKFAST_CLIBRARY -DIKFAST_NO_MAIN -DIKFAST_NAMESPACE=piper_ikfast \
	-I"$PKG" -I/usr/include/eigen3 -I/opt/ros/humble/include/urdfdom -I/opt/ros/humble/include/urdfdom_model \
	-c "$PKG/analytic_piper/piper_ikfast.cpp" -o "$WORK/ikfast.o"
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion \
	-I"$PKG" -I"$PKG/../unistackbot_interface/include" -I/usr/include/eigen3 \
	-I/opt/ros/humble/include/kdl_parser -I/opt/ros/humble/include/orocos_kdl \
	-I/opt/ros/humble/include/urdfdom -I/opt/ros/humble/include/urdfdom_model \
	"$DIR/test_wrist_fingerprint.cpp" "$PKG/analytic_piper/analytic_piper.cpp" \
	"$PKG/urdf_fk/urdf_fk.cpp" "$WORK/ikfast.o" -o "$WORK/test_wrist" \
	-lurdfdom_model -lorocos-kdl -lkdl_parser -L/opt/ros/humble/lib
"$WORK/test_wrist" "$WORK/piper_arm.urdf" base_link link6

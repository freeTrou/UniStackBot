#!/usr/bin/env bash
# stationlite 解析解测试 (g++ 直编; piper run_wrist_check_test.sh 同型)。
# 用例: 平行轴指纹 (合成正/反例 + canonical 实测) + FK-IK 回代黄金 200 目标 +
#        限位/seed 一致性/结果码/分支枚举。
# 前置: 无 (canonical URDF 已入库, 不依赖 description 构建)。
# 编译分两步 (2026-10-26 审核修 C4): 生成物 TU 带 -DIKFAST_NAMESPACE=stationlite_ikfast
# (与 colcon 出货配置逐项同构); 适配层/测试 TU 不带 (其 IkReal 为全局 typedef)。
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$DIR/../.."
URDF="$DIR/../gen/stationlite_canonical.urdf"
[ -f "$URDF" ] || { echo "缺 $URDF (gen/ 管线产物)"; exit 1; }

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

COMMON_CXX=(-std=c++17 -O2 -Wall -Wextra -Wpedantic -I"$PKG" -I"$PKG/../unistackbot_interface/include"
	-I/usr/include/eigen3 -I/opt/ros/humble/include/kdl_parser -I/opt/ros/humble/include/orocos_kdl
	-I/opt/ros/humble/include/urdfdom -I/opt/ros/humble/include/urdfdom_model)

g++ "${COMMON_CXX[@]}" \
	-Wno-ignored-qualifiers -Wno-pedantic -Wno-unused-parameter -Wconversion \
	-DIKFAST_CLIBRARY -DIKFAST_NO_MAIN -DIKFAST_NAMESPACE=stationlite_ikfast \
	-c "$PKG/analytic_stationlite/analytic_stationlite_ikfast.cpp" -o "$WORK/ikfast.o"

g++ "${COMMON_CXX[@]}" -Wconversion \
	"$DIR/test_stationlite_ik.cpp" "$PKG/analytic_stationlite/analytic_stationlite.cpp" \
	"$PKG/urdf_fk/urdf_fk.cpp" "$WORK/ikfast.o" -o "$WORK/test_stationlite_ik" \
	-lurdfdom_model -lorocos-kdl -lkdl_parser -L/opt/ros/humble/lib
"$WORK/test_stationlite_ik" "$URDF"

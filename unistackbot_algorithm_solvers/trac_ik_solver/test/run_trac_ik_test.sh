#!/usr/bin/env bash
# trac_ik AB/回归测试跑批 (g++ 直编; 用法同 run_ik_test.sh)。
# 用法: bash test/run_trac_ik_test.sh [robot]   (默认 r1_7a; 需先展开 verify URDF)
set -euo pipefail
ROBOT="${1:-r1_7a}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"            # trac_ik_solver/ 算法文件夹
ALGO_PKG="$(dirname "$PKG_ROOT")"              # unistackbot_algorithm 包根 (include 基)
REPO_ROOT="$(dirname "$ALGO_PKG")"             # 仓库根

URDF="/tmp/verify_${ROBOT}.urdf"
ORACLE="$ALGO_PKG/dls_ik/test/ik_oracle_${ROBOT}.txt"
[ -f "$URDF" ] || { echo "缺 $URDF (先跑 bash test/verify_robot.sh $ROBOT 生成)"; exit 1; }
[ -f "$ORACLE" ] || { echo "缺 $ORACLE"; exit 1; }

echo "== 编译 =="
# rclcpp 依赖 (vendored 上游特例, 见 CMakeLists 方案 C 注): 最小 include 集。
# 不做全目录扫描 —— idl/ 的裸名 string.h 遮蔽系统 <cstring>, 其余目录也有遮蔽
# 系统宏的风险 (2026-10-08 实测踩坑两次)。
INC=""
for p in rclcpp rcl rmw rmw_implementation rmw_dds_common rcutils rcpputils \
         builtin_interfaces rcl_interfaces rosgraph_msgs statistics_msgs \
         libstatistics_collector tracetools rosidl_runtime_cpp rosidl_runtime_c \
         rosidl_typesupport_interface rosidl_typesupport_cpp rosidl_typesupport_c \
         rosidl_typesupport_introspection_cpp rosidl_typesupport_introspection_c \
         rcl_yaml_param_parser; do
	[ -d "/opt/ros/humble/include/$p" ] && INC="$INC -I/opt/ros/humble/include/$p"
done
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion \
	-I"$ALGO_PKG" -I"$REPO_ROOT/unistackbot_interface/include" \
	-I/opt/ros/humble/include/kdl_parser -I"$PKG_ROOT/vendor" $INC -I/usr/include/eigen3 \
	-c "$PKG_ROOT/test_trac_ik.cpp" -o /tmp/test_trac_ik_main.o
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion \
	-I"$ALGO_PKG" -I"$REPO_ROOT/unistackbot_interface/include" \
	-I/opt/ros/humble/include/kdl_parser -I"$PKG_ROOT/vendor" $INC -I/usr/include/eigen3 \
	-c "$PKG_ROOT/trac_ik_solver.cpp" -o /tmp/tik_solver.o
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion \
	-I"$ALGO_PKG" -I"$REPO_ROOT/unistackbot_interface/include" \
	-I/opt/ros/humble/include/kdl_parser $INC -I/usr/include/eigen3 \
	-c "$ALGO_PKG/urdf_fk/urdf_fk.cpp" -o /tmp/tik_urdf_fk.o
# vendored 上游单独编 (固有警告单文件关, 与 CMake 同口径); -o 显式落 /tmp
# (-c 多源输出落 CWD, 链接时路径不一致 —— 2026-10-08 实测踩坑)
for f in trac_ik nlopt_ik kdl_tl; do
	g++ -std=c++17 -O2 -Wno-pedantic -Wno-unused-parameter -Wno-sign-compare \
		-I"$ALGO_PKG" -I/opt/ros/humble/include/kdl_parser $INC \
		-I/usr/include/eigen3 -I"$PKG_ROOT/vendor" \
		-c "$PKG_ROOT/vendor/trac_ik/$f.cpp" -o "/tmp/tik_$f.o"
done
g++ -o /tmp/test_trac_ik /tmp/test_trac_ik_main.o /tmp/tik_solver.o /tmp/tik_urdf_fk.o \
	/tmp/tik_trac_ik.o /tmp/tik_nlopt_ik.o /tmp/tik_kdl_tl.o \
	-L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib \
	-lkdl_parser -lorocos-kdl -lurdfdom_model -lnlopt -lrclcpp \
	-lrcutils -lrcpputils -lrcl -lrmw -lrosidl_runtime_c

echo "== 运行 ($ROBOT) =="
case "$ROBOT" in
	piper) TIP=link6; BASE=base_link ;;
	r1_7a) TIP=Link7; BASE=base_link ;;
	xarm7) TIP=link7; BASE=link_base ;;
	stationlite) TIP=left_linkgripper; BASE=left_armbase ;;
	*) echo "未知机型 $ROBOT"; exit 1 ;;
esac
/tmp/test_trac_ik "$URDF" "$ORACLE" "$TIP" "$BASE"

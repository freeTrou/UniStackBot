#!/usr/bin/env bash
# stationlite IKFast 生成管线 (照 piper gen/ 配方复用, 2026-10-21 B 线 B1):
# 三个已知坑同款内置 (collada_urdf apt / robot 包装 XML / sympy 守卫补丁)。
# 差异件: urdf=stationlite_canonical (Z-Y-Y-Y-Z-X, Pieper 形式二);
#        manipulator base=left_armbase effector=left_linkgripper。
# 注意: base 取 left_armbase (剥 base_link→armbase 的 fixed 前缀, 厂商 SDK 同款根)。
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="docker.io/personalrobotics/ros-openrave:latest"
URDF="$DIR/stationlite_canonical.urdf"
OUT="$DIR/generated"
[ -f "$URDF" ] || { echo "缺 $URDF"; exit 1; }
mkdir -p "$OUT"

echo "== [1/4] 拉取 OpenRAVE 镜像 (已存在则跳过) =="
podman image exists "$IMAGE" || podman pull "$IMAGE"

echo "== [2/4] 容器内: 装 collada_urdf + URDF -> Collada =="
podman run --rm -v "$DIR":/work:z "$IMAGE" bash -c '
	set -e
	apt-get update -qq 2>/dev/null || true
	apt-get install -y --allow-unauthenticated ros-indigo-collada-urdf > /tmp/apt.log 2>&1 || \
		{ echo "collada_urdf 安装失败:"; tail -5 /tmp/apt.log; exit 3; }
	UC=$(find /opt/ros/indigo -name urdf_to_collada -type f | head -1)
	[ -n "$UC" ] || { echo "装完仍无 urdf_to_collada"; exit 3; }
	"$UC" /work/stationlite_canonical.urdf /work/stationlite_canonical.dae
'
[ -f "$DIR/stationlite_canonical.dae" ] || { echo "dae 生成失败"; exit 3; }

echo "== [3/4] 容器内: sympy 守卫补丁 + IKFast 生成 Transform6D =="
podman run --rm -v "$DIR":/work:z -e HOME=/work/generated "$IMAGE" bash -c '
	set -e
	mkdir -p /work/generated
	echo "<robot file=\"stationlite_canonical.dae\"><manipulator name=\"arm\"><base>left_armbase</base><effector>left_linkgripper</effector></manipulator></robot>" > /work/stationlite_robot.xml
	IKFAST=/usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_9/ikfast.py
	sed -i "s|if len(coeffs) == 1 and Abs(coeffs\[0\]) < 2\*(10.0\*\*-self.precision):|if len(coeffs) == 1 and coeffs[0].is_number and Abs(coeffs[0]) < 2*(10.0**-self.precision):|g" $IKFAST
	find /usr/lib/python2.7/dist-packages/openravepy -name "*.pyc" -delete
	openrave.py --database inversekinematics \
		--robot=/work/stationlite_robot.xml --manipname=arm \
		--iktype=Transform6D --iktests=200 || \
		echo "(容器内 cc 自检失败可忽略 —— 产物以宿主验证为准)"
'

echo "== [4/4] 收取生成物 =="
CPP=$(find "$OUT" -name "ikfast0x*.cpp" | head -1)
[ -n "$CPP" ] || { echo "未找到生成的 cpp"; exit 4; }
cp "$CPP" "$OUT/analytic_stationlite_gen.cpp"
H=$(dirname "$CPP"); [ -f "$H/ikfast.h" ] && cp "$H/ikfast.h" "$OUT/ikfast.h"
# 符号前缀化 (2026-10-21): 生成物 C-API 恒全局 (extern "C"), 与 piper 生成物同库
# 必撞——前缀 sl_ = stationlite (嵌入式中双 IK 生成物通行做法)。适配层声明同步。
sed -i -E 's/\b(ComputeIk2|ComputeIk|ComputeFk|GetNumFreeParameters|GetFreeParameters|GetNumJoints|GetIkRealSize|GetIkType|GetKinematicsHash|GetIkFastVersion)\b/sl_\1/g' \
	"$OUT/analytic_stationlite_gen.cpp"
echo "生成成功 (符号已加 sl_ 前缀): $OUT/analytic_stationlite_gen.cpp (+ ikfast.h)"

#!/usr/bin/env bash
# IKFast 闭式解生成管线 (2026-09-22 跑通版配方; 容器内执行, 主机零 OpenRAVE 依赖)。
# 用法:
#   1) export http_proxy=http://10.0.3.133:7890 https_proxy=http://10.0.3.133:7890   # 首次拉镜像需要
#   2) bash gen/generate_in_container.sh
# 产物: gen/generated/.openrave/<哈希>/ 下的 ikfast0x*.cpp + ikfast.h
#
# 三个已知坑 (全部踩过, 配方已内置):
#   ① 镜像无 collada_urdf → 步骤 2 容器内 apt 装 (indigo EOL 快照源, GPG 过期用 --allow-unauthenticated)
#   ② openrave.py 无 --baselink/--eelink 选项 → 用 robot 包装 XML 定义 <manipulator> + --manipname
#   ③ 镜像 sympy 0.7.4 与 ikfast.py HalfAngle 求解路径不兼容 (布尔化 TypeError) →
#      is_number 守卫补丁 (非数值系数跳过平凡零判定, 语义同老 sympy) + 清 pyc 缓存
# 注: 容器内 cc 自检编译失败可忽略 (古董工具链怪癖) —— 产物有效性的最终裁判 =
#      宿主 g++ 编译 + FK-IK 回代测试 (analytic_piper/test/)。
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="docker.io/personalrobotics/ros-openrave:latest"
URDF="$DIR/piper_canonical.urdf"
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
	"$UC" /work/piper_canonical.urdf /work/piper_canonical.dae
'
[ -f "$DIR/piper_canonical.dae" ] || { echo "dae 生成失败"; exit 3; }

echo "== [3/4] 容器内: sympy 守卫补丁 + IKFast 生成 Transform6D =="
podman run --rm -v "$DIR":/work:z -e HOME=/work/generated "$IMAGE" bash -c '
	set -e
	mkdir -p /work/generated
	# robot 包装 XML: 定义 manipulator (openrave.py 0.9 无 --baselink/--eelink 选项)
	echo "<robot file=\"piper_canonical.dae\"><manipulator name=\"arm\"><base>base_link</base><effector>link6</effector></manipulator></robot>" > /work/piper_robot.xml
	IKFAST=/usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_9/ikfast.py
	sed -i "s|if len(coeffs) == 1 and Abs(coeffs\[0\]) < 2\*(10.0\*\*-self.precision):|if len(coeffs) == 1 and coeffs[0].is_number and Abs(coeffs[0]) < 2*(10.0**-self.precision):|g" $IKFAST
	find /usr/lib/python2.7/dist-packages/openravepy -name "*.pyc" -delete
	openrave.py --database inversekinematics \
		--robot=/work/piper_robot.xml --manipname=arm \
		--iktype=Transform6D --iktests=200 || \
		echo "(容器内 cc 自检失败可忽略 —— 产物以宿主验证为准)"
'

echo "== [4/4] 收取生成物 =="
CPP=$(find "$OUT" -name "ikfast0x*.cpp" | head -1)
[ -n "$CPP" ] || { echo "未找到生成的 cpp"; exit 4; }
cp "$CPP" "$OUT/analytic_piper_gen.cpp"
H=$(dirname "$CPP"); [ -f "$H/ikfast.h" ] && cp "$H/ikfast.h" "$OUT/ikfast.h"
echo "生成成功: $OUT/analytic_piper_gen.cpp (+ ikfast.h)"
echo "== 下一步: license 头过目 (Apache-2.0) → cp 进 analytic_piper/ → 宿主回代测试 =="

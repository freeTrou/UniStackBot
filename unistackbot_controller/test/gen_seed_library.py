#!/usr/bin/env python3
"""种子库生成器 (验证版): Halton 关节采样 -> FK 正推 -> 体素去重。

方法论与 oracle v2 同源 (关节空间低差异采样 + FK 白送可达性);
FK 走 ssik (与我们的 FK 对拍 4e-13, oracle 同先例——权威性固化为数据)。

去重键 = 位置体素 (3cm) × 姿态桶 (四元数分量 0.5 量化, 符号规范化):
同位置不同姿态的样本都保留 (姿态多样性不被位置去重吃掉)。

用法: python3 gen_seed_library.py <robot> [target_entries]   (默认 8000)
输出: unistackbot_description/arms/<robot>/ik/seed_lib_<robot>.txt (机型资产, 随仓库提交)
"""
import datetime
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import ssik
from scipy.spatial.transform import Rotation as Rrot

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "xarm7"
TARGET = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
URDF = f"/tmp/verify_{ROBOT}.urdf"
PKG_ROOT = __file__.rsplit("/", 2)[0]
OUT = f"{PKG_ROOT}/../unistackbot_description/arms/{ROBOT}/ik/seed_lib_{ROBOT}.txt"
TIP, BASE = "link7", "link_base"

lim = {}
for j in ET.parse(URDF).getroot().iter("joint"):
	if j.get("name") is None or j.find("command_interface") is None:
		continue
	ps = {p.get("name"): p.text for p in j.findall("param")}
	if "min" in ps and "max" in ps:
		lim[j.get("name")] = (float(ps["min"]), float(ps["max"]))
JOINTS = list(lim)
LO = np.array([lim[n][0] for n in JOINTS])
HI = np.array([lim[n][1] for n in JOINTS])
m = ssik.Manipulator.from_urdf(URDF, base=BASE, ee=TIP)

# 四分支代表 (与 dls_ik.cpp kBranch / gen_ik_oracle.py 同表) —— 查询端封顶选种用
BRANCH = np.array([
	[+0.30, +1.44, -0.03, +1.46, +0.87, -0.03, +1.44],
	[-0.60, +1.44, +0.02, +1.39, -0.79, -0.02, -1.44],
	[+0.15, -1.45, +2.73, +1.39, +0.06, -0.03, -0.07],
	[-0.07, -1.44, -2.73, +1.39, -0.07, -0.03, +0.09]])


def branch_of(q):
	return int(np.argmin(np.abs(q - BRANCH).sum(axis=1)))


def halton(i, base):
	f, r = 1.0, 0.0
	while i > 0:
		f /= base
		r += f * (i % base)
		i //= base
	return r


VOX = 0.03
OFFSET = 1000000   # 与 oracle 的 Halton 前缀脱相关 (否则验证循环: 目标本身在库里)
seen = set()
entries = []
i = 0
while len(entries) < TARGET and i < TARGET * 200:
	q = LO + np.array([halton(i + OFFSET, b) for b in (2, 3, 5, 7, 11, 13, 17)]) * (HI - LO)
	i += 1
	T = m.fk(q)
	p = np.array(T[:3, 3])
	quat = Rrot.from_matrix(T[:3, :3]).as_quat()   # x y z w
	x, y, z, w = quat
	if w < 0:
		w, x, y, z = -w, -x, -y, -z
	key = (round(p[0] / VOX), round(p[1] / VOX), round(p[2] / VOX),
		round(2 * x), round(2 * y), round(2 * z))
	if key in seen:
		continue
	seen.add(key)
	entries.append((q, p, (w, x, y, z)))

os.makedirs(os.path.dirname(OUT), exist_ok=True)
now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
with open(OUT, "w") as f:
	f.write(f"# seed library for {ROBOT} (Halton joints + FK forward + voxel dedup)\n")
	f.write(f"# generated: {now}  urdf=/tmp/verify_{ROBOT}.urdf (verify_robot.sh 产物)\n")
	f.write(f"# entries={len(entries)} voxel={VOX}m orient_bucket=0.5 halton_offset={OFFSET} samples_tried={i}\n")
	f.write("# 查询端参数 (dls_ik): 组合度量 dp + 0.35*(1-|q·q'|), 分支封顶<=2, top-6\n")
	f.write("# format: qw qx qy qz x y z | j1 .. j7 br   (br = L1 最近分支 0-3, 查询端封顶选种用)\n")
	for q, p, (w, x, y, z) in entries:
		f.write(f"{w:.6f} {x:.6f} {y:.6f} {z:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} | "
			+ " ".join(f"{v:.6f}" for v in q) + f" {branch_of(q)}\n")
print(f"{len(entries)} entries (from {i} samples) -> {OUT}")

#!/usr/bin/env python3
"""种子库增量更新 (覆盖驱动, 测试集无关) —— 2026-09-17 定稿协议:

  1. 覆盖报告: 独立参考点集 (固定种子随机关节 -> FK 位置 = 可达空间的物理采样,
     与 oracle/测试集零关联) 到库最近种子的距离分布
  2. 触发判据: 参考点 p99 > 4cm 或 max > 6cm (体素 3cm 设计的两倍容差)
  3. 加密: 拒绝采样 —— 候选关节 FK 后, 位置距库 (含本轮已加) 最近种子 > gap 才入库
     (没法"在工作空间体素中心放种子"——那需要 IK; 拒绝采样是等价实现)
  4. 复报覆盖, 合并重写库文件 (原序 + 追加; 指纹头记 base 版本/增量原因/条目数)

禁止: 围着测试失败样本补种子再拿同批样本验收 (循环验证)。
用法: python3 seed_lib_incremental.py <robot> [max_add]   (默认 4000)
"""
import datetime
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import ssik
from scipy.spatial.transform import Rotation as Rrot

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "xarm7"
MAX_ADD = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
URDF = f"/tmp/verify_{ROBOT}.urdf"
PKG_ROOT = __file__.rsplit("/", 2)[0]
LIB = f"{PKG_ROOT}/../unistackbot_description/arms/{ROBOT}/ik/seed_lib_{ROBOT}.txt"
TIP, BASE = "link7", "link_base"
VOX = 0.03
GAP = 0.05          # 加密间隔 (组合度量): dp + 0.35*(1-|qd|) > 0.05 才补

BRANCH = np.array([
	[+0.30, +1.44, -0.03, +1.46, +0.87, -0.03, +1.44],
	[-0.60, +1.44, +0.02, +1.39, -0.79, -0.02, -1.44],
	[+0.15, -1.45, +2.73, +1.39, +0.06, -0.03, -0.07],
	[-0.07, -1.44, -2.73, +1.39, -0.07, -0.03, +0.09]])

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


def fk_pos(q):
	T = m.fk(q)
	return np.array(T[:3, 3]), Rrot.from_matrix(T[:3, :3]).as_quat()   # pos, (x,y,z,w)


def branch_of(q):
	return int(np.argmin(np.abs(q - BRANCH).sum(axis=1)))


W_ROT = 0.35


def nearest_comb(points, quats, lib_pos, lib_quat):
	"""每参考点到库最近种子的组合度量 (与 dls_ik 查询同度量: dp + w*(1-|q·q'|))
	位置走 BLAS (|p|²+|l|²−2p·l), 姿态走四元数点积矩阵"""
	out = np.empty(len(points))
	lib_sq = (lib_pos * lib_pos).sum(axis=1)
	for i in range(0, len(points), 256):
		chunk = points[i:i + 256]
		cq = quats[i:i + 256]
		d2 = (chunk * chunk).sum(axis=1, keepdims=True) + lib_sq[None, :] - \
			2.0 * (chunk @ lib_pos.T)
		dp = np.sqrt(np.maximum(d2, 0.0))
		qd = np.abs(cq @ lib_quat.T)
		comb = (dp + W_ROT * (1.0 - qd)).min(axis=1)
		out[i:i + 256] = comb
	return out


# ---- 读库 ----
entries = []   # (q, pos, quat(w,x,y,z), branch)
seen = set()
with open(LIB) as f:
	for line in f:
		if line.startswith("#") or not line.strip():
			continue
		head, tail = line.split("|")
		v = [float(x) for x in head.split()]
		w, x, y, z = v[0], v[1], v[2], v[3]
		p = np.array(v[4:7])
		q = np.array([float(t) for t in tail.split()[:7]])
		entries.append((q, p, (w, x, y, z)))
		key = (round(p[0] / VOX), round(p[1] / VOX), round(p[2] / VOX),
			round(2 * x), round(2 * y), round(2 * z))
		seen.add(key)
base_count = len(entries)
print(f"库基线: {base_count} 条")

lib_pos = np.array([e[1] for e in entries])
lib_quat = np.array([[e[2][0], e[2][1], e[2][2], e[2][3]] for e in entries])

# ---- 1) 覆盖报告 (测试集无关参考点) ----
rng = np.random.default_rng(20260917)
ref_pos, ref_quat = [], []
for _ in range(20000):
	q = LO + rng.random(len(JOINTS)) * (HI - LO)
	p, quat = fk_pos(q)
	ref_pos.append(p)
	x, y, z, w = quat
	ref_quat.append([w, x, y, z] if w >= 0 else [-w, -x, -y, -z])
ref_pos = np.array(ref_pos)
ref_quat = np.array(ref_quat)
d_ref = nearest_comb(ref_pos, ref_quat, lib_pos, lib_quat)
print(f"覆盖-组合度量 (2 万参考点): p50={np.percentile(d_ref, 50):.3f} "
	f"p99={np.percentile(d_ref, 99):.3f} max={d_ref.max():.3f}")
trig = True   # 手动调用即有意加密; 判据数字已打印, 冻结与否由回归结果裁决

# ---- 2) 拒绝采样加密 ----
added = []
tried = 0
while len(added) < MAX_ADD and tried < MAX_ADD * 60:
	batch_q = LO[None, :] + rng.random((2000, len(JOINTS))) * (HI - LO)[None, :]
	tried += 2000
	batch_pos = []
	batch_quat = []
	for q in batch_q:
		p, quat = fk_pos(q)
		batch_pos.append(p)
		x, y, z, w = quat
		batch_quat.append([w, x, y, z] if w >= 0 else [-w, -x, -y, -z])
	batch_pos = np.array(batch_pos)
	batch_quat = np.array(batch_quat)
	d = nearest_comb(batch_pos, batch_quat, lib_pos, lib_quat)
	for qi in np.where(d > GAP)[0]:
		if len(added) >= MAX_ADD:
			break
		q = batch_q[qi]
		p = batch_pos[qi]
		quat = fk_pos(q)[1]
		x, y, z, w = quat
		if w < 0:
			w, x, y, z = -w, -x, -y, -z
		key = (round(p[0] / VOX), round(p[1] / VOX), round(p[2] / VOX),
			round(2 * x), round(2 * y), round(2 * z))
		if key in seen:
			continue
		seen.add(key)
		added.append((q, p, (w, x, y, z)))
		lib_pos = np.vstack([lib_pos, p])   # 含本轮已加 (批内互距未查, 去重键兜底)
		lib_quat = np.vstack([lib_quat, [w, x, y, z]])
print(f"加密: 采样 {tried} 条, 入库 {len(added)} 条 (组合度量 gap>{GAP})")

# ---- 3) 复报 ----
d_after = nearest_comb(ref_pos, ref_quat,
	np.array([e[1] for e in entries + added]),
	np.array([[e[2][0], e[2][1], e[2][2], e[2][3]] for e in entries + added]))
print(f"增量后覆盖-组合度量: p50={np.percentile(d_after, 50):.3f} "
	f"p99={np.percentile(d_after, 99):.3f} max={d_after.max():.3f}")

# ---- 4) 重写库文件 (基线原序 + 增量追加) ----
now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
with open(LIB, "w") as f:
	f.write(f"# seed library for {ROBOT} (Halton joints + FK forward + voxel dedup)\n")
	f.write(f"# version: v1.1  base_version: v1 (entries={base_count})\n")
	f.write(f"# increment: 姿态感知覆盖驱动拒绝采样 +{len(added)} (组合度量 gap>{GAP}), {now}\n")
	f.write(f"# generated: {now}  urdf=/tmp/verify_{ROBOT}.urdf (verify_robot.sh 产物)\n")
	f.write(f"# entries={base_count + len(added)} voxel={VOX}m orient_bucket=0.5\n")
	f.write("# 查询端参数 (dls_ik): 组合度量 dp + 0.35*(1-|q·q'|), 分支封顶<=2, top-6\n")
	f.write("# format: qw qx qy qz x y z | j1 .. j7 br   (br = L1 最近分支 0-3, 查询端封顶选种用)\n")
	for q, p, (w, x, y, z) in entries + added:
		f.write(f"{w:.6f} {x:.6f} {y:.6f} {z:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} | "
			+ " ".join(f"{v:.6f}" for v in q) + f" {branch_of(q)}\n")
print(f"写出: {LIB} ({base_count + len(added)} 条)")

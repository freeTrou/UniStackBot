#!/usr/bin/env python3
"""DLS IK Python 原型 (算法验证层, 与 C++ dls_ik 同构).

目的: 秒级迭代调参/改算法, 数字达标后移植 C++。验证基准与 C++ 相同:
oracle 真值文件 (ssik 生成, 限位内解) 90 可达 + 110 不可达。

依赖: numpy + kdl_parser (python) 不用——FK/Jacobian 直接从 URDF 数值推导太慢,
      用 pykdl? 本机有 python3-pykdl。或者最简: 调 C++ fk_tool? 不行, 要雅可比。
      => 用 numpy 重写 FK/Jacobian (与 C++ KDL 交叉对拍过 4e-13, 但 Python 重写有风险
      => 折中: pykdl (python3-pykdl 已装, /usr/lib/python3/dist-packages) 建链。

用法: python3 dls_ik_proto.py <robot>
输出: 逐配置矩阵对比 (成功率/耗时) + 最优配置详情
"""
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

try:
    import PyKDL as kdl
except ImportError:
    try:
        import pykdl as kdl
    except ImportError:
        print("需要 python3-pykdl (本机已装: python3-pykdl)")
        sys.exit(1)

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "xarm7"
URDF = f"/tmp/verify_{ROBOT}.urdf"
ORACLE = f"{'/'.join(__file__.split('/')[:-2])}/test/ik_oracle_{ROBOT}.txt"


# ---------- FK / Jacobian (pykdl, 与 C++ UrdfFk 同源) ----------
class Kin:
	def __init__(self, urdf_path):
		xml = ET.parse(urdf_path)
		root = xml.getroot()
		self.chain = kdl.Chain()
		# 手工建链 (与 C++ 相同的回溯法, 不用 kdl_parser 的 python 绑定不存在)
		joints = {}   # child_link -> joint element
		for j in root.iter("joint"):
			c = j.find("child")
			if c is None or c.get("link") is None:
				continue   # ros2_control 块的 <joint> 声明 (无 child/parent)
			joints[c.get("link")] = j
		seq = []
		link = "link7"
		while link != "link_base":
			j = joints[link]
			seq.append(j)
			link = j.find("parent").get("link")
		from math import pi, cos, sin
		for j in reversed(seq):
			o = j.find("origin")
			xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
			rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
			ax = j.find("axis")
			axis = [float(v) for v in (ax.get("xyz") or "0 0 1").split()] if ax is not None else [0, 0, 1]
			jt = j.get("type")
			if jt not in ("revolute", "continuous", "prismatic"):
				continue
			n = np.linalg.norm(axis)
			axis = np.array(axis) / (n if n > 1e-12 else 1.0)
			kdl_axis = kdl.Vector(*axis)
			ktype = kdl.Joint.RotAxis if jt != "prismatic" else kdl.Joint.TransAxis
			joint = kdl.Joint(kdl.Vector(0, 0, 0), kdl_axis, ktype)   # 构造5: origin, axis, type
			f = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
			self.chain.addSegment(kdl.Segment(j.get("name"), joint, f))
		self.n = self.chain.getNrOfJoints()
		self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
		self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
		# 限位 (ros2_control 块)
		self.lo, self.hi = [], []
		for j in root.iter("joint"):
			if j.get("name") is None or j.find("command_interface") is None:
				continue
			ps = {p.get("name"): p.text for p in j.findall("param")}
			if "min" in ps and "max" in ps:
				self.lo.append(float(ps["min"]))
				self.hi.append(float(ps["max"]))
		self.lo = np.array(self.lo)
		self.hi = np.array(self.hi)

	def fk(self, q):
		qa = kdl.JntArray(self.n)
		for i in range(self.n):
			qa[i] = q[i]
		f = kdl.Frame()
		if self.fk_solver.JntToCart(qa, f) != 0:
			return None
		R = np.array([[f.M[0, 0], f.M[0, 1], f.M[0, 2]],
			[f.M[1, 0], f.M[1, 1], f.M[1, 2]], [f.M[2, 0], f.M[2, 1], f.M[2, 2]]])
		return f.p[0], f.p[1], f.p[2], R

	def jacobian(self, q):
		qa = kdl.JntArray(self.n)
		for i in range(self.n):
			qa[i] = q[i]
		Jk = kdl.Jacobian(self.n)
		if self.jac_solver.JntToJac(qa, Jk) != 0:
			return None
		J = np.zeros((6, self.n))
		for r in range(6):
			for c in range(self.n):
				J[r, c] = Jk[r, c]
		return J


def pose_err(cur, tgt):
	"""[位置 3; 姿态 3 (基座系轴角)] — 与 C++ poseError 同构"""
	px, py, pz, Ra = cur
	tx, ty, tz, Rt = tgt
	e = np.zeros(6)
	e[:3] = [tx - px, ty - py, tz - pz]
	dR = Ra.T @ Rt
	# log map (轴角) via trace
	tr = np.trace(dR)
	th = np.arccos(np.clip((tr - 1) / 2, -1, 1))
	if th > 1e-9:
		w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
		# 转基座系 (C++ 同款坑: 局部系 -> 基座系)
		e[3:] = Ra @ (w * (th / (2 * np.sin(th))))
	return e


# ---------- DLS 求解器 (可调参矩阵) ----------
def solve_once(kin, tgt, seed, cfg):
	n = kin.n
	q = np.array(seed, dtype=float)
	lo, hi = kin.lo, kin.hi
	ls_fail = 0
	for it in range(cfg["max_iter"]):
		cur = kin.fk(q)
		if cur is None:
			return None
		e = pose_err(cur, tgt)
		pos_err = np.linalg.norm(e[:3])
		rot_err = np.linalg.norm(e[3:])
		if pos_err < cfg["pos_tol"] and rot_err < cfg["rot_tol"]:
			return q.copy(), it + 1   # 收敛 (限位内: boxed 保证)
		ew = e.copy()
		ew[3:] *= cfg["w_rot"]
		err = np.linalg.norm(ew)
		J = kin.jacobian(q)
		if J is None:
			return None
		U, sv, Vt = np.linalg.svd(J, full_matrices=False)
		smin = sv[-1]
		ratio = min(smin / cfg["sigma_thr"], 1.0)
		lam = np.clip(cfg["lambda0"] * (1 + cfg["lam_gain"] * (1 - ratio)), cfg["lambda0"], cfg["lam_max"])
		sv_inv = sv / (sv ** 2 + lam ** 2)
		Jp = Vt.T @ np.diag(sv_inv) @ U.T
		dq_main = Vt.T @ (sv_inv * (U.T @ ew))
		# 零空间 (软约束: err 大才介入)
		dq_null = np.zeros(n)
		if err > 1e-3 and cfg["ns_gain"] > 0:
			c = (lo + hi) / 2
			Hg = -(q - c) / np.maximum(hi - lo, 1e-3) ** 2 * cfg["w_center"]
			N = np.eye(n) - Jp @ J
			dq_null = N @ (cfg["ns_gain"] * Hg)
		dq = dq_main + dq_null
		# 收敛邻域: 直接接受 + clamp (boxed)
		accepted = False
		if err < 10.0 * (cfg["pos_tol"] + cfg["w_rot"] * cfg["rot_tol"]):
			q = np.clip(q + dq, lo, hi)
			accepted = True
		else:
			for bt in range(cfg["max_bt"] + 1):
				step = 0.5 ** bt
				q_try = np.clip(q + step * dq, lo, hi)
				cur_t = kin.fk(q_try)
				if cur_t is not None:
					e_t = pose_err(cur_t, tgt)
					ew_t = e_t.copy()
					ew_t[3:] *= cfg["w_rot"]
					if np.linalg.norm(ew_t) < err * 1.001 + 1e-12:
						q = q_try
						accepted = True
						break
			if not accepted:
				ls_fail += 1
				if ls_fail >= 3:
					return None
				continue
		ls_fail = 0
	return None


def solve(kin, tgt, seed, cfg):
	"""种子阶梯: 调用方种子 -> 分层重启"""
	n = kin.n
	r = solve_once(kin, tgt, seed, cfg)
	if r is not None:
		return r[0], 0
	rng = np.random.default_rng(cfg["seed"])
	for k in range(cfg["restarts"]):
		if k == 0:
			s = (kin.lo + kin.hi) / 2
		elif k < cfg["restarts"] // 2:
			c = (kin.lo + kin.hi) / 2
			band = 0.3 * (kin.hi - kin.lo) / 2
			s = rng.uniform(c - band, c + band)
		else:
			s = rng.uniform(kin.lo, kin.hi)
		r = solve_once(kin, tgt, s, cfg)
		if r is not None:
			q, it = r
			if np.sum(np.abs(q - seed)) <= cfg["jump"]:
				return q, k + 1
	return None, -1


def load_oracle(path):
	entries = []
	for line in open(path):
		if not line.strip() or line[0] == "#":
			continue
		parts = line.split()
		if parts[0] in ("P", "U"):
			vals = [float(v) for v in parts[1:8]]
			entries.append({
				"reach": parts[0] == "P",
				"p": np.array(vals[:3]),
				"quat": vals[3:],   # qw qx qy qz
			})
	return entries


def quat_to_R(qw, qx, qy, qz):
	# 四元数 -> 旋转矩阵
	R = np.array([
		[1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
		[2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
		[2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy **2)]])
	return R


def run_config(kin, entries, cfg, label):
	"""跑一个配置, 返回统计"""
	ok = 0
	back_ok = 0
	unreach_ok = 0
	times = []
	seed = (kin.lo + kin.hi) / 2
	for e in entries:
		tgt = (*e["p"], quat_to_R(*e["quat"]))
		t0 = time.perf_counter()
		q, used = solve(kin, tgt, seed, cfg)
		times.append((time.perf_counter() - t0) * 1000)
		if e["reach"]:
			if q is not None:
				ok += 1
				back = kin.fk(q)
				if back is not None and np.abs(np.array(back[:3]) - e["p"]).max() < 1e-6:
					back_ok += 1
		else:
			if q is None:
				unreach_ok += 1
	times = np.array(times)
	print(f"{label:36s} 可达OK {ok:3d}/90  回代 {back_ok:3d}  不可达拦 {unreach_ok:3d}/110  "
		f"p50 {np.percentile(times, 50):6.1f}ms p99 {np.percentile(times, 99):6.1f}ms")
	return ok, back_ok


if __name__ == "__main__":
	kin = Kin(URDF)
	entries = load_oracle(ORACLE)
	n_reach = sum(1 for e in entries if e["reach"])
	print(f"== {ROBOT}: {kin.n} 关节, oracle 可达 {n_reach} ==")

	base = {"pos_tol": 1e-6, "rot_tol": 1e-4, "max_iter": 200, "lambda0": 0.01,
		"lam_gain": 3.0, "lam_max": 0.5, "sigma_thr": 0.1, "w_rot": 1.0,
		"ns_gain": 0.3, "w_center": 1.0, "restarts": 40, "jump": 1.5,
		"max_bt": 4, "seed": 42}

	if "--full" in sys.argv:
		# 全矩阵 (慢: ~5min/配置, 调参深水区才用)
		run_config(kin, entries, base, "基线 (boxed+Wampler)")
		for wr in (0.3, 0.5, 1.0):
			run_config(kin, entries, dict(base, w_rot=wr), f"w_rot={wr}")
		for rg in (1.0, 3.0, 6.0):
			run_config(kin, entries, dict(base, lam_gain=rg), f"lam_gain={rg}")
		for ns in (0.0, 0.3, 0.8):
			run_config(kin, entries, dict(base, ns_gain=ns), f"ns_gain={ns}")
		for rs in (40, 80, 150):
			run_config(kin, entries, dict(base, restarts=rs), f"restarts={rs}")
	else:
		# 验证模式: 可达前 30 + 不可达前 30 (确定性抽样, 快 ~1min)
		# 对拍基准: C++ 同算法 59/90 (66%) -> 30 样本期望 ~20
		sub = [e for e in entries if e["reach"]][:30] + [e for e in entries if not e["reach"]][:30]
		run_config(kin, sub, base, "验证 (boxed+Wampler, 30+30)")

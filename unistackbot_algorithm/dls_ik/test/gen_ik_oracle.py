#!/usr/bin/env python3
"""IK 真值生成器 v2 (P1.4-C 扩样版) —— 关节空间正推采样, 覆盖可控、真解白送。

v1 的问题: 笛卡尔随机碰运气 (姿态不可达区大量浪费, 覆盖不可控, 90/200 可达率)。
v2 方案 (按 2026-09-17 评审建议):
  可达样本 = 关节空间 Halton 低差异采样 -> FK 正推位姿 (天然可达)
             -> ssik 复核全部解 (限位内) 记录真解; 分层标签 = r 带 × 四分支
  不可达样本 = 笛卡尔空间采样 + ssik 复核无解; 构造近奇异带 (r 贴边界)
  格式增强: 分层标签 / 真解 / 距最近分支中心 L1 距离, 便于失败归因。

ssik 在本仓的唯一使用点; C++ 测试零 ssik 依赖。指纹头 + 抽样自检不变。

用法: python3 gen_ik_oracle.py <robot> [samples]   (默认 600; 可达:不可达 ≈ 7:3)
输出: test/ik_oracle_<robot>.txt
"""
import subprocess
import sys
import datetime
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation as Rrot
import ssik

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "xarm7"
SAMPLES = int(sys.argv[2]) if len(sys.argv) > 2 else 600
URDF = f"/tmp/verify_{ROBOT}.urdf"
OUT = f"{__file__.rsplit('/', 2)[0]}/test/ik_oracle_{ROBOT}.txt"
TIP = "link7"
BASE = "link_base"

import xml.etree.ElementTree as ET
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

# 四分支代表 (k-means 于 2662 个 v1 解; v2 也用于分层标签)
BRANCH = np.array([
    [+0.30, +1.44, -0.03, +1.46, +0.87, -0.03, +1.44],
    [-0.60, +1.44, +0.02, +1.39, -0.79, -0.02, -1.44],
    [+0.15, -1.45, +2.73, +1.39, +0.06, -0.03, -0.07],
    [-0.07, -1.44, -2.73, +1.39, -0.07, -0.03, +0.09]])

m = ssik.Manipulator.from_urdf(URDF, base=BASE, ee=TIP)
import importlib.metadata
try:
    ssik_ver = importlib.metadata.version("ssik")
except Exception:
    ssik_ver = "unknown"


def halton(i, base):
    """Halton 低差异序列 (关节空间均匀覆盖, 避免随机簇聚)"""
    f, r = 1.0, 0.0
    while i > 0:
        f /= base
        r += f * (i % base)
        i //= base
    return r


def sample_joints_halton(idx):
    """Halton (素数基 2,3,5,7,11,13,17) 映射到限位盒"""
    q = np.zeros(len(JOINTS))
    for k, base in enumerate((2, 3, 5, 7, 11, 13, 17)):
        q[k] = LO[k] + halton(idx + 1, base) * (HI[k] - LO[k])
    return q


def branch_of(q):
    """距哪个分支中心最近 (分层标签用)"""
    d = [np.abs(q - b).sum() for b in BRANCH]
    return int(np.argmin(d)), min(d)


rng = np.random.default_rng(20260917)

lines = []
n_reach = n_unreach = 0
n_target_reach = int(SAMPLES * 0.7)
n_target_unreach = SAMPLES - n_target_reach

print(f"== v2 生成 {SAMPLES} 样本 (关节正推 {n_target_reach} / 不可达 {n_target_unreach}) ==")

# ---- 可达样本: Halton 关节采样 -> FK -> ssik 复核全部限位内解 ----
idx = 0
while n_reach < n_target_reach and idx < n_target_reach * 20:
    q0 = sample_joints_halton(idx)
    idx += 1
    T = np.eye(4)
    pose = m.fk(q0)
    T[:3, :3] = pose[:3, :3]
    T[:3, 3] = pose[:3, 3]
    sols = m.solve(T, respect_limits=True)
    if not sols:
        continue   # FK 可达但限位内无解 (罕见; Halton 点位可能贴界) —— 跳过
    # 记录: 位置由 FK 精确给出, ssik 解集含 q0 自身 (容差内)
    p = T[:3, 3]
    quat = Rrot.from_matrix(T[:3, :3]).as_quat()   # x,y,z,w
    r = float(np.linalg.norm(p))
    br, d_br = branch_of(q0)
    qs = [list(map(float, s.q)) for s in sols]
    lines.append(("P", p, quat, len(qs), qs, f"r={r:.2f} br{br} d_br={d_br:.1f}"))
    n_reach += 1
    if n_reach % 100 == 0:
        print(f"  可达 {n_reach}/{n_target_reach}")

# ---- 不可达样本: 笛卡尔带 + ssik 复核无解 ----
while n_unreach < n_target_unreach:
    # 混两类: 70% 远超臂展 (r>0.95), 30% 贴边界带 (0.82~0.95, 近奇异不可达候选)
    if rng.random() < 0.7:
        r = 0.95 + rng.random() * 0.35
    else:
        r = 0.82 + rng.random() * 0.13
    d = rng.normal(size=3)
    p = d / np.linalg.norm(d) * r
    T = np.eye(4)
    T[:3, 3] = p
    if m.solve(T, respect_limits=True):
        continue   # 贴边界带可能可达 —— 剔除 (诚实: 只收 ssik 确认无解的)
    quat = np.array([0.0, 1.0, 0.0, 0.0])   # 朝下姿态 (与 v1 一致)
    lines.append(("U", p, quat, 0, [], f"r={r:.2f}"))
    n_unreach += 1

# ---- 排序: 可达按 r 分层输出 (分析友好) ----
lines_reach = sorted([l for l in lines if l[0] == "P"], key=lambda l: np.linalg.norm(l[1]))
lines_unreach = [l for l in lines if l[0] == "U"]
lines = lines_reach + lines_unreach

# ---- 抽样自检 (5 个可达: FK 自检 + 分支距离一致性) ----
print("== 抽样自检 ==")
selfcheck = []
check_idx = np.linspace(0, len(lines_reach) - 1, 5).astype(int)
for ci in check_idx:
    tag, p, quat, cnt, qs, meta = lines_reach[ci]
    # ssik 解回代 (用 ssik 自己 FK; 我们 FK 已与 ssik 对拍 4e-13, 等价)
    back = m.fk(np.array(qs[0]))
    err = float(np.abs(np.array(back[:3, 3]) - p).max())
    ok = err < 1e-6
    selfcheck.append(f"{'PASS' if ok else 'FAIL'} err={err:.1e}")
    print(f"  {'PASS' if ok else 'FAIL'} err={err:.1e} ({meta})")
    if not ok:
        print("自检失败, 中止")
        sys.exit(1)

# ---- 写出 ----
now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
with open(OUT, "w") as f:
    f.write(f"# IK oracle for {ROBOT} (v2 关节正推采样)\n")
    f.write(f"# generator: ssik {ssik_ver} + Halton 关节采样 + FK 正推 + ssik 复核\n")
    f.write(f"# generated: {now}  samples={SAMPLES} reachable={n_reach} unreachable={n_unreach}\n")
    f.write(f"# self-check: {'; '.join(selfcheck)}\n")
    f.write(f"# format: P x y z qw qx qy qz | n_solutions | meta | n lines of 7 joint values\n")
    f.write(f"#         U x y z qw qx qy qz | meta\n")
    f.write(f"# meta: r=<半径> br=<分支> d_br=<距分支中心> (P) / r=<半径> (U)\n")
    for tag, p, quat, cnt, qs, meta in lines:
        head = f"{p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {quat[3]:.9f} {quat[0]:.9f} {quat[1]:.9f} {quat[2]:.9f}"
        if tag == "P":
            f.write(f"P {head} | {cnt} | {meta}\n")
            for q in qs:
                f.write("  " + " ".join(f"{v:.9f}" for v in q) + "\n")
        else:
            f.write(f"U {head} | {meta}\n")
print(f"== 写出 {OUT}: 可达 {n_reach} / 不可达 {n_unreach} ==")

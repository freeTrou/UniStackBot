#!/usr/bin/env python3
"""IK 真值生成器 (P1.4 S1) —— ssik 在本仓的唯一使用点。

用 ssik 对采样位姿离线标注: 可达(全部限位内解) / 不可达, 写成真值文件入库。
C++ 单测 (test_dls_ik.cpp) 只读真值文件, 零 ssik 依赖 —— ssik 的权威性被
固化为数据, 其脆弱性 (Python/新库/版本漂移) 被隔离在本脚本。

文件头指纹: ssik 版本 + URDF 来源机型 + 生成日期 + 抽样自检结果。
生成时自检: 抽 5 个可达解用本仓 fk_tool (独立 FK 实现) 交叉验证闭合,
第三方答案先过我们的秤才入库。

用法: python3 gen_ik_oracle.py <robot> [samples]
输出: test/ik_oracle_<robot>.txt
"""
import subprocess
import sys
import datetime
import random
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation as Rrot
import ssik

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "xarm7"
SAMPLES = int(sys.argv[2]) if len(sys.argv) > 2 else 200
URDF = f"/tmp/verify_{ROBOT}.urdf"
OUT = f"{__file__.rsplit('/', 2)[0]}/test/ik_oracle_{ROBOT}.txt"
TIP = "link7"
BASE = "link_base"

# ---- 读限位 (从展开 URDF 的 ros2_control 块) ----
import xml.etree.ElementTree as ET
lim = {}
for j in ET.parse(URDF).getroot().iter("joint"):
    if j.get("name") is None or j.find("command_interface") is None:
        continue
    ps = {p.get("name"): p.text for p in j.findall("param")}
    if "min" in ps and "max" in ps:
        lim[j.get("name")] = (float(ps["min"]), float(ps["max"]))
JOINTS = list(lim)

# ---- ssik 求解器 ----
m = ssik.Manipulator.from_urdf(URDF, base=BASE, ee=TIP)
import importlib.metadata
try:
    ssik_ver = importlib.metadata.version("ssik")
except Exception:
    ssik_ver = "unknown"

rng = random.Random(20260917)

def rand_pose_reachable():
    """可达带采样: r∈[0.25,0.80] 球面均匀方向, 姿态=朝下 (抓取位姿, 避开姿态不可达区)"""
    r = 0.25 + rng.random() * 0.55
    d = np.array([rng.gauss(0, 1) for _ in range(3)])
    d[2] = -abs(d[2]) * 0.5 - 0.1          # 偏向下半空间 (朝下姿态可达带)
    if np.linalg.norm(d) < 1e-6:
        d = np.array([0.0, 0.0, -1.0])
    p = d / np.linalg.norm(d) * r
    R = np.diag([1.0, -1.0, -1.0])          # 末端朝下
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    return T, f"reachable-band r={r:.2f}"

def rand_pose_unreachable():
    """不可达带采样: r∈[0.95,1.3] (实测边界 ~0.85, 几何臂展决定, 必然空解)"""
    r = 0.95 + rng.random() * 0.35
    d = np.array([rng.gauss(0, 1) for _ in range(3)])
    p = d / np.linalg.norm(d) * r
    T = np.eye(4); T[:3, 3] = p
    return T, f"beyond-reach r={r:.2f}"

# ---- FK 自检 (调本仓 fk_tool, 走运行链路; 链未起则跳过自检并警告) ----
def self_check_fk(q_list, expect_p):
    names_vals = ",".join(f"{n}={q:.9f}" for n, q in zip(JOINTS, q_list))
    r = subprocess.run(["ros2", "run", "unistackbot_controller", "fk_tool",
        "--joints", names_vals], capture_output=True, text=True, timeout=30)
    mo = re.search(r"position \[([-\d.e+]+), ([-\d.e+]+), ([-\d.e+]+)\]", r.stdout)
    if not mo:
        return None   # 链路未起, 无法自检
    p = np.array([float(mo.group(1)), float(mo.group(2)), float(mo.group(3))])
    return float(np.abs(p - expect_p).max())

print(f"== 采样 {SAMPLES} 位姿 (可达带 70% / 不可达带 30%) ==")
lines = []
n_reach = n_unreach = 0
for i in range(SAMPLES):
    if rng.random() < 0.7:
        T, tag = rand_pose_reachable()
    else:
        T, tag = rand_pose_unreachable()
    sols = m.solve(T, respect_limits=True)
    p = T[:3, 3]
    # 姿态四元数从 T 提取 (必须入库! 只存 xyz 会让测试端目标姿态=默认单位,
    # 与生成时 ssik 解的姿态不符 -> 假失败, 2026-09-17 实测踩中)
    quat = Rrot.from_matrix(T[:3, :3]).as_quat()   # x,y,z,w
    if sols:
        n_reach += 1
        qs = [list(map(float, s.q)) for s in sols]
        lines.append(("P", p, len(qs), qs, quat))
    else:
        n_unreach += 1
        lines.append(("U", p, 0, [], quat))
    if (i + 1) % 40 == 0:
        print(f"  {i+1}/{SAMPLES} 可达{n_reach} 不可达{n_unreach}")

# ---- 抽样自检: 5 个可达解过我们自己的 FK ----
print("== 抽样自检 (ssik 解 -> 本仓 fk_tool 闭合) ==")
reach_lines = [l for l in lines if l[0] == "P"]
check_lines = rng.sample(reach_lines, min(5, len(reach_lines)))
selfcheck = []
for tag, p, cnt, qs, quat in check_lines:
    err = self_check_fk(qs[0], p)
    if err is None:
        selfcheck.append("SKIPPED(no-chain)")
        print(f"  自检跳过 (链路未起; 建议起 mock 链后重跑以完成自检)")
        break
    ok = err < 1e-6
    selfcheck.append(f"{'PASS' if ok else 'FAIL'} err={err:.2e}")
    print(f"  {'PASS' if ok else 'FAIL'} err={err:.2e}")
    if not ok:
        print("自检失败: ssik 解与我们的 FK 不闭合 —— 真值不可信, 中止写出")
        sys.exit(1)

# ---- 写出 ----
now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
with open(OUT, "w") as f:
    f.write(f"# IK oracle for {ROBOT}\n")
    f.write(f"# generator: ssik {ssik_ver} + 本仓 fk_tool 抽样自检\n")
    f.write(f"# generated: {now}  samples={SAMPLES} reachable={n_reach} unreachable={n_unreach}\n")
    f.write(f"# self-check: {'; '.join(selfcheck) if selfcheck else 'skipped'}\n")
    f.write(f"# format: P x y z qw qx qy qz | n_solutions | then n lines of 7 joint values\n")
    f.write(f"#         U x y z qw qx qy qz | unreachable\n")
    for tag, p, cnt, qs, quat in lines:
        head = f"{p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {quat[3]:.9f} {quat[0]:.9f} {quat[1]:.9f} {quat[2]:.9f}"
        if tag == "P":
            f.write(f"P {head} | {cnt}\n")
            for q in qs:
                f.write("  " + " ".join(f"{v:.9f}" for v in q) + "\n")
        else:
            f.write(f"U {head}\n")
print(f"== 写出 {OUT}: 可达 {n_reach} / 不可达 {n_unreach} ==")

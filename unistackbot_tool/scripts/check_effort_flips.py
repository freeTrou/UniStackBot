#!/usr/bin/env python3
"""mcap effort 翻转率检查 —— 伺服病理段检测 (stationlite mujoco README #7 教训工具化)。

背景: MuJoCo 位置伺服在最小惯量构型可进继电器式极限环——effort ±限 满幅高频换向
(482Hz 采样下相邻符号翻转率 22.5%), 位置看似不动。此类段落的 effort 观测是仿真
病理非物理, 训练数据消费前应检出。

双判据 (2026-10-20 好/病包双向校准定形):
  ① 窗峰翻转率 > 阈值 —— 病理签名, 但物理小信号过零抖动 / 物理饱和换向也有高翻转;
  ② 速度-位移失配比 > 20 —— 病理独有: 位置净变化≈0 而 |v| 读数恒满 (振颤积分不出
     位移); 物理运动 (含往复) 两者恒同量级。仅①会误报 (实测 j3 26% 翻转为物理), 仅②
     漏报率低。两者同时成立才判 PATHOLOGIC。

用法:
  ros2 run unistackbot_tool check_effort_flips.py <mcap 路径或含 verify_*.mcap 的目录>
      [--window 2.0]    滑窗秒数 (默认 2.0s, 报告窗内峰值)
      [--threshold 5.0] 翻转率告警阈值 % (窗内 > 阈值即候选)
      [--eff-min 0.5]   参与判定的最小 |effort| N·m (滤零段, 默认 0.5)
      [--ratio 20.0]    失配比告警阈值 (默认 20)

退出码: 0=全部干净, 1=检出病理窗。录包须按规范 v1.1 含 /joint_states。
"""
import glob
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def load_states(path):
	"""返回 (mcap 路径, t_sec, names, effort[NxJ], pos[NxJ], vel[NxJ])"""
	mcap = path if os.path.isfile(path) else None
	if mcap is None:
		cands = sorted(glob.glob(os.path.join(path, '**', 'verify_*.mcap'), recursive=True))
		if not cands:
			raise SystemExit(f'未找到 mcap: {path} (传 mcap 文件或含 verify_*.mcap 的目录)')
		mcap = cands[-1]
	bag = rosbag2_py.SequentialReader()
	bag.open(rosbag2_py.StorageOptions(uri=mcap, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
	ts, eff_c, pos_c, vel_c, names = [], None, None, None, None
	while True:
		try:
			topic, data, t = bag.read_next()
		except Exception:
			break
		if topic != '/joint_states':
			continue
		m = deserialize_message(data, JointState)
		if eff_c is None:
			names = list(m.name)
			eff_c = [[] for _ in names]
			pos_c = [[] for _ in names]
			vel_c = [[] for _ in names]
		ts.append(t / 1e9)
		for i in range(len(names)):
			eff_c[i].append(m.effort[i] if i < len(m.effort) else 0.0)
			pos_c[i].append(m.position[i] if i < len(m.position) else 0.0)
			vel_c[i].append(m.velocity[i] if i < len(m.velocity) else 0.0)
	if eff_c is None:
		raise SystemExit('包内无 /joint_states')
	return mcap, np.array(ts), names, np.array(eff_c).T, np.array(pos_c).T, np.array(vel_c).T


def main():
	args = sys.argv[1:]
	path, window, thresh, eff_min, ratio_lim = None, 2.0, 5.0, 0.5, 20.0
	while args:
		a = args.pop(0)
		if a == '--window':
			window = float(args.pop(0))
		elif a == '--threshold':
			thresh = float(args.pop(0))
		elif a == '--eff-min':
			eff_min = float(args.pop(0))
		elif a == '--ratio':
			ratio_lim = float(args.pop(0))
		elif path is None:
			path = a
		else:
			raise SystemExit(f'未知参数 {a}')
	if path is None:
		raise SystemExit(__doc__)

	mcap, ts, names, eff, pos, vel = load_states(path)
	if len(ts) < 10:
		raise SystemExit(f'/joint_states 样本不足 ({len(ts)} 帧) —— 废数据或录制中断')
	dt = float(np.median(np.diff(ts)))
	if not np.isfinite(dt) or dt <= 0:
		raise SystemExit(f'时间戳异常 (dt={dt}) —— 包损坏或时钟停摆')
	w = max(int(window / dt), 10)
	print(f'{mcap}')
	print(f'时长 {ts[-1]-ts[0]:.1f}s, {len(ts)} 帧 @ {1/dt:.0f}Hz, 窗 {window:.1f}s, 阈值 翻转{thresh:.0f}%/失配比{ratio_lim:.0f}')
	print(f'{"关节":24s} {"|τ|max":>7s} {"活跃占比":>8s} {"窗峰翻转率":>10s} {"失配比":>9s}  判定')

	rc = 0
	for i, n in enumerate(names):
		e = eff[:, i]
		active = np.abs(e) > eff_min
		if active.mean() < 0.01:
			continue   # 近恒零关节 (如未用夹爪) 不判定
		flip = (np.diff(np.sign(e)) != 0) & active[:-1] & active[1:]
		peak, peak_t, peak_ratio = 0.0, 0.0, 0.0
		path_win = None   # 首个双判据成立窗 (翻转超阈 且 失配超阈) —— 任何一窗成立即病理
		for k in range(0, max(len(e) - w, 1), w // 2):
			sl = slice(k, min(k + w, len(e) - 1))
			den = int(active[sl].sum())
			if den == 0:
				continue   # 全零窗 (段间静止), 无可判定样本
			r = flip[sl].sum() / den * 100.0
			# 失配比 = 窗内平均|v|·时长 / 净位移 —— 物理运动 ≈1 (往复略大), 振颤 ≫20。
			# 仅当窗"看似运动" (vmean ≥ 0.05 rad/s) 才参与病理判定: 静止微抖 vmean≈0,
			# 失配比被 1e-9 分母放大成假 2000 (实测 j2/j3 包头瞬态误报)
			net = abs(pos[min(k + w, len(pos)) - 1, i] - pos[k, i])
			vmean = float(np.abs(vel[sl, i]).mean())
			ratio = (vmean * (min(k + w, len(e)) - k) * dt) / max(net, 1e-9)
			if vmean < 0.05:
				continue
			if r > peak:
				peak, peak_t, peak_ratio = r, ts[k] - ts[0], ratio
			if path_win is None and r > thresh and ratio > ratio_lim:
				path_win = (ts[k] - ts[0], ratio)
		pathologic = path_win is not None
		verdict = 'PATHOLOGIC' if pathologic else 'ok'
		if pathologic:
			rc = 1
			show_t, show_r = path_win[0], path_win[1]
		else:
			show_t, show_r = peak_t, peak_ratio
		# 高翻转但未达病理 = 物理换向 (标定例: j3 26% 翻转为小信号过零), 显式注明防误读
		note = '  (高翻转但速度位移一致, 判物理)' if (not pathologic and peak > thresh) else ''
		print(f'{n:24s} {np.abs(e).max():7.2f} {active.mean()*100:7.1f}% {peak:9.1f}% {show_r:9.0f}  {verdict} @t={show_t:.1f}s{note}')
	sys.exit(rc)


if __name__ == '__main__':
	main()

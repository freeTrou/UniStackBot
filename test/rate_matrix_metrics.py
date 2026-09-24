#!/usr/bin/env python3
"""频率矩阵指标 (2026-09-23): 从 rate_matrix_bench.sh 录的 mcap 包算一行 markdown 结果。

口径 (与 2026-09-23 hold vs ruckig A/B 一致):
- 命令达成Hz: /joint_stream_controller/command 消息 stamp 差分中位数 → Hz (demo 自报校准的旁证)
- JS实测Hz:   /joint_states stamp 差分中位数 → Hz (总线频率旁证)
- max|dv|/拍: 相邻 JS 拍各关节 |Δposition| 的全程最大 (worst 关节) —— 平滑度主指标
- p50|dv|/拍: 运动拍 (|dv|>0) 的中位数 (运动段背景水平, 排除静止占位)
- |Δq|2阶差 max: 相邻拍 |dv| 差的最大 (加速度/冲击代理)

用法: rate_matrix_metrics.py <bag目录> --bus-hz 500 --cmd-hz 50 --domain joint [--tag <标签>]
输出: 一行 markdown 表行 (stdout), 供 bench 脚本追加进 test/results/rate_matrix_<机型>_<链>.md
"""
import argparse
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from unistackbot_interface.msg import JointCommand


def median(xs):
	xs = sorted(xs)
	n = len(xs)
	if n == 0:
		return float('nan')
	return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def rate_hz(stamps):
	d = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
	return 1.0 / median(d) if d else float('nan')


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument('bag')
	ap.add_argument('--bus-hz', type=float, required=True)
	ap.add_argument('--cmd-hz', type=float, required=True)
	ap.add_argument('--domain', default='joint')
	ap.add_argument('--tag', default='')
	a = ap.parse_args()

	reader = rosbag2_py.SequentialReader()
	reader.open(
		rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'),
		rosbag2_py.ConverterOptions('', ''))
	js_t, js_q, js_names, cmd_t, cm_t = [], [], [], [], []
	while reader.has_next():
		topic, data, t = reader.read_next()
		if topic == '/joint_states':
			m = deserialize_message(data, JointState)
			js_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
			js_q.append(list(m.position))
			js_names = list(m.name)
		elif topic == '/joint_stream_controller/command':
			# JointCommand 无 header —— 用 bag 接收时间戳 (ns) 差分求率
			cmd_t.append(t * 1e-9)
		elif topic == '/cartesian_motion_controller/target':
			cm_t.append(t * 1e-9)   # cartesian 域命令侧旁证 (JS command 此时静默)
	del reader
	# cartesian 域: JS command 被 CM 切走后静默, 命令侧频率用 CM 目标流
	if len(cmd_t) < 5 and len(cm_t) >= 5:
		cmd_t = cm_t

	if len(js_t) < 20 or not js_names:
		print(f'FAIL: /joint_states 仅 {len(js_t)} 帧', file=sys.stderr)
		return 1

	cmd_hz = rate_hz(cmd_t) if len(cmd_t) >= 5 else float('nan')
	js_hz = rate_hz(js_t)

	# 平滑度: 相邻 JS 拍 per-joint |Δq|
	dv1 = [0.0] * len(js_names)
	nonzero = []
	d2max = 0.0
	for a_, b_, c_ in zip(js_q, js_q[1:], js_q[2:]):
		for j in range(len(js_names)):
			d = abs(b_[j] - a_[j])
			if d > dv1[j]:
				dv1[j] = d
			if d > 0.0:
				nonzero.append(d)
			d2 = abs((c_[j] - b_[j]) - d)
			if d2 > d2max:
				d2max = d2
	worst = max(range(len(js_names)), key=lambda j: dv1[j])
	p50 = median(nonzero) if nonzero else 0.0

	name = a.tag or f'{a.domain}_{int(a.cmd_hz)}'
	print(
		f'| {name} | {a.domain} | {int(a.bus_hz)} | {int(a.cmd_hz)} '
		f'| {cmd_hz:.1f} | {js_hz:.1f} '
		f'| {dv1[worst]:.5f} ({js_names[worst]}) | {p50:.6f} | {d2max:.5f} |', end=' |\n')
	return 0


if __name__ == '__main__':
	sys.exit(main())

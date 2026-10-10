#!/usr/bin/env python3
"""点位资产 v0 → v1 转换 (数据录制规范 v1 §3)。

v0: {side: {点名: [q..]}, right/rest/programs: 任意} —— 无 meta、序靠命名约定。
v1: 补 meta (robot/side/hz/joint_names/source/notes/limit_margin) + 自然序
    sequences (to_*/from_* 前缀分组, 尾号按数值——字典序事故根, 2026-10-09)。
已知瑕疵自动检出入 notes: 桩点与走廊首点的大跳 (v0 数据源常见)。

用法: convert_points_v0_v1.py <in.json> <out.json> [--robot r] [--side left]
        [--hz 20] [--operator name] [--notes "…"]
"""

import json
import re
import sys
from datetime import datetime


def natural_key(k):
	head = k.rstrip('0123456789')
	tail = k[len(head):]
	return (head, int(tail) if tail else -1)


def main():
	argv = sys.argv[1:]
	if len(argv) < 2:
		print(__doc__)
		return 2
	src, dst = argv[0], argv[1]
	robot = side = ''
	hz = ''
	operator = ''
	notes_extra = ''
	rest = argv[2:]
	while rest:
		a = rest.pop(0)
		if a == '--robot':
			robot = rest.pop(0)
		elif a == '--side':
			side = rest.pop(0)
		elif a == '--hz':
			hz = rest.pop(0)
		elif a == '--operator':
			operator = rest.pop(0)
		elif a == '--notes':
			notes_extra = rest.pop(0)
		else:
			raise SystemExit("未知参数 %s" % a)

	d = json.load(open(src))
	sides = [k for k, v in d.items()
	         if isinstance(v, dict) and all(isinstance(x, list) for x in v.values())]
	if not sides:
		raise SystemExit('未找到点位节 (期望 {side: {点名: [q..]}})')
	if not side:
		side = sides[0] if len(sides) == 1 else ','.join(sides)
	points = {s: d[s] for s in sides}

	notes = []
	seqs = {}
	for s in sides:
		names = list(points[s].keys())
		# 序列发现: to_/from_ 前缀分组自然序 + 首尾锚点 (无前缀的非 meta 点)
		anchors = [n for n in names if not re.search(r'_\d+$', n)]
		fwd = sorted((n for n in names if re.match(r'.*to_.*_\d+$', n)), key=natural_key)
		bck = sorted((n for n in names if re.match(r'.*from_.*_\d+$', n)), key=natural_key)
		if fwd:
			seqs['corridor_%s' % s] = (anchors[:1] + fwd
			                           + ([anchors[1]] if len(anchors) > 1 else []) + bck)
			# 已知瑕疵检出: 锚点 vs 走廊首点大跳 (>5° 单关节)
			if anchors and fwd:
				q0, q1 = points[s][anchors[0]], points[s][fwd[0]]
				dmax = max(abs(a - b) for a, b in zip(q0, q1))
				if dmax > 0.087:
					notes.append('%s: 锚点 %s 与走廊首点 %s 单关节差 %.1f° '
					             '(数据源跳变, 正式资产应重采)'
					             % (s, anchors[0], fwd[0], dmax * 57.3))
	if notes_extra:
		notes.append(notes_extra)

	meta = {
		'format': 1,
		'robot': robot or 'unknown',
		'side': side,
		'joint_names': [],
		'hz': hz,
		'created': datetime.now().isoformat(timespec='seconds'),
		'operator': operator,
		'source': src,
		'notes': '; '.join(notes) if notes else '',
		'limit_margin': '',
	}
	out = {'meta': meta, 'points': points}
	if seqs:
		out['sequences'] = seqs
	json.dump(out, open(dst, 'w'), indent=1, ensure_ascii=False)
	print('v1 已写: %s (sequences: %s)' % (dst, ', '.join('%s=%d 点' % (k, len(v))
	                                                    for k, v in seqs.items())))
	for n in notes:
		print('  ⚠ %s' % n)
	return 0


if __name__ == '__main__':
	sys.exit(main())

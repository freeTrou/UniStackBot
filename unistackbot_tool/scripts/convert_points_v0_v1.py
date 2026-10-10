#!/usr/bin/env python3
"""点位资产 v0 → v1 转换 (数据录制规范 v1 §3)。

v0: {side: {点名: [q..]}, right/rest/programs: 任意} —— 无 meta、序靠命名约定。
CSV (.csv): 表头 name,q1..q6 —— **序列 = 文件行序** (不做任何排序假设; 上游已排
    数值序, 2026-10-10 corridor_export.csv 形态, 双臂左臂数据直接吃)。
v1: 补 meta (robot/side/hz/joint_names/source/notes/limit_margin) + sequences
    (json v0 = to_*/from_* 前缀分组自然序; CSV = 行序)。
已知瑕疵自动检出入 notes: 锚点与走廊首点的大跳 (数据源常见)。

用法: convert_points_v0_v1.py <in.(json|csv)> <out.json> [--robot r] [--side left]
        [--hz 20] [--joint-names "left_joint1,..,6"] [--roundtrip]
        [--operator name] [--notes "…"]

--joint-names: 填 meta.joint_names (逐位对应点数据的 q 序; 消费端 demo_points_playback
    与链上侧关节逐位校验——映射是位置性的, 序错=回放错)。不给则 meta 留空并告警。
--roundtrip: CSV 输入时追加 corridor_<side>_roundtrip = 行序 + 倒序回程 (共享首末
    锚点) —— 61 点走廊 → 121 点往返, 可再生 (此前手工拼接不可再生)。
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
	joint_names = ''
	roundtrip = False
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
		elif a == '--joint-names':
			joint_names = rest.pop(0)
		elif a == '--roundtrip':
			roundtrip = True
		elif a == '--operator':
			operator = rest.pop(0)
		elif a == '--notes':
			notes_extra = rest.pop(0)
		else:
			raise SystemExit("未知参数 %s" % a)

	if src.endswith('.csv'):
		# CSV 形态: name,q1..q6 表头; 全部点进 --side 节 (默认 left), 序列=行序
		import csv as _csv
		if not side:
			side = 'left'
		rows = list(_csv.DictReader(open(src)))
		d = {side: {r['name']: [float(r['q%d_rad' % (i + 1)]) for i in range(6)]
		            for r in rows}}
		csv_order = [r['name'] for r in rows]
	else:
		csv_order = None
	sides = [k for k, v in d.items()
	         if isinstance(v, dict) and all(isinstance(x, list) for x in v.values())]
	if not sides:
		raise SystemExit('未找到点位节 (期望 {side: {点名: [q..]}})')
	if not side:
		side = sides[0] if len(sides) == 1 else ','.join(sides)
	points = {s: d[s] for s in sides}
	# json 输入且显式 --side: 只保留该侧 (csv 分支 side 已用于构造)

	notes = []
	seqs = {}
	for s in sides:
		names = list(points[s].keys())
		# 序列发现: to_/from_ 前缀分组自然序 + 首尾锚点 (无前缀的非 meta 点)
		anchors = [n for n in names if not re.search(r'_\d+$', n)]
		fwd = sorted((n for n in names if re.match(r'.*to_.*_\d+$', n)), key=natural_key)
		bck = sorted((n for n in names if re.match(r'.*from_.*_\d+$', n)), key=natural_key)
		if csv_order is not None:
			seqs['corridor_%s' % s] = list(csv_order)   # CSV: 行序即序 (上游已排)
			if roundtrip:
				# 往返 = 行序 + 倒序回程 (共享首末锚点): 61 → 121 点
				seqs['corridor_%s_roundtrip' % s] = (list(csv_order)
				                                     + list(reversed(csv_order[1:-1]))
				                                     + [csv_order[0]])
		elif fwd:
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

	jnames = [j.strip() for j in joint_names.split(',') if j.strip()] if joint_names else []
	if not jnames:
		notes.append('meta.joint_names 为空——q→关节映射无交叉校验 (补 --joint-names 可填)')
	meta = {
		'format': 1,
		'robot': robot or 'unknown',
		'side': side,
		'joint_names': jnames,
		'hz': float(hz) if hz else '',
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

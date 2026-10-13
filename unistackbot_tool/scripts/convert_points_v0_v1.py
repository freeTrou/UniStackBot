#!/usr/bin/env python3
"""点位资产 v0 → v1 转换 (数据录制规范 v1.1 §3) —— **纯搬运, 零加工** (2026-10-12 用户令)。

原则: 点位数据一个值都不动、一个序列都不猜。转换只做格式搬运 + meta 补齐:
  - points 原样保留 (无排序/无重排/无合成)
  - 输入自带 sequences (v1) → 原样透传
  - CSV 行序是文件固有事实 → 显式记为 file_order_<side> (不是推导, 是记录)
  - v0 json 的 to_*/from_* 命名启发式序列推导: **退役** (字典序事故同族假设;
    序列应由采集端显式给出, 回放侧 v0 双兼容路径仍在)
  - --roundtrip 合成往返: **退役** (生成性修改)

v0: {side: {点名: [q..]}} —— 无 meta、序靠命名约定。
CSV: 表头 name,q1_rad..q6_rad —— 全部点进 --side 节 (默认 left)。
v1: {meta, points, sequences?}。

用法: convert_points_v0_v1.py <in.(json|csv)> <out.json> [--robot r] [--side left]
        [--hz 20] [--joint-names "j1,j2,.."] [--operator name] [--notes "…"]

--joint-names: 填 meta.joint_names (逐位对应 q 序; 回放端逐位校验——序错=回放错)。
"""

import json
import sys
from datetime import datetime


def main():
	argv = sys.argv[1:]
	if len(argv) < 2:
		print(__doc__)
		return 2
	src, dst = argv[0], argv[1]
	robot = side = ''
	hz = ''
	joint_names = ''
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
		elif a == '--operator':
			operator = rest.pop(0)
		elif a == '--notes':
			notes_extra = rest.pop(0)
		else:
			raise SystemExit("未知参数 %s" % a)

	in_seq = None
	if src.endswith('.csv'):
		# CSV 形态: name,q1..q6 表头; 全部点进 --side 节 (默认 left)
		import csv as _csv
		if not side:
			side = 'left'
		rows = list(_csv.DictReader(open(src)))
		d = {side: {r['name']: [float(r['q%d_rad' % (i + 1)]) for i in range(6)]
		            for r in rows}}
		csv_order = [r['name'] for r in rows]
	else:
		csv_order = None
		data = json.load(open(src))
		d = data
		in_seq = data.get('sequences')   # v1 输入: 显式 sequences 原样透传
	sides = [k for k, v in d.items()
	         if isinstance(v, dict) and all(isinstance(x, list) for x in v.values())]
	if not sides:
		raise SystemExit('未找到点位节 (期望 {side: {点名: [q..]}})')
	if not side:
		side = sides[0] if len(sides) == 1 else ','.join(sides)
	points = {s: d[s] for s in sides}

	notes = []
	seqs = {}
	if in_seq:
		seqs.update(in_seq)   # 显式 sequences: 原样透传 (零修改)
	elif csv_order is not None:
		# CSV 行序 = 文件固有事实的显式化 (记录, 非推导); 中性命名
		seqs['file_order_%s' % side] = list(csv_order)
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
	print('v1 已写: %s (points %s 原样; sequences: %s)' % (
		dst, ', '.join('%s=%d 点' % (s, len(v)) for s, v in points.items()),
		', '.join('%s=%d 点' % (k, len(v)) for k, v in seqs.items()) or '无 (显式序列请由采集端给出)'))
	for n in notes:
		print('  ⚠ %s' % n)
	return 0


if __name__ == '__main__':
	sys.exit(main())

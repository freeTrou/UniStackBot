#!/usr/bin/env python3
"""stationlite 双臂真机轨迹回放 demo (机型专属; scripts/<robot>_arm/ 约定, 2026-10-12)。

**数据来源 = 真机**: stationlite_joint_cycle_points.json 是 startouch SDK JointCycle
记录 (2026-10-19 双臂循环测试) 的 q_target **全量原始点位** (每臂 ~400Hz, 零加工)
—— 左臂 28572 点/71.4s, 右臂 16170 点/40.4s。本 demo 把它在我们仿真链上原样复现:
双臂同步起跑, 各按自己的时间轴走位, 短的一臂走完后保持 (断流语义)。

对照价值: 真机该轨迹由逐点启停 (stop-and-go) 执行; 我们的 JS ruckig 档把同一串
点位缝成 C2 连续运动 —— 同样的点, 不同的平滑, 肉眼可辨。

前置: 任一链已启动, joint_stream_controller active (mujoco 链命令面 = 14 关节)。
用法: ros2 run unistackbot_demo stationlite_demo_replay.py [--rate 1.0] [--hz 100]
      --rate 0.5 = 慢放, 2.0 = 快放; --hz = 回放流频率 (默认 100, 原生 400)
"""
import json
import os
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unistackbot_interface.msg import JointCommand

def _find_points():
	"""定位点位资产: 源码树 (脚本旁) → install share (ros2 run 平铺场景)"""
	cands = [os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stationlite_joint_cycle_points.json')]
	try:
		from ament_index_python.packages import get_package_share_directory
		cands.append(os.path.join(get_package_share_directory('unistackbot_demo'), 'stationlite_arm', 'stationlite_joint_cycle_points.json'))
	except Exception:
		pass
	for c in cands:
		if os.path.isfile(c):
			return c
	raise SystemExit('点位资产未找到: %s (先 colcon build)' % cands)

POINTS_FILE = _find_points()
ARM_JOINTS = lambda s: [f'{s}_joint{i}' for i in range(1, 7)]
GRIPPER = lambda s: f'{s}_gripper_joint1'


def js_active():
	try:
		r = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True, timeout=15)
	except Exception:
		return False
	for line in r.stdout.splitlines():
		clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip()).split()
		if len(clean) >= 3 and clean[0] == 'joint_stream_controller' and clean[-1] == 'active':
			return True
	return False


def main():
	rate, hz = 1.0, 100
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--rate':
			rate = float(argv.pop(0))
		elif a == '--hz':
			hz = int(argv.pop(0))
		else:
			raise SystemExit("未知参数 '%s' (支持: --rate <f> --hz <n>)" % a)

	data = json.load(open(POINTS_FILE))
	lt, lq = data['left']['t'], data['left']['q']
	rt, rq = data['right']['t'], data['right']['q']
	print('点位: 左 %d 点 %.1fs | 右 %d 点 %.1fs | 回放 x%.1f @%dHz' % (len(lt), lt[-1], len(rt), rt[-1], rate, hz))

	if not js_active():
		raise SystemExit('joint_stream_controller 不在 active —— 先起链 (ros2 launch ... robot:=stationlite)')

	rclpy.init()
	node = Node('stationlite_replay')
	pub = node.create_publisher(JointCommand, '/joint_stream_controller/command',
		QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
	names = ARM_JOINTS('left') + [GRIPPER('left')] + ARM_JOINTS('right') + [GRIPPER('right')]

	def sample(t, t_arr, q_arr):
		"""时间轴二分: 最近样本 (走出末端 = 末值保持)"""
		if t >= t_arr[-1]:
			return q_arr[-1]
		lo, hi = 0, len(t_arr) - 1
		while hi - lo > 1:
			mid = (lo + hi) // 2
			if t_arr[mid] <= t:
				lo = mid
			else:
				hi = mid
		return q_arr[lo]

	msg = JointCommand()
	msg.mode = JointCommand.MODE_CSP
	msg.joint_names = names
	dur = max(lt[-1], rt[-1]) / rate
	t0 = time.time()
	sent = 0
	print('回放中 (%.0fs)... Ctrl-C 可中断' % dur)
	while True:
		elapsed = (time.time() - t0) * rate
		if elapsed > lt[-1] and elapsed > rt[-1]:
			break
		left = sample(elapsed, lt, lq)
		right = sample(elapsed, rt, rq)
		msg.position = list(left) + [0.0] + list(right) + [0.0]
		pub.publish(msg)
		sent += 1
		rclpy.spin_once(node, timeout_sec=0.0)
		nxt = t0 + sent / hz
		d = nxt - time.time()
		if d > 0:
			time.sleep(d)
	print('回放完成: %d 条流 (双臂各自走到末端, 已按断流语义保持)' % sent)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

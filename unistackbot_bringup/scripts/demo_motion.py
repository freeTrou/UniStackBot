#!/usr/bin/env python3
"""给 JointStream 发送一段往返演示运动 (正弦过渡的 50Hz 点流), 机型无关。

2026-09-21 重写: JTC action 退役 → JointCommand 点流 (点流语义: 每条消息=最新目标,
控制器端每周期步长饱和逼近——上层只管把目标流平滑地发下来)。

关节表/限位解析自 robot_state_publisher 的 robot_description (URDF <ros2_control> 块,
带 position 命令接口的关节, 与 JointStream 解析口径一致), 路径点按限位区间百分比推算
—— 本脚本不含任何机型硬编码。

前置: 三链任一已启动 (control/ign/mujoco.launch.py) 且 joint_stream_controller active。

用法: ros2 run unistackbot_bringup demo_motion.py [--hz 50] [--cycles 1]
"""

import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from unistackbot_interface.msg import JointCommand


class DemoMotion(Node):

	def __init__(self):
		super().__init__('demo_motion')

	def _get_param(self, node_name, name):
		"""经参数服务读远端节点参数 (Humble rclpy 无 SyncParameterClient, 手工建客户端)。"""
		cli = self.create_client(GetParameters, node_name + '/get_parameters')
		if not cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError('参数服务 %s/get_parameters 5 秒内未出现 (链路未起?)' % node_name)
		req = GetParameters.Request()
		req.names = [name]
		fut = cli.call_async(req)
		rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
		return fut.result().values[0]

	def _fetch_contract(self):
		"""关节表 + 每关节 (min, max), 全部来自运行中的链路, 不本地写死。

		mimic 关节 (mock 链手指: 有命令接口但无 min/max, 只有 mimic/multiplier)
		也必须列全 —— JS 契约要求消息含全部命令关节; 其目标 = multiplier×主关节+offset。
		返回 (直控关节表, 直控限位表, mimic 关节表[(name, master, k, off)])。"""
		uv = self._get_param('/robot_state_publisher', 'robot_description')
		if not uv.string_value:
			raise RuntimeError('读取 robot_state_publisher 的 robot_description 失败')
		joints, lims, mimics = [], [], []
		for j in ET.fromstring(uv.string_value).iter('joint'):
			if j.get('name') is None or j.find('command_interface') is None:
				continue
			if (j.find('command_interface').get('name') != 'position'):
				continue
			name = j.get('name')
			ps = {p.get('name'): p.text for p in j.findall('param')}
			if 'min' in ps and 'max' in ps:
				joints.append(name)
				lims.append((float(ps['min']), float(ps['max'])))
			elif 'mimic' in ps:
				mimics.append((name, ps['mimic'],
				               float(ps.get('multiplier', '1')), float(ps.get('offset', '0'))))
		if not joints:
			raise RuntimeError('URDF ros2_control 块无 position 命令关节')
		return joints, lims, mimics

	def run(self, hz, cycles):
		joints, lims, mimics = self._fetch_contract()
		self.get_logger().info('关节表 (读自链上 URDF): %s%s' % (
			joints, ' + mimic %s' % [m[0] for m in mimics] if mimics else ''))

		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
		                 history=HistoryPolicy.KEEP_LAST, depth=1)
		pub = self.create_publisher(JointCommand, '/joint_stream_controller/command', qos)

		# 往返序列 (限位区间百分比): 35% -> 65% -> 20% -> 零位(夹进限位)
		lo = [l for l, _ in lims]
		hi = [h for _, h in lims]
		frac = lambda f: [l + f * (h - l) for l, h in zip(lo, hi)]
		waypoints = [frac(0.35), frac(0.65), frac(0.20),
		             [min(max(0.0, l), h) for l, h in zip(lo, hi)]]

		# 段时长按关节跨度保守推算 (最大跨度/1 rad/s, 3~8s 夹紧), 段内 s 曲线平滑过渡
		prev = [min(max(0.0, l), h) for l, h in zip(lo, hi)]
		plan = []
		for wp in waypoints:
			span = max(abs(p - q) for p, q in zip(wp, prev))
			dur = max(3.0, min(8.0, span * 1.5 + 1.0))
			plan.append((prev, wp, dur))
			prev = wp
		total = sum(d for _, _, d in plan)
		self.get_logger().info('发送点流 @%dHz (%d 段, 约 %.0f 秒)...' % (hz, len(plan), total))

		# 消息关节集 = 直控 + mimic (JS 契约: 必须列全命令关节)
		msg_names = joints + [m[0] for m in mimics]

		def full_positions(direct_pos):
			pos = list(direct_pos)
			cur = dict(zip(joints, direct_pos))
			for name, master, k, off in mimics:
				pos.append(k * cur.get(master, 0.0) + off)
			return pos

		per_msg_timeout = 1.0 / hz
		for prev, wp, dur in plan * cycles:
			t0 = time.time()
			i = 0
			while True:
				elapsed = time.time() - t0
				if elapsed >= dur:
					break
				# smoothstep 过渡 (s 曲线, 起停零速度 —— 点流源头的运动学礼貌)
				s = min(1.0, elapsed / dur)
				s = s * s * (3.0 - 2.0 * s)
				m = JointCommand()
				m.mode = JointCommand.MODE_CSP
				m.joint_names = msg_names
				m.position = full_positions(
					[p + s * (w - p) for p, w in zip(prev, wp)])
				pub.publish(m)
				i += 1
				nxt = t0 + i * per_msg_timeout
				if nxt > time.time():
					time.sleep(nxt - time.time())
				rclpy.spin_once(self, timeout_sec=0)
		self.get_logger().info('演示流结束 (控制器按断流策略受控减速收尾)')


def main(args=None):
	hz = 50
	cycles = 1
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--hz':
			hz = int(argv.pop(0))
		elif a == '--cycles':
			cycles = int(argv.pop(0))
	rclpy.init(args=args)
	node = DemoMotion()
	try:
		node.run(hz, cycles)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

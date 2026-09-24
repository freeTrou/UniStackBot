#!/usr/bin/env python3
"""给 JointStream 发送一段大幅快慢变速演示运动 (正弦过渡的点流), 机型无关。

2026-09-24 运动内容 (用户需求: >10s 长时 + 大幅 + 有快有慢): 四段计划
85% → 15% → 85% → 零位(内缩), 快腿 2.0/2.5s 与慢腿 6.0/4.0s 交替, 共 ~14.5s。

2026-09-21 重写: JTC action 退役 → JointCommand 点流 (点流语义: 每条消息=最新目标,
控制器端每周期步长饱和逼近——上层只管把目标流平滑地发下来)。

三 demo = 三种命令域覆盖 (设计 §16.2 透传原则, 2026-09-23):
  demo_cartesian.py                末端位姿流 (IK 伺服路径, 慢于总线)
  demo_motion.py --hz 50 (默认)    关节慢流 (率失配 → JS ruckig 填充)
  demo_joint_fullrate (C++)        关节满速流 (= update_rate; rclpy 到不了 500Hz,
                                   满速域必须 C++, 见 demo_motion --hz 高值时的诚实报告)

关节表/限位解析自 robot_state_publisher 的 robot_description (URDF <ros2_control> 块,
带 position 命令接口的关节, 与 JointStream 解析口径一致), 路径点按限位区间百分比推算
—— 本脚本不含任何机型硬编码。

前置: 三链任一已启动 (control/ign/mujoco.launch.py) 且 joint_stream_controller active。

用法: ros2 run unistackbot_demo demo_motion.py [--hz 50] [--cycles 1]
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
		if fut.result() is None:
			raise RuntimeError('参数 %s/%s 调用 5 秒无响应 (CM 忙/链路退化)' % (node_name, name))
		return fut.result().values[0]

	def _fetch_contract(self):
		"""关节表 + 每关节 (min, max), 全部来自运行中的链路, 不本地写死。

		mimic 关节 (mock 链手指: 有命令接口但无 min/max, 只有 mimic/multiplier)
		也必须列全 —— JS 契约要求消息含全部命令关节; 其目标 = multiplier×主关节+offset。
		分类口径与 C++ (urdf_command_joints.hpp / demo_joint_fullrate) 一致: mimic 先判。
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
			if 'mimic' in ps:
				mimics.append((name, ps['mimic'],
				               float(ps.get('multiplier', '1')), float(ps.get('offset', '0'))))
			elif 'min' in ps and 'max' in ps:
				joints.append(name)
				lims.append((float(ps['min']), float(ps['max'])))
			else:
				# 静默丢弃 = JS 按长度拒整条消息 → demo 发流臂不动零诊断 (审查实锤)
				raise RuntimeError("命令关节 '%s' 无 min/max 也无 mimic 参数, 无法归类" % name)
		if not joints:
			raise RuntimeError('URDF ros2_control 块无 position 命令关节')
		masters = set(joints)
		for name, master, _, _ in mimics:
			if master not in masters:
				raise RuntimeError("mimic 关节 '%s' 的主关节 '%s' 不在直控关节表" % (name, master))
		return joints, lims, mimics

	def run(self, hz, cycles):
		joints, lims, mimics = self._fetch_contract()
		self.get_logger().info('关节表 (读自链上 URDF): %s%s' % (
			joints, ' + mimic %s' % [m[0] for m in mimics] if mimics else ''))

		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
		                 history=HistoryPolicy.KEEP_LAST, depth=1)
		pub = self.create_publisher(JointCommand, '/joint_stream_controller/command', qos)

		# 大幅快慢变速序列 (2026-09-24 用户需求: >10s 长时 + 大幅 + 有快有慢):
		# 显式段计划 (限位区间百分比目标, 段时长) —— 快腿/慢腿交替, 同跨度下
		# 命令速度对比 ~3×; 段内 smoothstep 过渡 (起停零速度)。
		# 内缩原因: gz 链命令精确停限位会触发限位咬死 (gz_ros2_control #165 残余),
		# 零位恰在 piper joint2/joint3 的限位线上
		lo = [l for l, _ in lims]
		hi = [h for _, h in lims]
		frac = lambda f: [l + f * (h - l) for l, h in zip(lo, hi)]
		def inset(v, l, h):
			return min(max(v, l + 0.02 * (h - l)), h - 0.02 * (h - l))
		home = [inset(0.0, l, h) for l, h in zip(lo, hi)]
		schedule = [
			(frac(0.85), 2.0, '快'),   # 部署: 大跨快腿
			(frac(0.15), 6.0, '慢'),   # 摆到对侧: ~70% 限位区间慢腿
			(frac(0.85), 2.5, '快'),   # 回摆: 全跨度快腿
			(home, 4.0, '慢'),         # 收尾: 慢腿归位
		]
		plan = []
		prev = list(home)
		for wp, dur, tag in schedule:
			plan.append((prev, wp, dur, tag))
			prev = wp
		total = sum(d for _, _, d, _ in plan)
		self.get_logger().info('大幅快慢序列: %d 段 / %.1fs (%s)' % (
			len(plan), total, ' → '.join('%s %.1fs' % (t, d) for _, _, d, t in plan)))
		self.get_logger().info('发送点流 @%dHz...' % hz)

		# 消息关节集 = 直控 + mimic (JS 契约: 必须列全命令关节)
		msg_names = joints + [m[0] for m in mimics]

		def full_positions(direct_pos):
			pos = list(direct_pos)
			cur = dict(zip(joints, direct_pos))
			for name, master, k, off in mimics:
				pos.append(k * cur.get(master, 0.0) + off)
			return pos

		per_msg_timeout = 1.0 / hz
		sent = 0
		late = 0
		t_start = time.time()
		for prev, wp, dur, _tag in plan * cycles:
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
				sent += 1
				i += 1
				# 绝对时间步进 (相对 sleep 会累积漂移, 矩阵测试的 x 轴要诚实)
				nxt = t0 + i * per_msg_timeout
				delay = nxt - time.time()
				if delay > 0.0:
					time.sleep(delay)
				else:
					late += 1
		wall = time.time() - t_start
		achieved = sent / wall if wall > 0.0 else 0.0
		late_note = ('; %d 拍落后 (rclpy 跟不上, 高频域请用 demo_joint_fullrate)' % late) if late else ''
		self.get_logger().info('演示流结束: 名义 %dHz, 实际达成 %.1fHz (%d 条 / %.1fs)%s' % (
			hz, achieved, sent, wall, late_note))
		self.get_logger().info('收尾: 控制器按断流策略受控减速')


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
		else:
			raise SystemExit("未知参数 '%s' (支持: --hz <n> --cycles <n>)" % a)
	rclpy.init(args=args)
	node = DemoMotion()
	try:
		node.run(hz, cycles)
	finally:
		node.destroy_node()
		try:
			rclpy.shutdown()
		except Exception:
			pass   # Ctrl-C 时 context 已关, 二次 shutdown 会抛 (守护, 不掩盖主流程)


if __name__ == '__main__':
	main()

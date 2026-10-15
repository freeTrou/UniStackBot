#!/usr/bin/env python3
"""r1_pro 关节控制 demo (机型专属; scripts/<robot>_arm/ 约定, 2026-10-12)。

**定位 = 控制开发人员的接入参考实现** (链路契约的完整正确消费样板);
本件是**四件式首个样板**: 双 JS 实例并发消费 —— js_arms 一条消息原子驱动
双臂 14 关节 (协同语义), js_torso 独立流驱动腰 4 关节。
最小入门请看 scripts/minimal/arms/ (外置通用教学件)。

航点六节: 双臂展示→同向扫→镜像扫→腰部扫→臂腰协同→回近零 (~20s)。
关节表/限位读自链上 URDF (单一事实源); 航点启动时逐点断言在限位 2% 内缩带。

前置: 任一链已启动, js_arms/js_torso active (demo 自会切换)。
用法: ros2 run unistackbot_demo r1_pro_demo_joint.py [--hz 50] [--cycles 1]
"""

import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from unistackbot_interface.msg import JointCommand

# ---- r1_pro 航点 (arms = 左 7 + 右 7 顺序, torso = 4; 启动时逐点断言限位) ----
# 右臂 j2 限位与左臂镜像 (左 [-0.14,3.11] / 右 [-3.11,0.14]) —— 对称姿态右臂 j2 取负
L_SHOW = (0.0, 1.0, 0.8, -0.8, 0.0, 0.0, 0.0)
R_SHOW = (0.0, -1.0, 0.8, -0.8, 0.0, 0.0, 0.0)
L_SWEEP = (0.6, 1.0, 0.8, -0.8, 0.4, 0.0, 0.5)     # 同向扫: 双臂同指一侧
R_SWEEP = (0.6, -1.0, 0.8, -0.8, 0.4, 0.0, 0.5)
L_MIRROR = (-0.6, 1.0, 0.8, -0.8, -0.4, 0.0, -0.5)   # 镜像扫: 双臂外展
R_MIRROR = (0.6, -1.0, 0.8, -0.8, 0.4, 0.0, 0.5)
T_ZERO = (0.0, 0.0, 0.0, 0.0)
T_SWEEP_L = (0.5, 0.0, 0.0, 0.6)                     # 腰部扫: t1/t4 (全向旋转腰)
T_SWEEP_R = (-0.5, 0.0, 0.0, -0.6)
L_SYNC = (-0.6, 1.0, 0.8, -0.8, 0.0, 0.0, 0.0)       # 臂腰协同: 臂 j1 与腰 t1 反向
R_SYNC = (0.6, -1.0, 0.8, -0.8, 0.0, 0.0, 0.0)
T_SYNC = (0.5, 0.0, 0.0, 0.0)
L_NEAR_ZERO = (0.0, 0.08, 0.06, -0.06, 0.0, 0.0, 0.0)   # 近零位 (2% 内缩)
R_NEAR_ZERO = (0.0, -0.08, 0.06, -0.06, 0.0, 0.0, 0.0)
T_NEAR_ZERO = (0.0, 0.05, 0.05, 0.05)

# 航点链: (arms 14 元组, torso 4 元组, 时长, 标签)
ARMS = lambda l, r: l + r
SCHEDULE = [
	(ARMS(L_SHOW, R_SHOW), T_ZERO, 2.5, '零位→双臂展示·慢'),
	(ARMS(L_SWEEP, R_SWEEP), T_ZERO, 0.9, '双臂同向扫·快'),
	(ARMS(L_SHOW, R_SHOW), T_ZERO, 0.9, '回展示·快'),
	(ARMS(L_MIRROR, R_MIRROR), T_ZERO, 0.9, '双臂镜像外展·快'),
	(ARMS(L_SHOW, R_SHOW), T_ZERO, 0.9, '回展示·快'),
	(ARMS(L_SHOW, R_SHOW), T_SWEEP_L, 1.6, '腰部左扫·慢'),
	(ARMS(L_SHOW, R_SHOW), T_SWEEP_R, 1.6, '腰部右扫·慢'),
	(ARMS(L_SHOW, R_SHOW), T_ZERO, 1.2, '腰回正·快'),
	(ARMS(L_SYNC, R_SYNC), T_SYNC, 1.4, '臂腰协同·慢'),
	(ARMS(L_SHOW, R_SHOW), T_ZERO, 1.2, '回展示·快'),
	(ARMS(L_NEAR_ZERO, R_NEAR_ZERO), T_NEAR_ZERO, 2.5, '回近零位·慢'),
]


class R1ProDemoJoint(Node):

	def __init__(self):
		super().__init__('r1_pro_demo_joint')

	def _fetch_contract(self):
		"""关节表+限位读自链上 URDF <ros2_control> 块 (与 JS 解析同口径),
		按名分组: torso_joint* → js_torso (4), *_arm_joint* → js_arms (14)。"""
		cli = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
		if not cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError('参数服务 /robot_state_publisher 5 秒内未出现 (链路未起?)')
		req = GetParameters.Request()
		req.names = ['robot_description']
		fut = cli.call_async(req)
		rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
		if fut.result() is None or not fut.result().values[0].string_value:
			raise RuntimeError('读取 robot_description 失败')
		lims = {}
		for j in ET.fromstring(fut.result().values[0].string_value).iter('joint'):
			ci = j.find('command_interface')
			if j.get('name') is None or ci is None or ci.get('name') != 'position':
				continue
			ps = {p.get('name'): p.text for p in j.findall('param')}
			if 'min' in ps and 'max' in ps:
				lims[j.get('name')] = (float(ps['min']), float(ps['max']))
		arms = [n for n in lims if 'arm_joint' in n]
		torso = [n for n in lims if 'torso_joint' in n]
		# 顺序锚定: 左臂 7 (j1..j7) → 右臂 7 —— 与航点元组顺序一致
		arms = sorted(arms, key=lambda n: (n.startswith('right'), int(n.split('joint')[-1])))
		torso = sorted(torso, key=lambda n: int(n.split('joint')[-1]))
		if len(arms) != 14 or len(torso) != 4:
			raise RuntimeError('命令关节分组 %d+%d != 14+4 (%s | %s) —— 链上不是 r1_pro?'
				% (len(arms), len(torso), arms, torso))
		return arms, torso, lims

	def _read_start(self, joints):
		"""起始关节角, 读 /joint_states 按名对齐 (JSB 乱序已知坑)。"""
		got = {}
		sub = self.create_subscription(JointState, '/joint_states',
			lambda m: got.update(zip(m.name, m.position)), 10)
		t0 = time.time()
		while len(got) < 18 and time.time() - t0 < 3.0:
			rclpy.spin_once(self, timeout_sec=0.1)
		self.destroy_subscription(sub)
		if len(got) < 18:
			raise RuntimeError('/joint_states 3 秒未凑齐关节表 (链路未起?)')
		return [got[n] for n in joints]

	def _ensure_js_active(self):
		"""进入声明 (幂等+两段式): CM 若 active 先停落定, 再激活两个 JS 实例
		(单 flag 多值 —— argparse 重复同名 flag 会后者覆盖前者, 正确姿势)。"""
		for attempt in range(7):
			r = subprocess.run(['ros2', 'control', 'list_controllers'],
				capture_output=True, text=True, timeout=15)
			states = {}
			for line in r.stdout.splitlines():
				clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip())
				parts = clean.split()
				if len(parts) >= 3:
					states[parts[0]] = parts[-1]
			if states.get('js_arms') == 'active' and states.get('js_torso') == 'active' \
				and states.get('cartesian_motion_controller') != 'active':
				return
			if states.get('cartesian_motion_controller') == 'active':
				subprocess.run(['ros2', 'control', 'switch_controllers', '--deactivate',
					'cartesian_motion_controller'], capture_output=True, timeout=15)
				time.sleep(0.5)
			subprocess.run(['ros2', 'control', 'switch_controllers', '--activate',
				'js_arms', 'js_torso'], capture_output=True, timeout=15)
			time.sleep(1.0)
		raise RuntimeError('切换 JS 失败 (7 次尝试)')

	def run(self, hz, cycles):
		self._ensure_js_active()
		arms, torso, lims = self._fetch_contract()
		joints = arms + torso
		self.get_logger().info('关节编组: js_arms %s | js_torso %s' % (arms, torso))
		# 航点限位断言 (2% 内缩带): 常量写错在启动期暴露, 不进运行时
		for wp, tw, _dur, tag in SCHEDULE:
			for n, q in zip(joints, wp + tw):
				lo, hi = lims[n]
				if not (lo + 0.02 * (hi - lo) <= q <= hi - 0.02 * (hi - lo)):
					raise RuntimeError('航点 %s 的 %s=%+.3f 越出限位内缩带 [%.3f, %.3f]'
						% (tag, n, q, lo + 0.02 * (hi - lo), hi - 0.02 * (hi - lo)))
		start = self._read_start(joints)
		total = sum(d for _, _, d, _ in SCHEDULE) * cycles
		self.get_logger().info('r1_pro 关节演示: %d 段 / %.1fs (%s)' % (
			len(SCHEDULE) * cycles, total, ' → '.join(t for _, _, _, t in SCHEDULE)))

		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
		                 history=HistoryPolicy.KEEP_LAST, depth=1)
		pub_arms = self.create_publisher(JointCommand, '/js_arms/command', qos)
		pub_torso = self.create_publisher(JointCommand, '/js_torso/command', qos)

		def stream(pub, names, prev, wp, s):
			m = JointCommand()
			m.mode = JointCommand.MODE_CSP
			m.joint_names = names
			m.position = [p + s * (w - p) for p, w in zip(prev, wp)]
			pub.publish(m)

		per_msg = 1.0 / hz
		sent = late = 0
		t_start = time.time()
		prev_a, prev_t = start[:14], start[14:]
		for _ in range(cycles):
			for wp, tw, dur, _tag in SCHEDULE:
				t0 = time.time()
				i = 0
				while True:
					elapsed = time.time() - t0
					if elapsed >= dur:
						break
					s = min(1.0, elapsed / dur)
					s = s * s * (3.0 - 2.0 * s)   # smoothstep (起停零速度)
					stream(pub_arms, arms, prev_a, wp, s)     # 双臂一条消息 = 原子协同
					stream(pub_torso, torso, prev_t, tw, s)
					sent += 1
					i += 1
					nxt = t0 + i * per_msg   # 绝对时间步进 (相对 sleep 累积漂移)
					delay = nxt - time.time()
					if delay > 0.0:
						time.sleep(delay)
					else:
						late += 1
				prev_a, prev_t = list(wp), list(tw)
		wall = time.time() - t_start
		achieved = sent / wall if wall > 0.0 else 0.0
		note = ('; %d 拍落后' % late) if late else ''
		self.get_logger().info('演示流结束: 名义 %dHz, 实际达成 %.1fHz (%d 拍 / %.1fs)%s' % (
			hz, achieved, sent, wall, note))
		self.get_logger().info('收尾: 停在近零位, 控制器按断流语义保持')


def main():
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
	rclpy.init(args=None)
	node = R1ProDemoJoint()
	try:
		node.run(hz, cycles)
	finally:
		node.destroy_node()
		try:
			rclpy.shutdown()
		except Exception:
			pass   # Ctrl-C 时 context 已关 (守护, 不掩盖主流程)


if __name__ == '__main__':
	main()

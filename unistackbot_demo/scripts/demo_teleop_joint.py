#!/usr/bin/env python3
"""键盘关节域遥操作 (命令域②交互版) —— 每键=一个关节往上限/下限方向, 直发 JS。

设计裁决 (2026-10-08, 用户提出): 笛卡尔键盘遥操 (demo_teleop_key) 的别扭是
结构性的 —— 6-DoF 意图经 IK 翻译, 有奇异减速/姿态冻结重构/坐标系心理旋转
三重损耗。关节域直发绕开全部: 键=关节=方向, 所见即所得; 不过 IK 无奇异问题;
顶限位=贴住停住 (JS ruckig 收敛 + write 层 clamp), 无"诚实拒绝"困惑。

键位 (一列一关节, 上排往上限, 下排往下限, 直控关节序):
  Q W E R T Y U   ← J1..J7 正方向
  A S D F G H J   ← J1..J7 负方向
  -/= 速度档   Shift=该键 2×   Space=清键急停   Ctrl-C=退出
直控关节 >7 时多余关节只保持不控 (双臂机未来工作项)。

链路: 键态 → 每关节速度 → 目标矢量自推进 (目标空间积分, 同 demo_teleop_key
的教训——反馈重锚会让 JS 收敛动力学吞掉指令速度) → 30Hz 发 JointCommand CSP
点流 (JS ruckig 档 C2 填充) → 松键断流 → JS 断流受控刹停 (链上 200ms)。

关节表/限位/mimic 全部自持解析链上 URDF (与 demo_motion 同口径, mimic 先判;
mimic 目标 = multiplier×主关节+offset 随消息补全)。零机型硬编码。

前置: 三链任一已启动 (JS 本就 active, 无需切换)。mujoco/gz 链注意: 命令精确
压限位静置在 gz 链会触发关节冻结 (上游 #165), 目标钳位保留 2% 内缩规避。

用法: ros2 run unistackbot_demo demo_teleop_joint.py
        [--hz 30] [--speed 0.25,0.5,1.0] [--deadman 0.15]
"""

import os
import select
import sys
import termios
import time
import tty
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from unistackbot_interface.msg import JointCommand

DEADMAN_S = 0.15          # 重复续命窗口 (松键判定; 停流=收敛最后目标保持)
FRESH_STREAM_S = 0.45     # 新按下保底发流窗 (覆盖 OS 键重复初始延迟 ~0.3-0.6s)
JS_SILENT_S = 1.0         # /joint_states 静默超时
LIMIT_INSET = 0.98        # 目标钳位保留 2% 内缩 (gz 链限位冻结规避, demo_motion 同款)

UP_KEYS = 'qwertyu'       # 每列上排 = 正方向 (往上限), 7 列
DOWN_KEYS = 'asdfghj'     # 每列下排 = 负方向 (往下限)
SHIFT_FACTOR = 2.0


class TeleopJoint(Node):

	def __init__(self):
		super().__init__('demo_teleop_joint')
		self.q = {}               # name → 实测 rad (按名对齐, JSB 乱序免疫)
		self.q_t = 0.0
		self.create_subscription(JointState, '/joint_states', self._on_js, 10)
		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
		                 history=HistoryPolicy.KEEP_LAST, depth=1)
		self.pub = self.create_publisher(JointCommand,
		                                 '/joint_stream_controller/command', qos)
		self.sent_n = 0
		self.t_first = None

	def _get_param(self, node_name, name):
		cli = self.create_client(GetParameters, node_name + '/get_parameters')
		if not cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError('参数服务 %s 5 秒内未出现 (链路未起?)' % node_name)
		req = GetParameters.Request()
		req.names = [name]
		fut = cli.call_async(req)
		rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
		if fut.result() is None:
			raise RuntimeError('参数 %s 调用无响应' % node_name)
		return fut.result().values[0]

	def fetch_contract(self):
		"""直控关节表+限位+mimic (demo_motion 同口径: mimic 先判, 列全才发)。"""
		uv = self._get_param('/robot_state_publisher', 'robot_description')
		if not uv.string_value:
			raise RuntimeError('读取 robot_description 失败')
		joints, lims, mimics = [], [], []
		for j in ET.fromstring(uv.string_value).iter('joint'):
			if j.get('name') is None or j.find('command_interface') is None:
				continue
			if j.find('command_interface').get('name') != 'position':
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
				raise RuntimeError("命令关节 '%s' 无 min/max 也无 mimic, 无法归类" % name)
		if not joints:
			raise RuntimeError('URDF ros2_control 块无 position 命令关节')
		masters = set(joints)
		for name, master, _, _ in mimics:
			if master not in masters:
				raise RuntimeError("mimic '%s' 主关节 '%s' 不在直控表" % (name, master))
		return joints, lims, mimics

	def _on_js(self, msg):
		for name, pos in zip(msg.name, msg.position):
			self.q[name] = pos
		self.q_t = time.monotonic()

	def publish(self, tgt, joints, mimics):
		m = JointCommand()
		m.mode = JointCommand.MODE_CSP
		m.joint_names = list(joints) + [mm[0] for mm in mimics]
		pos = list(tgt)
		for _name, master, k, off in mimics:
			pos.append(k * tgt[joints.index(master)] + off)
		m.position = pos
		self.pub.publish(m)
		self.sent_n += 1
		if self.t_first is None:
			self.t_first = time.monotonic()


def short_name(name):
	"""HUD 紧凑显示: 厂商前缀剥离 (joint1/J1/J2... → J1..)。"""
	tail = name.split('_')[-1].lstrip('jJ')
	try:
		return 'J%d' % int(tail)
	except ValueError:
		return name[:4]


def active_cols(now_hold, now, speed):
	"""活动列 → {列号: 带符号速度}。Shift 大写=2×, 合成同列互抵。"""
	cols = {}
	for i in range(len(UP_KEYS)):
		d = 0.0
		for kb, sign in ((ord(UP_KEYS[i]), 1.0), (ord(DOWN_KEYS[i]), -1.0)):
			t = now_hold.get(kb)
			if t is not None and now < t:
				f = SHIFT_FACTOR if 0x41 <= kb <= 0x5A else 1.0
				d += sign * f
		if abs(d) > 1e-9:
			cols[i] = max(-SHIFT_FACTOR, min(SHIFT_FACTOR, d)) * speed
	return cols


def fmt_hud(tgt, joints, lims, cols, level, n_lv, speed, clamped):
	parts = []
	for i, name in enumerate(joints[:7]):
		tag = short_name(name)
		if i in cols:
			tag += '↑' if cols[i] > 0 else '↓'
		if i in clamped:
			tag += '■'
		parts.append('%s%+.2f' % (tag, tgt[i]))
	return '\r[关节遥操] %s | 档%d/%d %.2frad/s | [Space急停 Ctrl-C退]\x1b[K' % (
		' '.join(parts), level + 1, n_lv, speed)


def run(node, hz, speeds, joints, lims, mimics):
	period = 1.0 / hz
	lo = [l for l, _ in lims]
	hi = [h for _, h in lims]
	level = 1
	now_hold = {}
	tgt = None            # 目标矢量 (目标空间积分; None=下次发流前重锚实测)
	tgt_active = False
	clamped = set()
	fd = sys.stdin.fileno()
	old_term = termios.tcgetattr(fd)
	try:
		tty.setcbreak(fd)
		# 清 ISIG: Ctrl-C 走带内字节 (上方 0x03 处理), 真实终端与 pty 同语义;
		# cbreak 默认 ISIG 会在线路规程吞掉 \x03, pty 无会话时信号无家可归
		_a = termios.tcgetattr(fd)
		_a[3] &= ~termios.ISIG
		termios.tcsetattr(fd, termios.TCSANOW, _a)
		node.get_logger().info(
			'关节列: %s' % '  '.join('%s/%s=%s±' % (
				UP_KEYS[i].upper(), DOWN_KEYS[i], short_name(n))
				for i, n in enumerate(joints[:7])))
		next_t = time.monotonic()
		while True:
			now = time.monotonic()
			r, _, _ = select.select([fd], [], [], max(0.0, next_t - now))
			if r:
				for ch in os.read(fd, 32):
					was_stale = now_hold.get(ch) is None or \
						time.monotonic() >= now_hold.get(ch, 0)
					now_hold[ch] = time.monotonic() + (FRESH_STREAM_S if was_stale
					                                   else DEADMAN_S)
					if ch == 0x03:   # Ctrl-C 带内处理 (pty 无会话时线路规程不转 SIGINT)
						raise KeyboardInterrupt
					if ch == ord(' '):
						now_hold.clear()   # 急停: 清键 → 停流 → JS 断流刹停
					elif ch in (ord('-'), ord('_')):
						level = max(0, level - 1)
					elif ch in (ord('='), ord('+')):
						level = min(len(speeds) - 1, level + 1)
			now = time.monotonic()
			speed = speeds[level]
			cols = active_cols(now_hold, now, speed)
			streaming = False
			if cols and node.q and now - node.q_t <= JS_SILENT_S:
				if not tgt_active or tgt is None:
					tgt = [node.q.get(n, 0.0) for n in joints]   # 边沿重锚实测
				clamped = set()
				for i, d in cols.items():
					tgt[i] += d * period
					raw = tgt[i]
					tgt[i] = max(lo[i] * LIMIT_INSET, min(hi[i] * LIMIT_INSET, tgt[i]))
					if abs(tgt[i] - raw) > 1e-9:
						clamped.add(i)
				node.publish(tgt, joints, mimics)
				tgt_active = True
				streaming = True
			else:
				tgt_active = False   # 松键: 停流 → JS 断流刹停; 下次重锚实测
			sys.stdout.write(fmt_hud(tgt or [node.q.get(n, 0.0) for n in joints],
			                         joints, lims, cols, level, len(speeds),
			                         speed, clamped))
			sys.stdout.flush()
			next_t += period
			if next_t < time.monotonic() - period:
				next_t = time.monotonic()
	finally:
		sys.stdout.write('\r\x1b[K')
		sys.stdout.flush()
		termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
		wall = time.monotonic() - (node.t_first or time.monotonic())
		node.get_logger().info('汇总: 发目标 %d 条 / 活动窗 %.1fs' % (
			node.sent_n, wall))


def main(args=None):
	hz = 30.0
	speeds_s = '0.25,0.5,1.0'
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--hz':
			hz = float(argv.pop(0))
		elif a == '--speed':
			speeds_s = argv.pop(0)
		elif a == '--deadman':
			global DEADMAN_S
			DEADMAN_S = float(argv.pop(0))
		else:
			raise SystemExit("未知参数 '%s' (支持: --hz --speed --deadman)" % a)
	if hz <= 0.0 or hz > 100.0:
		print('--hz 须在 (0,100]')
		return 2
	speeds = [float(x) for x in speeds_s.split(',')]
	if not speeds or any(v <= 0.0 for v in speeds):
		print('--speed 档须全为正')
		return 2
	if not sys.stdin.isatty():
		print('stdin 不是终端 —— 遥操作需要真实 TTY')
		return 2
	rclpy.init(args=args)
	node = TeleopJoint()
	try:
		joints, lims, mimics = node.fetch_contract()
		node.get_logger().info('关节表 (读自链上 URDF): %s%s' % (
			joints, ' + mimic %s' % [m[0] for m in mimics] if mimics else ''))
		if len(joints) > 7:
			node.get_logger().warn('%d 个直控关节 > 7 列, 多余关节只保持不控' % len(joints))
		executor = SingleThreadedExecutor()
		executor.add_node(node)
		import threading
		spin = threading.Thread(target=executor.spin, daemon=True)
		spin.start()
		try:
			run(node, hz, speeds, joints, lims, mimics)
		except KeyboardInterrupt:
			pass
		finally:
			executor.remove_node(node)
			executor.shutdown()
			spin.join(timeout=2.0)
			node.destroy_node()
	finally:
		if rclpy.ok():
			rclpy.shutdown()
	return 0


if __name__ == '__main__':
	sys.exit(main())

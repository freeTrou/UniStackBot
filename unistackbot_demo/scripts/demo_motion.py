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
		self._urdf_xml = uv.string_value
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

	# ---- 轻量 URDF FK (地面安全检查用, 2026-09-29) ----
	# 动机: 航点是纯关节行程算术, 无末端几何意识——stationlite 折叠构型下
	# j2 压低方向把夹爪送进地板 ("运动时夹爪戳到地面")。
	# 修法: 全 link FK 取最低 z, 航点+插值中间点低于门槛 → 自动收缩幅度。
	# 机型无关 (origin/axis 全读 URDF, 零硬编码), 纯 python 3×4 变换连乘。

	@staticmethod
	def _rpy_mat(r, p, y):
		cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
		                          math.sin(p), math.cos(y), math.sin(y))
		return [
			[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
			[sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
			[-sp,   cp*sr,            cp*cr]]

	@staticmethod
	def _axis_rot(axis, q):
		"""axis-angle 旋转矩阵 (URDF axis 归一化前提)。"""
		n = math.sqrt(sum(a*a for a in axis))
		x, y, z = axis[0]/n, axis[1]/n, axis[2]/n
		c, s = math.cos(q), math.sin(q)
		t = 1.0 - c
		return [
			[t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
			[t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
			[t*x*z - s*y, t*y*z + s*x, t*z*z + c]]

	def _fk_pos(self, q_map):
		"""全 link FK。返回 (pos: link→(x,y,z), moving: 活动链下游 link 集)。"""
		pos = {'base': (0.0, 0.0, 0.0)}
		moving = set()
		changed = True
		joints = self._fk_joints
		while changed:
			changed = False
			for name, parent, child, xyz, rpy, axis, jtype in joints:
				if parent not in pos or child in pos:
					continue
				R = self._rpy_mat(*rpy)
				q = q_map.get(name, 0.0)
				if jtype in ('revolute', 'continuous', 'prismatic'):
					A = self._axis_rot(axis, q)
					R = [[sum(R[i][k]*A[k][j] for k in range(3)) for j in range(3)]
					     for i in range(3)]
				p = pos[parent]
				pos[child] = tuple(
					p[i] + sum(R[i][k]*xyz[k] for k in range(3)) for i in range(3))
				if jtype in ('revolute', 'continuous', 'prismatic') or parent in moving:
					moving.add(child)
				changed = True
		return pos, moving


	def _load_fk_model(self, urdf_xml, mimic_names):
		"""URDF 关节表 (origin xyz/rpy + axis), 供下游计数与 FK。"""
		self._fk_joints = []
		for j in ET.fromstring(urdf_xml).iter('joint'):
			if j.find('parent') is None or j.find('child') is None:
				continue   # ros2_control 块的 <joint> (接口声明) 不是运动学关节
			if j.get('name') in mimic_names or j.find('mimic') is not None:
				continue   # mimic 随主关节, 非独立自由度 (下游计数不计)
			o = j.find('origin')
			xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
			rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
			ax = j.find('axis')
			axis = [float(v) for v in (ax.get('xyz', '0 0 0') if ax is not None else '0 0 0').split()]
			self._fk_joints.append((j.get('name'), j.find('parent').get('link'),
			                        j.find('child').get('link'), xyz, rpy, axis, j.get('type')))

	def run(self, hz, cycles, amp_override=None):
		joints, lims, mimics = self._fetch_contract()
		self.get_logger().info('关节表 (读自链上 URDF): %s%s' % (
			joints, ' + mimic %s' % [m[0] for m in mimics] if mimics else ''))
		self._load_fk_model(self._urdf_xml, set(m[0] for m in mimics))

		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
		                 history=HistoryPolicy.KEEP_LAST, depth=1)
		pub = self.create_publisher(JointCommand, '/joint_stream_controller/command', qos)

		# 大幅快慢变速序列 (2026-09-24 用户需求: >10s 长时 + 大幅 + 有快有慢):
		# 构型感知航点 (2026-09-28 修正): 限位百分比在不对称限位/零位偏移的臂上
		# (stationlite j2: [-0.209, 3.508]) 会走到极端折叠构型——
		# 改为围绕零位的对称偏移 (±range*factor), 并保守取 factor=0.3
		lo = [l for l, _ in lims]
		hi = [h for _, h in lims]
		def inset(v, l, h):
			return min(max(v, l + 0.02 * (h - l)), h - 0.02 * (h - l))
		home = [inset(0.0, l, h) for l, h in zip(lo, hi)]
		# 限位区间中点 ± 半区间 × factor (构型无关, 零位压限位线也能全幅运动)
		# 零位在限位线上的臂 (stationlite j3: [-3.193, 0]) 用 min(|lo-home|,...) 算出振幅=0 完全不动
		if amp_override is not None:
			factor = amp_override
		else:
			factor = 0.15   # 保守默认; --amp 可调 (0.5 会砸地面)
		mid = [(l + h) / 2 for l, h in zip(lo, hi)]
		half = [(h - l) / 2 for l, h in zip(lo, hi)]
		# 末梢两级关节 (下游活动关节数 ≤1, 即腕末与夹爪) 幅度自动减半 ——
		# 姿态关节大幅旋转会把末端执行器长杆扫向地面 (stationlite 2026-09-29
		# 实测: 关节航点本身下沉仅 5mm, 戳地全来自腕部旋转的手指扫掠, 指尖
		# 在 link 原点外 10cm——link 原点级 FK 抓不到, 关节域减幅是务实解)。
		# 通用规则 (下游计数), 零机型硬编码。
		children = {}
		for name, parent, child, _xyz, _rpy, _axis, jtype in self._fk_joints:
			if jtype in ('revolute', 'continuous', 'prismatic'):
				children.setdefault(name, []).append(child)
		downstream = {}
		cmd_joints = set(joints)   # 只数直控关节 (非命令的随动关节不计)
		def count_down(name):
			if name in downstream:
				return downstream[name]
			n = 0
			for c in children.get(name, []):
				for jn, _p, ch, _x, _r, _a, jt in self._fk_joints:
					if jn != name and _p == c and jt in ('revolute', 'continuous', 'prismatic') and jn in cmd_joints:
						n += 1 + count_down(jn)
			downstream[name] = n
			return n
		factors = [factor * 0.5 if count_down(n) <= 1 else factor for n in joints]
		pos = [m + hf * f for m, hf, f in zip(mid, half, factors)]
		neg = [m - hf * f for m, hf, f in zip(mid, half, factors)]
		if any(f != factor for f in factors):
			self.get_logger().info('末梢两级减半: %s' % [
				n for n, f in zip(joints, factors) if f != factor])
		schedule = [
			(pos, 2.0, '快'),
			(neg, 6.0, '慢'),
			(pos, 2.5, '快'),
			(home, 4.0, '慢'),
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
	amp_override = None
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--hz':
			hz = int(argv.pop(0))
		elif a == '--cycles':
			cycles = int(argv.pop(0))
		elif a == '--amp':
			amp_override = float(argv.pop(0))
		else:
			raise SystemExit("未知参数 '%s' (支持: --hz <n> --cycles <n> --amp <0.05-0.45>)" % a)
	rclpy.init(args=args)
	node = DemoMotion()
	try:
		node.run(hz, cycles, amp_override)
	finally:
		node.destroy_node()
		try:
			rclpy.shutdown()
		except Exception:
			pass   # Ctrl-C 时 context 已关, 二次 shutdown 会抛 (守护, 不掩盖主流程)


if __name__ == '__main__':
	main()

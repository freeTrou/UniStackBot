#!/usr/bin/env python3
"""r1_pro 笛卡尔控制 demo (机型专属; scripts/<robot>_arm/ 约定, 2026-10-12)。

**定位 = 控制开发人员的接入参考实现** (链路契约的完整正确消费样板);
本件是**四件式首个笛卡尔样板**: CM=左臂 (yaml base_link→left_arm_link7)。
完整工况: '@js' 通配切换 CM 接管 → 抬升 → 前伸 → 收回 → 切回全部 JS → 回关节近零位。
教科书点: CM 契约 QoS / 目标对账锚点 / 三结局判据 / IK 错误语义 (last_result
=IkResult + solver_health/arrival, RESULT_NAME 人话映射) / '@js' 形态盲切换。
最小入门请看 scripts/minimal/arms/ (外置通用教学件)。

**四件式当前语义 (诚实账)**: CM 认领腰 4+左臂 7 共 11 关节, 与 js_arms (双臂 14)
/ js_torso (腰 4) 接口互斥 —— CM 激活期间两个 JS 全停, **右臂冻结在最后命令位**
(接口独占的代价, 双臂单 CM 裁决的现状)。

前置: 任一链已启动。用法: ros2 run unistackbot_demo r1_pro_demo_cartesian.py
      [--lift 0.10] [--fwd 0.10]
"""

import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from unistackbot_interface.msg import CartesianMotionStatus, JointCommand

BASE, TIP = 'base_link', 'left_arm_link7'   # 机型专属 demo 用常量 (与 yaml 同口径)
CM_CMD = '/cartesian_motion_controller/command'
CM_STATUS = '/cartesian_motion_controller/status'

# status.last_result 透传的 IkResult 枚举 (码表: unistackbot_interface/ik_result.hpp)
RESULT_NAME = {1: 'UNREACHABLE', 2: 'NEAR_SINGULAR', 3: 'ITERATION_LIMIT',
               4: 'LIMIT_CONFLICT', 5: 'NOT_READY', 6: 'UNSUPPORTED'}

# 收尾回近零位 (2% 限位内缩; 双臂 + 腰 —— JS 消息必须列全该实例认领的关节)
ARMS_NEAR_ZERO = (0.0, 0.08, 0.06, -0.06, 0.0, 0.0, 0.0,
                  0.0, -0.08, 0.06, -0.06, 0.0, 0.0, 0.0)
TORSO_NEAR_ZERO = (0.0, 0.05, 0.05, 0.05)
ARMS_NAMES = ['left_arm_joint%d' % (i + 1) for i in range(7)] + \
             ['right_arm_joint%d' % (i + 1) for i in range(7)]
TORSO_NAMES = ['torso_joint%d' % (i + 1) for i in range(4)]


class R1ProDemoCartesian(Node):

	def __init__(self):
		super().__init__('r1_pro_demo_cartesian')
		self.status = {}       # 最近一帧 CM status 摘要
		self.status_t = 0.0    # 该帧到达时刻 (静默检测用)
		self.sent = None       # 对账锚点: 最近一次发布的目标位置

	# ---- 链路契约消费 (接入方照抄的三件套) ----

	@staticmethod
	def _controller_states():
		r = subprocess.run(['ros2', 'control', 'list_controllers'],
			capture_output=True, text=True, timeout=15)
		states = {}
		for line in r.stdout.splitlines():
			clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip())
			parts = clean.split()
			if len(parts) >= 3:
				states[parts[0]] = (parts[1], parts[-1])
		if not states:
			raise RuntimeError('list_controllers 无输出 —— 链未启动? 先 ros2 launch 再跑 demo')
		return states

	@staticmethod
	def _js_names(states, want_state):
		"""'@js' 通配: JointStream 实例按类型发现, 命名任意 (单块机型
		joint_stream_controller / 四件式 js_arms+js_torso), 形态盲。"""
		return sorted(n for n, (t, s) in states.items()
			if s == want_state and 'JointStreamController' in t)

	def ensure_controllers(self, activate, deactivate=(), retries=7):
		"""进入声明/退出礼让 (幂等+两段式): 先停占用方并落定, 再激活。
		名单里 '@js' = 全部 JointStream 实例 (按类型发现)。
		多实例单 flag 多值 (argparse 重复 flag 后者覆盖前者, 勿重复写)。"""
		for attempt in range(retries):
			states = self._controller_states()
			need_deact = (self._js_names(states, 'active') if '@js' in deactivate
				else [c for c in deactivate if states.get(c, ('', ''))[1] == 'active'])
			need_act = (self._js_names(states, 'inactive') if '@js' in activate
				else [c for c in activate if states.get(c, ('', ''))[1] != 'active'])
			if not need_deact and not need_act:
				return
			if need_deact:
				subprocess.run(['ros2', 'control', 'switch_controllers', '--deactivate'] + need_deact,
					capture_output=True, timeout=15)
				time.sleep(1.0)   # 接口释放落定 (gz 实测 >1s, 勿缩短)
			if need_act:
				s = subprocess.run(['ros2', 'control', 'switch_controllers', '--activate'] + need_act,
					capture_output=True, text=True, timeout=15)
				if s.returncode == 0:
					return
			self.get_logger().warn('switch 未就绪 (attempt %d/%d), 5s 重试' % (attempt + 1, retries))
			time.sleep(5.0)
		raise RuntimeError('switch 失败: %s' % s.stderr.strip()[-120:])

	def publish_target(self, pub, snapshot, dxyz):
		"""发一条 CM 目标 = TF 快照 + 偏移。QoS 必须 reliable+KeepLast(1):
		深度>1 会在消费端跟不上时积压陈旧目标, CM 追着旧目标走。"""
		t = PoseStamped()
		t.header.frame_id = BASE
		t.pose.position.x = snapshot.transform.translation.x + dxyz[0]
		t.pose.position.y = snapshot.transform.translation.y + dxyz[1]
		t.pose.position.z = snapshot.transform.translation.z + dxyz[2]
		t.pose.orientation = snapshot.transform.rotation
		self.status.clear()    # 只认发布之后的新 status
		self.sent = (t.pose.position.x, t.pose.position.y, t.pose.position.z)
		pub.publish(t)

	def wait_result(self, timeout=30.0, reject_after=1.5):
		"""等当前点位的三种结局之一:
		  ('conv', err)     误差进容差 (SETTLED 档, 双条件防抖动穿越假亮)
		  ('reject', code)  1.5s 仍不收敛且结果码非零 = 求解器在拒绝
		  ('plateau', err)  误差钉住但进不了容差 (物理链跟踪极限)
		"""
		t0 = time.time()
		err_ref = None
		while time.time() - t0 < timeout:
			rclpy.spin_once(self, timeout_sec=0.05)
			if self.status_t and time.time() - self.status_t > 5.0:
				raise RuntimeError('CM status 静默 >5s —— 链已死或 CM 未激活')
			tgt = self.status.get('tgt')
			if self.status.get('arrival') == 2 and self.sent and tgt and \
				max(abs(a - b) for a, b in zip(tgt, self.sent)) < 1e-9:
				return 'conv', self.status['err']
			if time.time() - t0 > reject_after and self.status.get('res', 0) != 0:
				return 'reject', self.status['res']
			err = self.status.get('err')
			if err is not None and time.time() - t0 > 3.0:
				if err_ref is None:
					err_ref = err
				elif abs(err - err_ref) < 0.1 * max(abs(err_ref), 1e-4):
					return 'plateau', err
		raise RuntimeError('点位 %.0fs 未收敛也未拒绝' % timeout)

	# ---- 演示动作 ----

	def goto(self, pub, snapshot, dxyz, tag):
		"""点到点一腿 (流式插值过去, 等结局)。返回 'conv'/'plateau'; 拒绝保持原位。"""
		cur = ((self.sent[0] - snapshot.transform.translation.x,
		        self.sent[1] - snapshot.transform.translation.y,
		        self.sent[2] - snapshot.transform.translation.z) if self.sent else (0.0, 0.0, 0.0))
		steps = 60   # ~1.2s @50Hz
		for i in range(1, steps + 1):
			a = i / steps
			a = a * a * (3.0 - 2.0 * a)   # smoothstep 起停零速度
			self.publish_target(pub, snapshot,
				tuple(c + (d - c) * a for c, d in zip(cur, dxyz)))
			rclpy.spin_once(self, timeout_sec=0)   # 流式期间持续消费 status 回调
			time.sleep(0.02)
		kind, v = self.wait_result()
		if kind == 'conv':
			self.get_logger().info('%s: 收敛 err=%.3fmm' % (tag, v * 1000.0))
			return 'conv'
		if kind == 'plateau':
			self.get_logger().warn('%s: 稳态 err=%.3fmm (物理链正常), 继续' % (tag, v * 1000.0))
			return 'plateau'
		sh = self.status.get('sh', 1)
		self.get_logger().warn('%s: 诚实拒绝 (result=%s %s, solver_health=%s), 保持原位' % (
			tag, v, RESULT_NAME.get(v, '?'), {0: 'NONE', 1: 'OK', 2: 'NEAR_SINGULAR'}.get(sh, sh)))
		return 'reject'

	def send_zero(self):
		"""收尾回近零位: 切回 JS 后各实例一条 CSP 目标 (消息必须列全该实例认领的
		关节 —— js_arms 14 / js_torso 4), ruckig 平滑收敛 (断流语义)。"""
		pub_a = self.create_publisher(JointCommand, '/js_arms/command', 10)
		pub_t = self.create_publisher(JointCommand, '/js_torso/command', 10)
		ma, mt = JointCommand(), JointCommand()
		ma.mode = mt.mode = JointCommand.MODE_CSP
		ma.joint_names, ma.position = ARMS_NAMES, list(ARMS_NEAR_ZERO)
		mt.joint_names, mt.position = TORSO_NAMES, list(TORSO_NEAR_ZERO)
		time.sleep(0.3)      # 切换服务返回 ≠ 订阅匹配完成
		pub_a.publish(ma); pub_t.publish(mt)
		time.sleep(0.3)
		pub_a.publish(ma); pub_t.publish(mt)   # 双发防发现窗首丢
		self.get_logger().info('已回关节近零位目标 (双 JS 实例, ruckig 平滑收敛)')

	# ---- 主流程 ----

	def run(self, lift=0.10, fwd=0.10):
		# 1. 起跑前置: 非零位告警 (偏移几何假设零位附近快照)
		jq = {}
		sub = self.create_subscription(JointState, '/joint_states',
			lambda m: jq.update(zip(m.name, m.position)), 10)
		t0 = time.time()
		while not jq and time.time() - t0 < 2.0:
			rclpy.spin_once(self, timeout_sec=0.1)
		self.destroy_subscription(sub)
		if jq and max(abs(v) for n, v in jq.items() if 'arm' in n or 'torso' in n) > 0.15:
			self.get_logger().warn('非零位起跑: 目标可能落边界外, 建议先跑关节 demo 回零')

		# 2. '@js' 通配切换 CM (四件式: 全部 JS 实例让出接口; 右臂冻结入档)
		self.ensure_controllers(activate=['cartesian_motion_controller'], deactivate=['@js'])
		pub = self.create_publisher(PoseStamped, CM_CMD,
			QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
			           history=HistoryPolicy.KEEP_LAST, depth=1))
		self.create_subscription(CartesianMotionStatus, CM_STATUS, self._on_status, 10)
		self.get_logger().info('CM 接管左臂 (@js 已全停, 右臂冻结在最后命令位)')

		# 3. TF 快照 = 目标派生基准
		tfbuf = Buffer()
		TransformListener(tfbuf, self)
		t0 = time.time()
		while time.time() - t0 < 5.0 and \
			not tfbuf.can_transform(BASE, TIP, rclpy.time.Time()):
			rclpy.spin_once(self, timeout_sec=0.1)
		snapshot = tfbuf.lookup_transform(BASE, TIP, rclpy.time.Time())
		p = snapshot.transform.translation
		self.get_logger().info('左臂末端快照: (%.3f, %.3f, %.3f) m' % (p.x, p.y, p.z))

		try:
			# 4. 三腿: 抬升 → 前伸 → 收回 (拒绝保持原位, 演示继续)
			self.goto(pub, snapshot, (0.0, 0.0, lift), '抬升 %+.0fcm' % (lift * 100))
			self.goto(pub, snapshot, (fwd, 0.0, lift), '前伸 %+.0fcm' % (fwd * 100))
			self.goto(pub, snapshot, (0.0, 0.0, 0.0), '收回原位')
		finally:
			# 5. 退出礼让: '@js' 全部回来 + 回近零位 (生命周期纪律)
			self.ensure_controllers(activate=['@js'], deactivate=['cartesian_motion_controller'])
			self.send_zero()

	def _on_status(self, msg):
		self.status.update(err=msg.position_error, arrival=msg.arrival,
		                   res=msg.last_result, sh=msg.solver_health,
		                   tgt=(msg.target_pose.position.x,
		                        msg.target_pose.position.y,
		                        msg.target_pose.position.z))
		self.status_t = time.time()


def main():
	lift = 0.10
	fwd = 0.10
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--lift':
			lift = float(argv.pop(0))
		elif a == '--fwd':
			fwd = float(argv.pop(0))
		else:
			raise SystemExit("未知参数 '%s' (支持: --lift <m> --fwd <m>)" % a)
	rclpy.init(args=None)
	node = R1ProDemoCartesian()
	try:
		node.run(lift, fwd)
	finally:
		node.destroy_node()
		try:
			rclpy.shutdown()
		except Exception:
			pass   # Ctrl-C 时 context 已关 (守护, 不掩盖主流程)


if __name__ == '__main__':
	main()

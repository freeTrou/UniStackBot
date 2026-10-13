#!/usr/bin/env python3
"""r1_7a 笛卡尔控制 demo (机型专属; scripts/<robot>_arm/ 约定, 2026-10-12)。

**定位 = 控制开发人员的接入参考实现** (链路契约的完整正确消费样板)。
完整工况: 切 CM → 部署 → 两圈 36 点连续圆周流 (快/慢) → 收回 → 切回 JS → 回关节零位。
教科书点: CM 契约 QoS / 目标对账锚点 / 三结局判据 / IK 错误语义 (last_result
=IkResult + solver_health/arrival 新正字段, RESULT_NAME 人话映射) / 生命周期纪律 (收尾回零位)。
最小入门请看 scripts/minimal_cartesian.py (外置通用教学件)。

前置: 任一链已启动, joint_stream_controller active。
用法: ros2 run unistackbot_demo r1_7a_demo_cartesian.py
      [--lift 0.15] [--fwd 0.15] [--span 0.08] [--stream-hz 50]
"""

import math
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

BASE, TIP = 'base_link', 'Link7'   # r1_7a 常量 (机型专属 demo 不做发现)
CM_CMD = '/cartesian_motion_controller/command'
CM_STATUS = '/cartesian_motion_controller/status'
JS_CMD = '/joint_stream_controller/command'

# status.last_result 透传的 IkResult 枚举 (码表: unistackbot_interface/ik_result.hpp)
RESULT_NAME = {1: 'UNREACHABLE', 2: 'NEAR_SINGULAR', 3: 'ITERATION_LIMIT',
               4: 'LIMIT_CONFLICT', 5: 'NOT_READY', 6: 'UNSUPPORTED'}

# 收尾回零位目标 (2% 限位内缩; 真零位是折叠奇异邻域, 勿精确回 0)
NEAR_ZERO = [0.0, 0.08, 0.05, 0.08, 0.0, 0.0, 0.0]


class R17aDemoCartesian(Node):

	def __init__(self):
		super().__init__('r1_7a_demo_cartesian')
		self.status = {}       # 最近一帧 CM status 摘要
		self.status_t = 0.0    # 该帧到达时刻 (静默检测用)
		self.sent = None       # 对账锚点: 最近一次发布的目标位置

	# ---- 链路契约消费 (接入方照抄的三件套) ----

	def ensure_controllers(self, activate, deactivate, retries=7):
		"""进入声明 (幂等): 先查当前态, 只对需要变化的发 switch。
		activate 已 active 的控制器会被 CM 硬拒 (not inactive) —— 幂等由状态查询实现。"""
		import subprocess
		for attempt in range(retries):
			try:
				r = subprocess.run(['ros2', 'control', 'list_controllers'],
					capture_output=True, text=True, timeout=15)
			except subprocess.TimeoutExpired:
				raise RuntimeError('list_controllers 超时 —— 链未启动或 CLI 缺失?'
					' 先起链 (ros2 launch ...); CLI 缺失装 ros-humble-ros2controlcli')
			states = {}
			for line in r.stdout.splitlines():
				clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip())
				parts = clean.split()
				if len(parts) >= 3:
					states[parts[0]] = parts[-1]
			if not states:
							raise RuntimeError('list_controllers 无输出 —— 链未启动? 先 ros2 launch 再跑 demo')
			args = []
			for c in deactivate:
				if states.get(c) == 'active':
					args += ['--deactivate', c]
			for c in activate:
				if states.get(c) != 'active':
					args += ['--activate', c]
			if not args:
				return
			s = subprocess.run(['ros2', 'control', 'switch_controllers'] + args,
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
		  ('conv', err)     误差进容差 —— 只认发布之后且 target 对得上账的 status
		  ('reject', code)  reject_after 秒仍不收敛且结果码非零 = 求解器在拒绝
		  ('plateau', err)  误差钉住但进不了容差 (物理链跟踪极限, mujoco 正常)
		"""
		t0 = time.time()
		err_ref = None
		while time.time() - t0 < timeout:
			rclpy.spin_once(self, timeout_sec=0.05)
			if self.status_t and time.time() - self.status_t > 5.0:
				raise RuntimeError('CM status 静默 >5s —— 链已死或 CM 未激活')
			tgt = self.status.get('tgt')
			# 到位判定用 arrival 单枚举 (2026-10-13 新正字段; converged 已 deprecated):
			# 逐条下发场景取 SETTLED 档 ("到了且停住", 双条件防抖动穿越容差带假亮)
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

	def stream_path(self, pub, snapshot, path_fn, n=36, step_s=0.05, tag='流'):
		"""通用连续轨迹流: path_fn(t∈[0,1]) → (x,y,z) 相对快照偏移。n 段小步
		推进不等待 (形状由发布端插值保证), 流尾等一次收敛确认。"""
		t0 = time.time()
		for k in range(n):
			p0, p1 = path_fn(k / n), path_fn((k + 1) / n)
			for i in range(1, 6):
				a = i / 6.0
				a = a * a * (3.0 - 2.0 * a)   # smoothstep 段内插值
				self.publish_target(pub, snapshot,
					tuple(p0[j] + (p1[j] - p0[j]) * a for j in range(3)))
				rclpy.spin_once(self, timeout_sec=0)   # 长流必须持续 spin: 回调积压会饿死 status 判定
				time.sleep(max(0.0, step_s - 0.0))
		self.wait_result()
		self.get_logger().info('%s: %d 点流完 (%.1fs), err=%.3fmm' % (
			tag, n, time.time() - t0, self.status.get('err', 0.0) * 1000.0))

	def send_zero(self):
		"""收尾回关节零位: 切回 JS 后一条 CSP 目标, ruckig 平滑收敛 (断流语义)。"""
		pub = self.create_publisher(JointCommand, JS_CMD, 10)
		m = JointCommand()
		m.mode = JointCommand.MODE_CSP
		m.joint_names = ['J%d' % (i + 1) for i in range(7)]
		m.position = NEAR_ZERO
		time.sleep(0.3)      # 切换服务返回 ≠ 订阅匹配完成
		pub.publish(m)
		time.sleep(0.3)
		pub.publish(m)       # 双发防发现窗首丢
		self.get_logger().info('已回关节零位目标 (JS ruckig 平滑收敛)')

	# ---- 主流程 ----

	def run(self, lift=0.15, fwd=0.15, span=0.08):
		# 1. 起跑前置: 非零位告警 (偏移几何假设零位快照)
		jq = {}
		sub = self.create_subscription(JointState, '/joint_states',
			lambda m: jq.update(zip(m.name, m.position)), 10)
		t0 = time.time()
		while not jq and time.time() - t0 < 2.0:
			rclpy.spin_once(self, timeout_sec=0.1)
		self.destroy_subscription(sub)
		if jq and max(abs(v) for v in jq.values()) > 0.1:
			self.get_logger().warn('非零位起跑: 目标可能落边界外, 建议先回零再跑')

		# 2. TF 快照 = 所有目标偏移的参照原点
		tfbuf = Buffer()
		TransformListener(tfbuf, self)
		t0 = time.time()
		while time.time() - t0 < 5.0 and not tfbuf.can_transform(BASE, TIP, rclpy.time.Time()):
			rclpy.spin_once(self, timeout_sec=0.1)
		snapshot = tfbuf.lookup_transform(BASE, TIP, rclpy.time.Time())

		self.create_subscription(CartesianMotionStatus, CM_STATUS, self._on_status, 10)
		pub = self.create_publisher(PoseStamped, CM_CMD,
			QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
			           history=HistoryPolicy.KEEP_LAST, depth=1))

		self.ensure_controllers(activate=['cartesian_motion_controller'],
			deactivate=['joint_stream_controller'])
		self.get_logger().info('CM 接管 (%s → %s), 部署→圆周/横扫/竖扫/8字→收回' % (BASE, TIP))
		try:
			# 部署 (逐点确认)
			self.goto(pub, snapshot, (0.0, 0.0, lift), '抬升 z+%dcm' % round(lift * 100))
			self.goto(pub, snapshot, (fwd, 0.0, lift), '前伸部署 x+%dcm' % round(fwd * 100))

			# 主体: 四类轨迹形态各一 (以部署位为心, 邻域 ±15cm 球内可达)
			# ① 圆周 (闭合轨迹, 快慢两圈)
			self.stream_path(pub, snapshot,
				lambda t: (fwd + span * math.cos(2 * math.pi * t),
				           span * math.sin(2 * math.pi * t), lift),
				tag='①圆周·慢', step_s=0.06)
			self.stream_path(pub, snapshot,
				lambda t: (fwd + span * math.cos(2 * math.pi * t),
				           span * math.sin(2 * math.pi * t), lift),
				tag='①圆周·快', step_s=0.025)
			# ② 横扫 y (左右空间覆盖 —— 圆周只在 fwd 平面, 这里补 y 向)
			self.stream_path(pub, snapshot,
				lambda t: (fwd, -span * math.cos(math.pi * t), lift),   # +y → -y 一个来回
				n=24, tag='②横扫y', step_s=0.05)
			# ③ 竖扫 z (高度覆盖: 中段 +6cm, 起终点回落 lift 平面)
			self.stream_path(pub, snapshot,
				lambda t: (fwd, 0.0, lift + 0.06 * math.sin(math.pi * t)),
				n=24, tag='③竖扫z', step_s=0.05)
			# ④ 8 字 (Lissajous: x 一频 y 二频 —— 复合轨迹, 曲率连续变化)
			self.stream_path(pub, snapshot,
				lambda t: (fwd + span * math.sin(2 * math.pi * t),
				           span * 0.7 * math.sin(4 * math.pi * t), lift),
				n=48, tag='④8字', step_s=0.04)

			# 收回
			self.goto(pub, snapshot, (fwd, 0.0, lift + 0.05), '上浮')
			self.goto(pub, snapshot, (0.0, 0.0, lift), '收回中')
			self.goto(pub, snapshot, (0.0, 0.0, 0.02), '落回近起点')   # z+2cm 离奇异核心
		finally:
			try:
				self.ensure_controllers(activate=['joint_stream_controller'],
					deactivate=['cartesian_motion_controller'])
				self.send_zero()
			except Exception as e:
				self.get_logger().error('收尾失败 (%s) —— 手动: ros2 control switch_controllers '
					'--deactivate cartesian_motion_controller --activate joint_stream_controller'
					% str(e).strip()[-120:])

	def _on_status(self, msg):
		self.status.update(err=msg.position_error, arrival=msg.arrival,
		                   res=msg.last_result, sh=msg.solver_health,
		                   tgt=(msg.target_pose.position.x,
		                        msg.target_pose.position.y,
		                        msg.target_pose.position.z))
		self.status_t = time.time()


def main():
	rclpy.init()
	node = R17aDemoCartesian()
	try:
		node.run()
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()

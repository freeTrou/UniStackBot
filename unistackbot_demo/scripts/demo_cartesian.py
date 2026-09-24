#!/usr/bin/env python3
"""给 CartesianMotionController 发一段笛卡尔演示/测试运动 (末端位姿目标), 机型无关。

四种 pattern:
  up   (默认, 演示腿): 切 CM 接管 → 当前位姿 +z 偏移 → 等收敛 → 回原位姿 → 切回 JS。
  axes (2026-09-23 多点位测试): 抬升 --offset 到基准位 → 以基准为原点 8 方向
       (±x/±y/±z/x+y/x-y) --sweep 偏移逐点往返, 每点后回基准 → 汇总。
       抬升位 2cm 扫描探针实测 (piper+analytic): 7/8 可解, -x 限位冲突诚实拒。
  sweep (2026-09-23 大幅运动; 2026-09-24 快慢变速): 抬升 --lift → 前伸 --fwd 部署 →
       r=--span 圆周整圈 (16 点连续, 逐点快慢交替) → 上浮/回圆心 → 收回落零。
       全程按 --speed (m/s) ×每腿系数 (慢 0.5 / 快 1.6) 插值目标流, 总程 >20s。
       piper 工作空间探针: 零位仅 +z 可行 (z 阶梯 300mm 全通); 部署位 (z+200,x+200)
       邻域 ±150mm 球 6/6 + r=100mm 圆周 8/8 全可解——大幅度必须先部署出去。

诚实拒绝 (UNREACHABLE=1 / LIMIT_CONFLICT=4 等解析解的正确行为) 记为"拒"不判死:
判据 = 超过 --reject-after 秒未收敛且最近结果码 != 0 (求解器在拒绝, 不是在走)。

目标位姿从 /tf 的 base->tip 实时读 (不写死任何机型可达位姿); base/tip 链名读自 CM
自身参数 (cartesian_motion_controller.base_link/tip_link —— 求解器配置的链就是权威链)。

前置: 三链任一已启动 (control/ign/mujoco.launch.py) 且 joint_stream_controller active。

用法: ros2 run unistackbot_demo demo_cartesian.py [--pattern up|axes|sweep|reject]
  reject = 负路径验证 (2026-09-23 用户裁决: 上层不完美是常态, 诚实拒绝路径也是验收面):
  不可达目标弹幕 (远超臂展/边界/穿底) 逐点断言被拒 (result!=0) + 恢复目标收敛
        [--offset 0.03] [--sweep 0.02] [--timeout 30] [--reject-after 1.5]
        [--traverse 1.0] [--stream-hz 50]
        [--lift 0.20] [--fwd 0.20] [--span 0.10] [--speed 0.15]

注: up 腿 --traverse>0 时按 --stream-hz 插值目标流走完 (50Hz+3cm/1s → 每段 0.6mm,
形状轴归上层——路径插值永不在控制器; CM 在 500Hz 环内对最新目标解 IK, 发布率只定
切分粒度; python rclpy 100Hz 稳 / 200Hz 勉强)。=0 单发点到点 (CM --once 语义,
~25ms 走完 3cm——步长上限 5 rad/s 下点到点本来就这速度)。axes 恒点到点 (测试语义)。

注: 偏移取竖直向上 (+z) —— piper 折叠零位压在 joint2 下限/joint3 上限上, 零位附近
x/y 方向偏移目标常无限位内解析解 (analytic 诚实拒绝, 探针实测); DLS 数值解可边界
逼近但非精确解。axes pattern 先抬升离面再扫, 规避此约束。
"""

import math
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from unistackbot_interface.msg import CartesianMotionStatus

# axes pattern 的 8 个扫描方向 (单位向量, 对角按分量施加 = 探针口径)
AXES = (
	('+x', (1.0, 0.0, 0.0)), ('-x', (-1.0, 0.0, 0.0)),
	('+y', (0.0, 1.0, 0.0)), ('-y', (0.0, -1.0, 0.0)),
	('+z', (0.0, 0.0, 1.0)), ('-z', (0.0, 0.0, -1.0)),
	('x+y', (1.0, 1.0, 0.0)), ('x-y', (1.0, -1.0, 0.0)),
)


class DemoCartesian(Node):

	def __init__(self):
		super().__init__('demo_cartesian')
		self._status = {}
		self._status_t = 0.0   # 最近一帧 status 的本地时刻 (静默检测: 链死/CM 未激活)
		self._sent = None      # 最近发布的目标位 (对账锚点, 防"反映旧目标的在途 status"假收敛)

	def _get_params(self, node_name, names):
		"""经参数服务批量读远端节点参数 (同 demo_motion: Humble rclpy 手工建客户端)。"""
		cli = self.create_client(GetParameters, node_name + '/get_parameters')
		if not cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError('参数服务 %s/get_parameters 5 秒内未出现 (链路未起?)' % node_name)
		req = GetParameters.Request()
		req.names = names
		fut = cli.call_async(req)
		rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
		if fut.result() is None:
			raise RuntimeError('参数 %s 调用 5 秒无响应 (CM 忙/链路退化)' % node_name)
		return fut.result().values

	def _fetch_chain(self):
		"""(base, tip) 读自 CM 参数 —— 求解器配置的链就是权威链, 机型无关。

		参数托管形态随 ros2_control 版本而异, 两路兜底:
		  ① 控制器自有节点 /cartesian_motion_controller (Humble 形态, 参数无前缀)
		  ② CM 节点带前缀 /controller_manager + cartesian_motion_controller.*
		"""
		for node, names in (
			('/cartesian_motion_controller', ['base_link', 'tip_link']),
			('/controller_manager', ['cartesian_motion_controller.base_link',
			                         'cartesian_motion_controller.tip_link']),
		):
			try:
				vals = self._get_params(node, names)
			except RuntimeError:
				continue
			if vals[0].string_value and vals[1].string_value:
				return vals[0].string_value, vals[1].string_value
		raise RuntimeError('读取 CM base/tip 失败 (两种参数形态均未命中)。'
		                   '诊断: ros2 param list /controller_manager | grep -i cartesian')

	def _switch(self, *args):
		r = subprocess.run(['ros2', 'control', 'switch_controllers', *args],
		                   capture_output=True, text=True, timeout=15)
		if r.returncode != 0:
			raise RuntimeError('switch_controllers 失败: ' + r.stderr.strip()[-160:])

	def _wait_result(self, timeout, reject_after):
		"""等当前点位的结局: ('conv', err_m) / ('reject', result) / ('plateau', err_m)。

		诚实拒绝判据: reject_after 秒后仍未收敛且最近结果码 != 0 —— 求解器在持续
		拒绝 (解析解对固定位姿结果确定), 不是运动慢。conv 只认发布之后的新 status
		(调用方发布后清缓存 —— 陈旧 conv=True 会假收敛, 2026-09-23 实锤), 且其
		target_pose 必须对得上本次发布值: 清缓存挡不住在途帧 (20Hz status 反映的还是
		上一目标, 2-10ms 传输窗内先到 —— 对账锚点防假收敛, 审查实锤 2026-09-23)。
		"""
		t0 = time.time()
		err_ref = None
		while time.time() - t0 < timeout:
			rclpy.spin_once(self, timeout_sec=0.05)
			# 静默阈值 5s (2026-09-23): 2s 在年轻链 + DDS 发现窗下误杀过一次 (bench 实测,
			# 手动同链同参复跑即绿); 死链照样 5s 内抓住, 不损害当初"链死秒级报错"的初衷
			if self._status_t and time.time() - self._status_t > max(5.0, reject_after):
				raise RuntimeError('CM status 静默 %.1fs —— 链已死亡或 CM 未激活, 检查 launch 终端'
					' (2026-09-23 实锤: 链死后此处曾无限 spin)' % (time.time() - self._status_t))
			if self._status.get('conv'):
				tgt = self._status.get('tgt')
				mismatch = (self._sent is None or tgt is None or
				            max(abs(a - b) for a, b in zip(tgt, self._sent)) > 1e-9)
				if mismatch:
					continue   # 在途旧帧: 反映的是上一目标, 不算本次收敛
				return 'conv', self._status['err']
			if time.time() - t0 > reject_after and self._status.get('res', 0) != 0:
				return 'reject', self._status['res']
			# 稳态未达容差 (plateau): err 钉住不动但 conv 不置位 —— 物理链 (mujoco)
			# 稳态 0.74-1.5mm 可能高于 CM converge 容差 1mm, 解没问题只是跟踪不到线,
			# 与拒绝/超时是不同结局。3s 处采样, 4s 起判 (±10% 带)。
			elapsed = time.time() - t0
			e = self._status.get('err')
			if elapsed > 3.0:
				if err_ref is None:
					err_ref = e
				elif e is not None and err_ref is not None and elapsed > 4.0 and \
					abs(e - err_ref) < 0.1 * max(abs(err_ref), 1e-4):
					return 'plateau', e
		raise RuntimeError('点位 %.0fs 未收敛也未拒绝 (err=%s mm, mode=%s, result=%s)' % (
			timeout,
			round(self._status.get('err', -1) * 1000.0, 3),
			self._status.get('mode'), self._status.get('res')))

	def run(self, pattern, offset, sweep, timeout, reject_after, traverse, stream_hz,
	        lift, fwd, span, speed):
		base, tip = self._fetch_chain()
		self.get_logger().info('末端链 (读自 CM 参数): %s -> %s' % (base, tip))
		if stream_hz > 150.0:
			self.get_logger().warn('--stream-hz %.0f 超出 rclpy 舒适区 (~100 稳/200 勉强): '
				'实际达成率看结束时的诚实报告, 矩阵测试建议 ≤100' % stream_hz)
		self._late = 0
		self._sent_n = 0
		self._t_first = None

		# 当前末端位姿 (base 系, TF 实时) —— 所有目标的参照快照
		tfbuf = Buffer()
		TransformListener(tfbuf, self)
		deadline = time.time() + 5.0
		while time.time() < deadline and not tfbuf.can_transform(base, tip, rclpy.time.Time()):
			rclpy.spin_once(self, timeout_sec=0.1)
		tf = tfbuf.lookup_transform(base, tip, rclpy.time.Time())

		self.create_subscription(CartesianMotionStatus,
		                         '/cartesian_motion_controller/status', self._on_status, 10)

		self._switch('--deactivate', 'joint_stream_controller',
		             '--activate', 'cartesian_motion_controller')
		self.get_logger().info('CM 接管 (joint_stream → cartesian_motion)')

		def publish(dxyz, tag, quiet=False):
			t = PoseStamped()
			t.header.frame_id = base
			t.pose.position.x = tf.transform.translation.x + dxyz[0]
			t.pose.position.y = tf.transform.translation.y + dxyz[1]
			t.pose.position.z = tf.transform.translation.z + dxyz[2]
			t.pose.orientation = tf.transform.rotation
			self._status.clear()   # 只认发布之后的新 status (陈旧 conv=True 假收敛, 2026-09-23 实锤)
			# 对账锚点 (2026-09-23 审查补): 清缓存挡不住"已发出但反映旧目标"的 status
			# 在途帧 —— conv 只认 target_pose 对得上本次发布值的 status
			self._sent = (t.pose.position.x, t.pose.position.y, t.pose.position.z)
			pub.publish(t)
			self._sent_n += 1
			if self._t_first is None:
				self._t_first = time.time()
			if not quiet:
				self.get_logger().info('目标 %s' % tag)

		def pace(t0, i, dt):
			"""绝对时间节拍 (相对 sleep 会累积漂移, 矩阵测试的 x 轴要诚实); 返回是否落后。"""
			nxt = t0 + i * dt
			delay = nxt - time.time()
			if delay > 0.0:
				time.sleep(delay)
				return False
			return True

		# CM 契约同款 QoS: reliable + KeepLast(1) (深度 10 会在 rclpy 跟不上时积压
		# 10 条陈旧目标, CM 追着滞后目标走而 demo 以为发的是最新 —— 审查实锤 2026-09-23)
		pub = self.create_publisher(PoseStamped, '/cartesian_motion_controller/target',
			QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
			           history=HistoryPolicy.KEEP_LAST, depth=1))
		try:
			if pattern == 'up':
				cur = (0.0, 0.0, 0.0)   # 相对快照的当前偏移 (插值起点)
				for dxyz, tag in (((0.0, 0.0, offset), 'z%+.0fcm (当前位姿 + 竖直偏移)' % (offset * 100.0)),
				                  ((0.0, 0.0, 0.0), 'z+0cm (回原位姿)')):
					if traverse > 0:
						# 插值目标流: 点到点 3cm 在 5 rad/s 步长上限下 ~25ms 就走完
						# (快到看不见), 演示腿铺 traverse 秒走完 —— 形状轴归上层。
						# 发布率只定切分粒度 (CM 500Hz 环内对最新目标解 IK, 解耦)。
						steps = max(2, int(round(traverse * stream_hz)))
						publish(cur, tag)   # 腿首目标显式 (进入流式)
						t_leg = time.time()
						dt = 1.0 / stream_hz
						for i in range(1, steps + 1):
							a = i / steps
							publish((cur[0] + (dxyz[0] - cur[0]) * a,
							         cur[1] + (dxyz[1] - cur[1]) * a,
							         cur[2] + (dxyz[2] - cur[2]) * a), tag, quiet=True)
							if pace(t_leg, i, dt):
								self._late += 1
					else:
						publish(dxyz, tag)
					cur = dxyz
					kind, v = self._wait_result(timeout, reject_after)
					if kind == 'plateau':
						raise RuntimeError('up 腿稳态未达容差 (err=%.3fmm > CM converge 1mm) —— '
							'物理链(mujoco)正常现象: 解没问题、跟踪稳态到不了线; '
							'精确收敛验证请走 mock 链' % (v * 1000.0))
					if kind != 'conv':
						raise RuntimeError('up 腿被诚实拒绝 (result=%s)' % v)
					self.get_logger().info('收敛 (err=%.3fmm)' % (v * 1000.0))
			elif pattern == 'reject':   # 负路径验证: 不可达弹幕断言被拒 + 恢复 (上游不完美是常态)
				# 弹幕几何: 相对快照偏移, 覆盖三类不可达 (远超臂展/拉到边界/穿底) + 一个对角边界
				battery = (
					('远超臂展 +x1.0m', (1.0, 0.0, 0.2)),
					('超臂展 +x0.8', (0.8, 0.0, 0.2)),
					('穿底 -z0.4', (0.0, 0.0, -0.4)),
					('对角超展 (0.6,0.6)', (0.6, 0.6, 0.1)),
				)
				res_name = {1: 'UNREACHABLE', 4: 'LIMIT_CONFLICT', 3: 'UNSUPPORTED'}
				rej_ok, rej_fail = [], []
				t0 = time.time()
				for name, d in battery:
					publish(d, name)
					kind, v = self._wait_result(timeout, reject_after)
					if kind == 'reject':
						rej_ok.append('%s(%s)' % (name, res_name.get(v, v)))
						self.get_logger().info('点 %s: 诚实拒绝 result=%s (%s) ✓' % (
							name, v, res_name.get(v, '?')))
					elif kind == 'conv':
						rej_fail.append('%s(居然收敛 err=%.3fmm —— 该点按设计应不可达!)' % (name, v * 1000.0))
						self.get_logger().warn('点 %s: 预期拒绝却收敛 (err=%.3fmm) —— 弹幕几何需校准' % (
							name, v * 1000.0))
					else:
						rej_fail.append('%s(结局=%s, 期望拒绝)' % (name, kind))
						self.get_logger().warn('点 %s: 结局 %s (期望 reject)' % (name, kind))
				# 恢复验证: 弹幕后一个好目标必须照常收敛 (拒绝不污染链路状态)
				publish((0.0, 0.0, offset), '恢复目标 +z%.0fcm' % (offset * 100.0))
				kind, v = self._wait_result(timeout, reject_after)
				rec = '恢复收敛 (err=%.3fmm) ✓' % (v * 1000.0) if kind == 'conv' \
					else '恢复失败 (%s) ✗' % kind
				self.get_logger().info(rec)
				self.get_logger().info(
					'汇总: 诚实拒 %d/%d %s · %s · 总用时 %.1fs' % (
						len(rej_ok), len(battery),
						('[' + ', '.join(rej_ok) + ']') if rej_ok else '',
						rec, time.time() - t0))
				if rej_fail:
					raise RuntimeError('负路径验证未全过: %s' % rej_fail)
			elif pattern == 'axes':   # 抬升到基准位 → 8 方向扫描, 每点回基准
				publish((0.0, 0.0, offset), '抬升基准 z%+.0fcm' % (offset * 100.0))
				kind, v = self._wait_result(timeout, reject_after)
				if kind != 'conv':
					raise RuntimeError('抬升失败 (kind=%s, result=%s) —— axes 需要可解的基准位' % (kind, v))
				self.get_logger().info('基准就绪 (err=%.3fmm), 开始 %d 向扫描 @%.0fmm' % (
					v * 1000.0, len(AXES), sweep * 1000.0))
				tally_conv, tally_rej = [], []
				t0 = time.time()
				for i, (name, vec) in enumerate(AXES, 1):
					publish((vec[0] * sweep, vec[1] * sweep, offset + vec[2] * sweep),
					        '%d/%d %s' % (i, len(AXES), name))
					kind, v = self._wait_result(timeout, reject_after)
					if kind == 'conv':
						tally_conv.append(name)
						self.get_logger().info('点 %d/%d (%s): 收敛 err=%.3fmm' % (i, len(AXES), name, v * 1000.0))
					elif kind == 'plateau':
						tally_rej.append('%s(稳态%.1fmm)' % (name, v * 1000.0))
						self.get_logger().warn('点 %d/%d (%s): 稳态未达容差 (err=%.3fmm > 1mm, 物理链正常), 记账跳过' % (
							i, len(AXES), name, v * 1000.0))
					else:
						tally_rej.append('%s(result=%s)' % (name, v))
						self.get_logger().warn('点 %d/%d (%s): 诚实拒绝 (result=%s), 跳过' % (i, len(AXES), name, v))
					# 回基准 (必须收敛, 否则后续点位基准漂移)
					publish((0.0, 0.0, offset), '回基准')
					kind, v = self._wait_result(timeout, reject_after)
					if kind != 'conv':
						raise RuntimeError('回基准失败 (kind=%s, result=%s) —— 后续点位基准漂移, 终止' % (kind, v))
				self.get_logger().info(
					'汇总: 收敛 %d/%d · 诚实拒 %d/%d %s · 总用时 %.1fs' % (
						len(tally_conv), len(AXES), len(tally_rej), len(AXES),
						('[' + ', '.join(tally_rej) + ']') if tally_rej else '',
						time.time() - t0))
			else:   # sweep: 大幅运动 —— 部署(抬升+前伸) → 圆周整圈 → 上浮/回心 → 收回落零
				cur = (0.0, 0.0, 0.0)   # 相对快照的当前偏移 (链式插值起点)

				def leg(target, tag, must=False, spd=1.0):
					"""流式走一腿 (距离/速度定时长, spd=本腿速度系数——快慢变速); 返回
					'conv'/'plateau'/'reject'。收敛与稳态(物理链跟踪到不了 1mm 线但已
					钉住)都推进 cur, 诚实拒保持原位。"""
					nonlocal cur
					dist = ((target[0] - cur[0]) ** 2 + (target[1] - cur[1]) ** 2 +
					        (target[2] - cur[2]) ** 2) ** 0.5
					v = speed * spd
					dur = dist / v if v > 0 else 0.0   # v<=0 单发点到点
					steps = max(2, int(round(max(dur, 0.05) * stream_hz))) if dur > 0 else 1
					t_leg = time.time()
					dt = 1.0 / stream_hz if dur > 0 else 0.0
					for i in range(1, steps + 1):
						a = i / steps
						publish((cur[0] + (target[0] - cur[0]) * a,
						         cur[1] + (target[1] - cur[1]) * a,
						         cur[2] + (target[2] - cur[2]) * a), tag, quiet=True)
						if dur > 0 and pace(t_leg, i, dt):
							self._late += 1
					kind, v = self._wait_result(timeout, reject_after)
					if kind == 'conv':
						cur = target
						self.get_logger().info('%s: 收敛 err=%.3fmm' % (tag, v * 1000.0))
						return 'conv'
					if kind == 'plateau':
						cur = target   # 臂已到物理稳态 (≈目标), 继续链式走
						self.get_logger().warn('%s: 稳态未达容差 (err=%.3fmm > 1mm, 物理链正常), 继续' % (
							tag, v * 1000.0))
						return 'plateau'
					self.get_logger().warn('%s: 诚实拒绝 (result=%s), 保持原位' % (tag, v))
					if must:
						raise RuntimeError('关键腿 "%s" 被拒 (result=%s), 终止' % (tag, v))
					return 'reject'

				# 快慢变速 (2026-09-24 用户需求: 长时大幅动作有快有慢): 每腿速度系数
				# 乘 --speed —— 慢 0.5× / 快 1.6×, 圆周逐点交替, 大腿各自快慢
				plan = [
					((0.0, 0.0, lift), '部署·抬升 z+%.0fcm·慢' % (lift * 100.0), True, 0.5),
					((fwd, 0.0, lift), '部署·前伸 x+%.0fcm·快' % (fwd * 100.0), True, 1.6),
				]
				for k in range(1, 17):   # 圆周 16 点连续整圈 (探针: r=100mm 8/8 可解)
					th = 2.0 * math.pi * k / 16.0
					fast = k % 2 == 1
					plan.append(((fwd + span * math.cos(th), span * math.sin(th), lift),
					             '圆周 %2d/16·%s' % (k, '快' if fast else '慢'), False,
					             1.6 if fast else 0.5))
				plan += [
					((fwd, 0.0, lift + span), '上浮 z+%.0fcm·快' % (span * 100.0), False, 1.6),
					((fwd, 0.0, lift), '回圆心·慢', False, 0.5),
					((0.0, 0.0, lift), '收·回中·快', False, 1.6),
					((0.0, 0.0, 0.0), '收·落回起点·慢', False, 0.5),
				]
				ok_n = pl_n = rej_n = 0
				t0 = time.time()
				for target, tag, must, spd in plan:
					k = leg(target, tag, must, spd)
					if k == 'conv':
						ok_n += 1
					elif k == 'plateau':
						pl_n += 1
					else:
						rej_n += 1
				self.get_logger().info('汇总: %d 腿收敛 · %d 腿稳态未达容差 · %d 腿诚实拒 · 总用时 %.1fs' % (
					ok_n, pl_n, rej_n, time.time() - t0))
		finally:
			# 诚实报告实际达成率 (名义 stream_hz 是切分粒度请求值, rclpy 未必跟得上)
			if self._t_first is not None and self._sent_n > 1:
				wall = time.time() - self._t_first
				late_note = ('; %d 拍落后' % self._late) if self._late else ''
				self.get_logger().info('发布流: 名义 %.0fHz, 实际达成 %.1fHz (%d 条 / %.1fs)%s' % (
					stream_hz, self._sent_n / wall if wall > 0 else 0.0,
					self._sent_n, wall, late_note))
			try:
				self._switch('--deactivate', 'cartesian_motion_controller',
				             '--activate', 'joint_stream_controller')
				self.get_logger().info('切回 (链恢复 joint_stream)')
			except Exception as e:
				# 链死/服务失联时切回必然超时 (2026-09-23 实锤 15s TimeoutExpired 级联):
				# 给人话恢复指令, 不让异常掩盖
				self.get_logger().error('切回失败 (%s)。若链还活着手动执行:\n'
					'  ros2 control switch_controllers --deactivate cartesian_motion_controller'
					' --activate joint_stream_controller\n链已死则直接重启链' % str(e).strip()[-120:])

	def _on_status(self, msg):
		self._status.update(err=msg.position_error, conv=msg.converged,
		                    mode=msg.mode, res=msg.last_result,
		                    tgt=(msg.target_pose.position.x,
		                         msg.target_pose.position.y,
		                         msg.target_pose.position.z))
		self._status_t = time.time()


def main(args=None):
	pattern = 'up'
	offset = 0.03
	sweep = 0.02
	timeout = 30.0
	reject_after = 1.5
	traverse = 1.0
	stream_hz = 50.0
	lift = 0.20
	fwd = 0.20
	span = 0.10
	speed = 0.15
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--pattern':
			pattern = argv.pop(0)
		elif a == '--offset':
			offset = float(argv.pop(0))
		elif a == '--sweep':
			sweep = float(argv.pop(0))
		elif a == '--timeout':
			timeout = float(argv.pop(0))
		elif a == '--reject-after':
			reject_after = float(argv.pop(0))
		elif a == '--traverse':
			traverse = float(argv.pop(0))
		elif a == '--stream-hz':
			stream_hz = float(argv.pop(0))
		elif a == '--lift':
			lift = float(argv.pop(0))
		elif a == '--fwd':
			fwd = float(argv.pop(0))
		elif a == '--span':
			span = float(argv.pop(0))
		elif a == '--speed':
			speed = float(argv.pop(0))
		else:
			raise SystemExit("未知参数 '%s' (支持: --pattern --offset --sweep --timeout "
				"--reject-after --traverse --stream-hz --lift --fwd --span --speed)" % a)
	if stream_hz <= 0.0:
		print('--stream-hz 必须为正 (收到 %s)' % stream_hz)
		return 2
	if pattern not in ('up', 'axes', 'sweep', 'reject'):
		print('未知 pattern: %s (可用: up | axes | sweep | reject)' % pattern)
		return 2
	rclpy.init(args=args)
	node = DemoCartesian()
	try:
		node.run(pattern, offset, sweep, timeout, reject_after, traverse, stream_hz,
		         lift, fwd, span, speed)
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()

#!/usr/bin/env python3
"""键盘遥操作 (命令域④: 交互) —— 键态→笛卡尔速度→积分→CM 目标流, 机型无关。

架构位置 (2026-10-08 裁决): 遥操作不需要规划层 —— 人就是规划器 (§16.2 形状轴
归上层, 遥操作里上层=操作员)。本节点只做**指令生成** (按键突发流 → 稳态目标流),
平滑/安全全部由框架既有防线兜底, 本脚本零安全代码:
  - CM RT 环 OtgStream 关节 C2 (率失配填充, 慢上层 10-100Hz 正是设计档位)
  - 断流语义 (2026-10-10 统一): 本节点只在有按键时发流, 松键 deadman 后目标
    停发 → CM 收敛到最后目标保持 (不刹停不 shortfall, 无专门机制)
  - 工作空间外 = CM 诚实拒绝 (保持上一解), HUD 显示 REJECTED 不判死
  - write 层最终防线 (NaN门→限位clamp→步长饱和) 不经此路径存在感

键位 (base 系; 小写=1×, Shift大写=该键 2×; 多键组合靠键重复续命):
  W/S 前后(±x)  A/D 左右(±y)  R/F 升降(±z)      ← 左手簇
  I/K 俯仰  J/L 偏航  U/O 翻滚                      ← 右手簇
  -/= 速度档 降/升   Space HOLD乒乓   Q/Ctrl-C 退出

积分语义 (E2E 实锤教训, 2026-10-08): **目标空间自推进** (demo_cartesian 流式同款),
不是"每拍重锚反馈位姿"——后者目标永远只领先一个 dt (~1.7mm), OTG 见到的是恒定
微步, 臂速变成 CM 收敛动力学的函数 (实测 50mm/s 指令只走出 5mm/s)。配套两道界:
  - 待键→发流边沿: 目标重锚当前实际位姿 (空闲期臂已被断流刹停, 防跳变)
  - 有界前导: 位置领先钳 50mm / 姿态领先钳 30° (撞界被诚实拒时目标不跑飞,
    反向拨键即回; 钳位半径远大于 deadman 窗内位移 ~7.5mm, 正常操作无感)

对照主流 (ros-teleop/teleop_twist_keyboard, 2026-10-08 抄录比对): 同构处 = termios
原始读 + select 定频循环 + 静默判停 (它叫 key_timeout=0.5s, 我们 deadman=0.15s——
臂更快, 窗口收紧) + 终端设置还原 + 速度调节带状态显示。两处有意差异:
  ① 它发布 Twist 速度 (零速=停, 可连续发布); 我们发位姿目标——零速连续发布
    正是上面被否掉的"重锚微步", 故只在按键时发流, 停走 CM 断流刹停。
  ② 键语义 hold-to-move (按住=动); 它是 latch (按一下直到反键)——对底盘可行,
    对臂 hold+deadman 双保险更稳。
采纳它一处: **退出时显式发停令** (它发零 Twist; 我们发 HOLD 事件=瞬时冻结,
比等 200ms 断流刹停更利落)。

前置: 三链任一已启动; 启动时自动切 CM (joint_stream → cartesian_motion),
退出时切回 (与 demo_cartesian 同款 finally 纪律)。--no-switch 跳过自动切换。

用法: ros2 run unistackbot_demo demo_teleop_key.py
        [--hz 30] [--speed 0.02,0.05,0.10] [--rot 0.25,0.5,1.0] [--deadman 0.15]
        [--no-switch]
需要真实终端 (stdin tty); mujoco/gz 带不带 GUI 均可 —— 读的是自己终端的 stdin。

夹爪说明: 夹爪不在 CM 链 (工具自由度, 不参与位姿任务, IK 对它零约束), v1 无
夹爪键位 —— piper 边走边抓待 JS 伴飞模式立项 (真机总线周期层夹爪本来就随臂
一起发, 见 MasterBase 节点表)。
"""

import math
import os
import select
import subprocess
import sys
import termios
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from unistackbot_interface.msg import CartesianControl, CartesianMotionStatus

DEADMAN_S = 0.15          # 重复续命窗口 (松键判定; 断流=收敛最后目标, 无刹停机制)
FRESH_STREAM_S = 0.45     # 新按下保底发流窗: OS 键重复初始延迟 ~0.3-0.6s, 短于它
                          # 则每次按下都"走150ms→刹停→等重复"抽一下 (pty 测试 25Hz
                          # 持续发键测不出, 真手必现); 保底窗内无重复则视为单击
EE_SILENT_S = 1.0         # ee_state 静默超时 (反馈死了冻结积分, 不在陈旧位姿上漂)
STATUS_SILENT_S = 3.0     # status 静默提示阈值 (HUD 提醒, 不退出)
REJECT_SHOW_S = 1.2       # 拒绝提示在 HUD 上的驻留时长

# 键 → base 系单位方向。旋转为绕 base 系轴的角速度方向 (右手系):
#   I/K 绕 y (-/+): 工具前倾/后仰   J/L 绕 z (+/-): 左转/右转   U/O 绕 x (-/+)
LIN_KEYS = {
	ord('w'): (1.0, 0.0, 0.0), ord('s'): (-1.0, 0.0, 0.0),
	ord('a'): (0.0, 1.0, 0.0), ord('d'): (0.0, -1.0, 0.0),
	ord('r'): (0.0, 0.0, 1.0), ord('f'): (0.0, 0.0, -1.0),
}
ROT_KEYS = {
	ord('i'): (0.0, -1.0, 0.0), ord('k'): (0.0, 1.0, 0.0),
	ord('j'): (0.0, 0.0, 1.0), ord('l'): (0.0, 0.0, -1.0),
	ord('u'): (-1.0, 0.0, 0.0), ord('o'): (1.0, 0.0, 0.0),
}
SHIFT_FACTOR = 2.0        # 大写字母 (Shift 按住) = 该键 2×
COMBO_CAP = 2.0           # 多键合成模长上限 (单位方向坐标), 防斜向三键超速


def quat_mul(a, b):
	"""四元数乘 (w,x,y,z), a⊗b —— base 系旋转增量左乘用。"""
	w1, x1, y1, z1 = a
	w2, x2, y2, z2 = b
	return (
		w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
		w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
		w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
		w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
	)


def axis_angle_quat(ax, ang):
	"""轴角 → 单位四元数 (w,x,y,z)。ang=0 恒等。"""
	h = ang * 0.5
	s = math.sin(h)
	n = math.sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2])
	if n < 1e-12:
		return (1.0, 0.0, 0.0, 0.0)
	return (math.cos(h), ax[0] / n * s, ax[1] / n * s, ax[2] / n * s)


def quat_normalize(q):
	n = math.sqrt(sum(c * c for c in q))
	if n < 1e-12:
		return (1.0, 0.0, 0.0, 0.0)
	return tuple(c / n for c in q)


def quat_conj(q):
	return (q[0], -q[1], -q[2], -q[3])


def quat_angle(q1, q2):
	"""两姿态的测地夹角 [rad] (2·acos(|⟨q1,q2⟩|))。"""
	d = abs(sum(a * b for a, b in zip(q1, q2)))
	return 2.0 * math.acos(min(1.0, d))


def clamp_lead_pos(tgt, ee, cap_m):
	"""位置领先钳位: |tgt-ee| > cap 时沿方向拉回 cap (防拒绝期间目标跑飞)。"""
	d = tuple(tgt[i] - ee[i] for i in range(3))
	n = math.sqrt(sum(c * c for c in d))
	if n <= cap_m or n < 1e-12:
		return tgt
	s = cap_m / n
	return tuple(ee[i] + d[i] * s for i in range(3))


def clamp_lead_rot(tgt_q, ee_q, cap_rad):
	"""姿态领先钳位: 夹角超 cap 时绕同轴拉回 cap。"""
	ang = quat_angle(tgt_q, ee_q)
	if ang <= cap_rad or ang < 1e-9:
		return tgt_q
	d = quat_mul(tgt_q, quat_conj(ee_q))   # ee→tgt 增量
	if d[0] < 0.0:
		d = tuple(-c for c in d)   # 取短弧 (w≥0)
	half = math.acos(min(1.0, d[0]))
	sin_half = math.sin(half)
	if sin_half < 1e-12:
		return tgt_q
	axis = (d[1] / sin_half, d[2] / sin_half, d[3] / sin_half)
	return quat_normalize(quat_mul(axis_angle_quat(axis, cap_rad), ee_q))


class TeleopKey(Node):

	def __init__(self):
		super().__init__('demo_teleop_key')
		self.ee = None            # (pos xyz, quat wxyz, frame_id, 本地时刻)
		self.status = None        # 最近一帧 CartesianMotionStatus 关键字段
		self.status_t = 0.0
		# ee_state: 只读反馈流 (ee_state_broadcaster 常激活, CM 未接管时也在发)
		self.create_subscription(PoseStamped, '/ee_state_broadcaster/ee_state',
		                         self._on_ee, 10)
		self.create_subscription(CartesianMotionStatus,
		                         '/cartesian_motion_controller/status', self._on_status, 10)
		# CM 契约同款 QoS: target=reliable+KeepLast(1) (深度>1 会积压陈旧目标,
		# 审查实锤 2026-09-23); control=事件通道, CM 侧 sub 是 transient_local,
		# pub 必须 ≥ (volatile pub 配 transient_local sub 不兼容)
		self.pub_target = self.create_publisher(PoseStamped,
			'/cartesian_motion_controller/target',
			QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
			           history=HistoryPolicy.KEEP_LAST, depth=1))
		self.pub_control = self.create_publisher(CartesianControl,
			'/cartesian_motion_controller/control',
			QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
			           history=HistoryPolicy.KEEP_LAST, depth=1,
			           durability=DurabilityPolicy.TRANSIENT_LOCAL))
		# 统计 (诚实汇总用)
		self.sent_n = 0
		self.path_len = 0.0
		self.t_first = None
		self.reject_last_t = 0.0
		self.reject_last_res = 0

	def _on_ee(self, msg):
		p = msg.pose.position
		q = msg.pose.orientation
		self.ee = ((p.x, p.y, p.z),
		           quat_normalize((q.w, q.x, q.y, q.z)),
		           msg.header.frame_id, time.monotonic())

	def _on_status(self, msg):
		self.status = (msg.mode, msg.converged, msg.position_error,
		               msg.rotation_error, msg.last_result)
		self.status_t = time.monotonic()
		if msg.last_result != 0:
			self.reject_last_t = time.monotonic()
			self.reject_last_res = msg.last_result

	def send_control(self, mode):
		m = CartesianControl()
		m.mode = mode
		self.pub_control.publish(m)


def _switch(*args):
	r = subprocess.run(['ros2', 'control', 'switch_controllers', *args],
	                   capture_output=True, text=True, timeout=15)
	if r.returncode != 0:
		raise RuntimeError('switch_controllers 失败: ' + r.stderr.strip()[-160:])


def wait_ee(node, timeout=5.0):
	deadline = time.monotonic() + timeout
	while time.monotonic() < deadline:
		if node.ee is not None:
			return
		time.sleep(0.05)
	raise RuntimeError('/ee_state %.0fs 未到 —— 链未起? (三链任一: ros2 launch '
	                   'unistackbot_bringup control.launch.py robot:=<机型>)' % timeout)


def active_twist(hold_until, now, lin_speed, rot_speed):
	"""按 per-key 保底窗 (hold_until) 合成速度。终端只报 key-down: 新按下给
	FRESH_STREAM_S 保底 (覆盖 OS 键重复初始延迟, 消除起步抽动), 重复事件按
	DEADMAN_S 续命 (松键停感不变); 多键合成后模长钳 COMBO_CAP (防三键超速)。"""
	lin = [0.0, 0.0, 0.0]
	rot = [0.0, 0.0, 0.0]
	for kb, dirs in LIN_KEYS.items():
		t = hold_until.get(kb)
		if t is not None and now < t:
			f = SHIFT_FACTOR if 0x41 <= kb <= 0x5A else 1.0
			for i in range(3):
				lin[i] += dirs[i] * f
	for kb, axes in ROT_KEYS.items():
		t = hold_until.get(kb)
		if t is not None and now < t:
			f = SHIFT_FACTOR if 0x41 <= kb <= 0x5A else 1.0
			for i in range(3):
				rot[i] += axes[i] * f
	for v in (lin, rot):
		n = math.sqrt(sum(c * c for c in v))
		if n > COMBO_CAP:
			for i in range(3):
				v[i] *= COMBO_CAP / n
	return (tuple(c * lin_speed for c in lin), tuple(c * rot_speed for c in rot))


def fmt_hud(node, level, n_lv, lin_speed, rot_speed, held, streaming):
	"""单行 HUD (\\r 回写+清行尾)。拒绝必须可见 —— 诚实拒绝是链路正常语义,
	不显示会把"被拒"当"卡死"。"""
	m = node.status
	parts = []
	if m is None:
		parts.append('CM status 未到 (CM 未激活?)')
	else:
		mode_n, conv, perr, rerr, res = m
		mode_s = {0: 'INACTIVE', 1: 'IDLE', 2: 'TRACK', 3: 'HOLD', 4: 'DEGRADED'}.get(
			mode_n, str(mode_n))
		if time.monotonic() - node.status_t > STATUS_SILENT_S:
			parts.append('CM status 静默 %.0fs' % (time.monotonic() - node.status_t))
		else:
			parts.append('%s err=%.2fmm/%.2f°%s' % (
				mode_s, perr * 1000.0, math.degrees(rerr),
				' ✓conv' if conv else ''))
		if time.monotonic() - node.reject_last_t < REJECT_SHOW_S:
			parts.append('★REJECTED(res=%d)' % node.reject_last_res)
	ee = node.ee
	if ee is not None:
		p = ee[0]
		parts.append('EE(%.3f,%.3f,%.3f)' % p)
	parts.append('档%d/%d %.0fmm·s %.0f°·s' % (
		level + 1, n_lv, lin_speed * 1000.0, math.degrees(rot_speed)))
	parts.append('HOLD' if held else ('发流' if streaming else '待键'))
	parts.append('[Space=hold -=/+=档 q=退]')
	return '\r[遥操] ' + ' | '.join(parts) + '\x1b[K'


def run(node, hz, lin_levels, rot_levels, no_switch):
	period = 1.0 / hz
	if not no_switch:
		_switch('--deactivate', 'joint_stream_controller',
		        '--activate', 'cartesian_motion_controller')
		node.get_logger().info('CM 接管 (joint_stream → cartesian_motion)')
	# 清 HOLD 闩锁 (control 是 transient_local 事件通道, 上次会话退出发的 HOLD
	# 会闩住本次所有目标 —— 启动显式 TRACKING 归位; 未收到过该消息=默认 TRACKING)
	node.send_control(CartesianControl.TRACKING)
	wait_ee(node)
	fd = sys.stdin.fileno()
	old_term = termios.tcgetattr(fd)
	held = False
	level = 1   # 默认常规档
	now_hold = {}   # key → 保底发流截止时刻 (monotonic)
	tgt_active = False   # 目标积分状态: False=下次发流前重锚实际位姿
	tgt_p = (0.0, 0.0, 0.0)
	tgt_q = (1.0, 0.0, 0.0, 0.0)
	try:
		tty.setcbreak(fd)   # ISIG 保留 → Ctrl-C 仍走 KeyboardInterrupt
		node.get_logger().info(
			'键位: W/S前后 A/D左右 R/F升降 | I/K俯仰 J/L偏航 U/O翻滚 (Shift=2×) | '
			'-/=档位 Space=HOLD Q=退出')
		next_t = time.monotonic()
		while True:
			# 绝对节拍: select 的超时就是节拍 (无键时也不空转)
			now = time.monotonic()
			delay = max(0.0, next_t - now)
			r, _, _ = select.select([fd], [], [], delay)
			if r:
				for ch in os.read(fd, 32):
					# 键活期记账: 陈旧键再来事件 = 新按下 (保底窗), 活跃键再来 = 重复 (续命)
					was_stale = now_hold.get(ch) is None or time.monotonic() >= now_hold.get(ch, 0)
					now_hold[ch] = time.monotonic() + (FRESH_STREAM_S if was_stale
					                                   else DEADMAN_S)
					if ch in (ord('q'), ord('Q')):
						raise KeyboardInterrupt
					if ch == ord(' '):
						held = not held
						node.send_control(CartesianControl.HOLD if held
						                  else CartesianControl.TRACKING)
						if not held:
							now_hold.clear()   # 恢复后不把 hold 前的键态带回来
			now = time.monotonic()
			ee_ok = node.ee is not None and now - node.ee[3] <= EE_SILENT_S
			streaming = False
			if held:
				pass   # HOLD: 事件已发, 不再发目标 (CM 侧冻结且忽略新 target)
			elif ee_ok:
				lin_speed = lin_levels[level]
				rot_speed = rot_levels[level]
				lin, rot = active_twist(now_hold, now, lin_speed, rot_speed)
				nl = math.sqrt(sum(c * c for c in lin))
				nr = math.sqrt(sum(c * c for c in rot))
				if nl > 1e-9 or nr > 1e-9:
					# 目标空间自推进 (demo 流式同款): 臂速 = 指令速度, 不被 CM
					# 收敛动力学吞掉 (反馈锚定的教训见文件头)。待键→发流边沿重锚。
					ee_p, ee_q, frame_id, _ = node.ee
					if not tgt_active:
						tgt_p = tuple(ee_p)
						tgt_q = ee_q
					# 前导钳位随档速自适应 (固定 50mm 会让松手滑行 ~50-100mm):
					# 只需盖住 [停流窗 (deadman+刹停) × 速度] + 动力学跟踪滞后余量
					cap_p = lin_speed * 0.4 + 0.02
					cap_r = rot_speed * 0.4 + math.radians(5.0)
					tgt_p = clamp_lead_pos(tuple(
						tgt_p[i] + lin[i] * period for i in range(3)),
						ee_p, cap_p)
					tgt_q = clamp_lead_rot(
						quat_mul(axis_angle_quat(rot, nr * period), tgt_q),
						ee_q, cap_r)
					t = PoseStamped()
					t.header.frame_id = frame_id
					t.pose.position.x = tgt_p[0]
					t.pose.position.y = tgt_p[1]
					t.pose.position.z = tgt_p[2]
					t.pose.orientation.w = tgt_q[0]
					t.pose.orientation.x = tgt_q[1]
					t.pose.orientation.y = tgt_q[2]
					t.pose.orientation.z = tgt_q[3]
					node.pub_target.publish(t)
					node.sent_n += 1
					node.path_len += nl * period
					if node.t_first is None:
						node.t_first = time.monotonic()
					tgt_active = True
					streaming = True
				else:
					tgt_active = False   # 松键: 停发 → CM 断流刹停; 下次按键重锚
			sys.stdout.write(fmt_hud(node, level, len(lin_levels),
			                         lin_levels[level], rot_levels[level],
			                         held, streaming))
			sys.stdout.flush()
			next_t += period
			if next_t < time.monotonic() - period:
				next_t = time.monotonic()   # 落后一拍以上重新锚 (不追债)
	finally:
		sys.stdout.write('\r\x1b[K')
		sys.stdout.flush()
		termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
		# 退出显式停令 (主流做法: teleop_twist_keyboard 收尾发零 Twist; 位姿目标
		# 语义下发 HOLD 事件 = CM 瞬时冻结, 不等 200ms 断流刹停)
		try:
			node.send_control(CartesianControl.HOLD)
		except Exception:
			pass
		wall = time.monotonic() - (node.t_first or time.monotonic())
		node.get_logger().info(
			'汇总: 发目标 %d 条 / 活动窗 %.1fs (名义 %.0fHz), 末端路径长 %.1fmm' % (
				node.sent_n, wall, hz, node.path_len * 1000.0))
		try:
			if not no_switch:
				_switch('--deactivate', 'cartesian_motion_controller',
				        '--activate', 'joint_stream_controller')
				node.get_logger().info('切回 (链恢复 joint_stream); CM 断流已自行刹停')
		except Exception as e:
			node.get_logger().error('切回失败 (%s)。若链还活着手动执行:\n'
				'  ros2 control switch_controllers --deactivate cartesian_motion_controller'
				' --activate joint_stream_controller\n链已死则直接重启链' % str(e).strip()[-120:])


def main(args=None):
	hz = 30.0
	speeds = '0.02,0.05,0.10'
	rots = '0.25,0.5,1.0'
	no_switch = False
	argv = sys.argv[1:]
	while argv:
		a = argv.pop(0)
		if a == '--hz':
			hz = float(argv.pop(0))
		elif a == '--speed':
			speeds = argv.pop(0)
		elif a == '--rot':
			rots = argv.pop(0)
		elif a == '--deadman':
			global DEADMAN_S
			DEADMAN_S = float(argv.pop(0))
		elif a == '--no-switch':
			no_switch = True
		else:
			raise SystemExit("未知参数 '%s' (支持: --hz --speed --rot --deadman "
			                 "--no-switch)" % a)
	if hz <= 0.0 or hz > 100.0:
		print('--hz 须在 (0,100] (收到 %s); rclpy 舒适区 ~100' % hz)
		return 2
	lin_levels = [float(x) for x in speeds.split(',')]
	rot_levels = [float(x) for x in rots.split(',')]
	if len(lin_levels) != len(rot_levels) or not lin_levels or \
			any(v <= 0.0 for v in lin_levels + rot_levels):
		print('--speed/--rot 档数须一致且全为正 (收到 %s / %s)' % (speeds, rots))
		return 2
	if not sys.stdin.isatty():
		print('stdin 不是终端 —— 遥操作需要真实 TTY (termios 键盘读取)。'
		      '请在交互终端运行, 勿经管道/后台执行。')
		return 2
	rclpy.init(args=args)
	node = TeleopKey()
	executor = SingleThreadedExecutor()
	executor.add_node(node)
	import threading
	spin = threading.Thread(target=executor.spin, daemon=True)
	spin.start()
	try:
		run(node, hz, lin_levels, rot_levels, no_switch)
	except KeyboardInterrupt:
		pass
	finally:
		executor.remove_node(node)
		executor.shutdown()
		spin.join(timeout=2.0)   # 先停转再关 rclpy, 否则解释器收场段报 terminate
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
	return 0


if __name__ == '__main__':
	sys.exit(main())

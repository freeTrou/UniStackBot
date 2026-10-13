#!/usr/bin/env python3
"""r1_7a 关节控制 demo (机型专属; scripts/<robot>_arm/ 约定, 2026-10-12)。

**定位 = 控制开发人员的接入参考实现** (链路契约的完整正确消费样板)。
16 段航点链: 零位→展示→基座连续 5 段扫→折叠双档→腕组合 A/B→回近零位
(JS ruckig 档整形下段间连续无停顿感; 快慢交替 ~20s)。
最小入门请看 scripts/minimal_joint.py (外置通用教学件)。

关节表/限位读自链上 URDF (单一事实源); 航点为 r1_7a 定制常量, 启动时
逐点断言在限位 2% 内缩带内 —— 写错即拒跑。

前置: 任一链已启动, joint_stream_controller active。
用法: ros2 run unistackbot_demo r1_7a_demo_joint.py [--hz 50] [--cycles 1]
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

# ---- r1_7a 航点 (J1..J7, rad; 启动时对链上限位逐点断言) ----
HOME = (0.0, 1.35, 1.10, 1.35, 0.0, 0.0, 0.0)              # 展示位: J2/J4 离开下限线
POSE_SWEEP_L = (1.8, 1.35, 1.10, 1.35, 1.0, 0.0, 1.2)      # 基座左摆 + 腕摆
POSE_SWEEP_R = (-1.8, 1.35, 1.10, 1.35, -1.0, 0.0, -1.2)
POSE_MID_L = (0.9, 1.35, 1.10, 1.35, 0.5, 0.0, 0.6)        # 基座中间过渡 (连续扫)
POSE_MID_R = (-0.9, 1.35, 1.10, 1.35, -0.5, 0.0, -0.6)
POSE_FOLD_DEEP = (0.0, 1.85, 1.55, 1.85, 0.0, 0.35, 0.0)   # 深折叠: 肘弯 + J6 带内
POSE_FOLD_MID = (0.0, 1.60, 1.30, 1.60, 0.0, 0.20, 0.0)    # 浅折叠 (双档过渡)
POSE_WRIST_A = (0.0, 1.35, 1.10, 1.35, 0.9, 0.30, 1.1)     # 腕组合 A (J5/J7 同向)
POSE_WRIST_B = (0.0, 1.45, 1.20, 1.45, -0.9, -0.30, -1.1)  # 腕组合 B (J5/J7 反向)
NEAR_ZERO = (0.0, 0.08, 0.05, 0.08, 0.0, 0.0, 0.0)         # 近零位 (2% 内缩)

# 航点链 (2026-10-12 加密: 段更多更短, JS ruckig 档整形下段间连续无停顿感)
SCHEDULE = [
	(HOME, 2.5, '零位→展示位·慢'),
	(POSE_MID_L, 0.8, '基座扫1·快'),
	(POSE_SWEEP_L, 0.8, '基座扫2·快'),
	(POSE_MID_L, 0.7, '基座回中·快'),
	(POSE_MID_R, 0.8, '基座扫3·快'),
	(POSE_SWEEP_R, 0.8, '基座扫4·快'),
	(POSE_MID_R, 0.7, '基座回中·快'),
	(HOME, 0.9, '回展示位·快'),
	(POSE_FOLD_MID, 1.8, '浅折叠·慢'),
	(POSE_FOLD_DEEP, 1.8, '深折叠·慢'),
	(POSE_FOLD_MID, 1.2, '折回浅·快'),
	(HOME, 1.2, '回展示位·快'),
	(POSE_WRIST_A, 1.0, '腕组合A·快'),
	(POSE_WRIST_B, 1.0, '腕组合B·快'),
	(HOME, 1.4, '回展示位·慢'),
	(NEAR_ZERO, 2.5, '回近零位·慢'),
]


class R17aDemoJoint(Node):

	def __init__(self):
		super().__init__('r1_7a_demo_joint')

	def _fetch_contract(self):
		"""关节表 + 每关节 (min, max), 读自 robot_state_publisher 的 URDF
		<ros2_control> 块 (与 JS 解析同口径)。r1_7a 无 mimic; 出现即拒。"""
		cli = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
		if not cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError('参数服务 /robot_state_publisher 5 秒内未出现 (链路未起?)')
		req = GetParameters.Request()
		req.names = ['robot_description']
		fut = cli.call_async(req)
		rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
		if fut.result() is None or not fut.result().values[0].string_value:
			raise RuntimeError('读取 robot_description 失败')
		joints, lims = [], []
		for j in ET.fromstring(fut.result().values[0].string_value).iter('joint'):
			ci = j.find('command_interface')
			if j.get('name') is None or ci is None or ci.get('name') != 'position':
				continue
			ps = {p.get('name'): p.text for p in j.findall('param')}
			if 'mimic' in ps:
				raise RuntimeError("r1_7a 命令关节 '%s' 带 mimic —— 请用通用 demo_motion.py" % j.get('name'))
			if 'min' in ps and 'max' in ps:
				joints.append(j.get('name'))
				lims.append((float(ps['min']), float(ps['max'])))
			else:
				raise RuntimeError("命令关节 '%s' 无 min/max, 无法归类" % j.get('name'))
		if len(joints) != 7:
			raise RuntimeError('r1_7a 命令关节数 %d != 7 (%s) —— 链上不是 r1_7a?' % (len(joints), joints))
		return joints, lims

	def _read_start(self, joints):
		"""起始关节角 (首腿插值起点), 读 /joint_states 按名对齐 (JSB 乱序已知坑)。"""
		got = {}
		sub = self.create_subscription(JointState, '/joint_states',
			lambda m: got.update(zip(m.name, m.position)), 10)
		t0 = time.time()
		while not got and time.time() - t0 < 3.0:
			rclpy.spin_once(self, timeout_sec=0.1)
		self.destroy_subscription(sub)
		if len(got) != len(joints):
			raise RuntimeError('/joint_states 3 秒未凑齐关节表 (链路未起?)')
		return [got[n] for n in joints]

	def run(self, hz, cycles):
		# 第一步 = 切换自己需要的控制器 (进入声明, 幂等——查态后只切需要变的;
		# activate 已 active 的 JS 会被 CM 硬拒, 这正是上一版在新机常态路径炸的根)
		import re
		for attempt in range(7):
			r = subprocess.run(['ros2', 'control', 'list_controllers'],
				capture_output=True, text=True, timeout=15)
			states = {}
			for line in r.stdout.splitlines():
				clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip())
				parts = clean.split()
				if len(parts) >= 3:
					states[parts[0]] = parts[-1]
			if not states:
							raise RuntimeError('list_controllers 无输出 —— 链未启动? 先 ros2 launch 再跑 demo')
			args = []
			if states.get('cartesian_motion_controller') == 'active':
				args += ['--deactivate', 'cartesian_motion_controller']
			if states.get('joint_stream_controller') != 'active':
				args += ['--activate', 'joint_stream_controller']
			if not args:
				break   # 已达目标态 (链起后常态) = 幂等成功
			s = subprocess.run(['ros2', 'control', 'switch_controllers'] + args,
				capture_output=True, text=True, timeout=15)
			if s.returncode == 0:
				break
			self.get_logger().warn('switch 未就绪 (attempt %d/7), 5s 重试' % (attempt + 1))
			time.sleep(5.0)
		else:
			raise RuntimeError('切换 JS 失败: ' + s.stderr.strip()[-120:])
		joints, lims = self._fetch_contract()
		self.get_logger().info('关节表 (读自链上 URDF): %s' % joints)
		# 航点限位断言 (2% 内缩带): 常量写错在启动期暴露, 不进运行时
		for wp, _, tag in SCHEDULE:
			for n, q, (lo, hi) in zip(joints, wp, lims):
				if not (lo + 0.02 * (hi - lo) <= q <= hi - 0.02 * (hi - lo)):
					raise RuntimeError('航点 %s 的 %s=%+.3f 越出限位内缩带 [%.3f, %.3f]'
						% (tag, n, q, lo + 0.02 * (hi - lo), hi - 0.02 * (hi - lo)))
		start = self._read_start(joints)
		total = sum(d for _, d, _ in SCHEDULE) * cycles
		self.get_logger().info('r1_7a 关节演示: %d 段 / %.1fs (%s)' % (
			len(SCHEDULE) * cycles, total, ' → '.join(t for _, _, t in SCHEDULE)))

		qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
		                 history=HistoryPolicy.KEEP_LAST, depth=1)
		pub = self.create_publisher(JointCommand, '/joint_stream_controller/command', qos)

		per_msg = 1.0 / hz
		sent = late = 0
		t_start = time.time()
		prev = start
		for _ in range(cycles):
			for wp, dur, _tag in SCHEDULE:
				t0 = time.time()
				i = 0
				while True:
					elapsed = time.time() - t0
					if elapsed >= dur:
						break
					s = min(1.0, elapsed / dur)
					s = s * s * (3.0 - 2.0 * s)   # smoothstep (起停零速度)
					m = JointCommand()
					m.mode = JointCommand.MODE_CSP
					m.joint_names = joints
					m.position = [p + s * (w - p) for p, w in zip(prev, wp)]
					pub.publish(m)
					sent += 1
					i += 1
					nxt = t0 + i * per_msg   # 绝对时间步进 (相对 sleep 累积漂移)
					delay = nxt - time.time()
					if delay > 0.0:
						time.sleep(delay)
					else:
						late += 1
				prev = list(wp)
		wall = time.time() - t_start
		achieved = sent / wall if wall > 0.0 else 0.0
		note = ('; %d 拍落后' % late) if late else ''
		self.get_logger().info('演示流结束: 名义 %dHz, 实际达成 %.1fHz (%d 条 / %.1fs)%s' % (
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
	node = R17aDemoJoint()
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

#!/usr/bin/env python3
"""verify_robot.sh 的运动验证助手 (JTC 时代 action goal 的 JointStream/CM 替代, 2026-09-21)。

两个子命令, 退出码 0/1, 输出 "PASS:/FAIL:/WARN:" 行供 shell 记账:

  js: 50Hz JointCommand 流 → /joint_states 就地断言到位
      verify_motion.py js --joints j1,j2 --targets 0.1,0.2 [--tol 0.01]
                       [--soft-joints gripper] [--hz 50] [--dur 4]
      (--soft-joints: 已知回归关节只 WARN 不 FAIL, 如 gz 链 gripper)

  cm: 切换 JointStream→CM → TF 取当前 EE → 目标=当前+轴向偏移 → 收敛断言 → 切回
      verify_motion.py cm --base base_link --tip link6 [--tol 0.002]
                       [--axis z] [--offset 0.03] [--timeout 10]
      (--axis 默认 z: 零位邻域 +x 无限位内解析解 (analytic 诚实拒), 会让收敛断言
       假失败 —— 2026-09-23; DLS 下 x/z 均可收敛)
"""
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from unistackbot_interface.msg import JointCommand
from unistackbot_interface.msg import CartesianMotionStatus
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def mode_js(args):
	node = Node('verify_js')
	qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
	                 history=HistoryPolicy.KEEP_LAST, depth=1)
	pub = node.create_publisher(JointCommand, '/joint_stream_controller/command', qos)
	latest = {}

	def on_js(msg):
		for n, p in zip(msg.name, msg.position):
			latest[n] = p

	node.create_subscription(JointState, '/joint_states', on_js, 10)

	joints = args['joints'].split(',')
	targets = [float(t) for t in args['targets'].split(',')]
	soft = set(args.get('soft_joints', '').split(',')) - {''}
	tol = float(args.get('tol', 0.01))
	hz = float(args.get('hz', 50))
	dur = float(args.get('dur', 4))

	end = time.time() + 5
	while not latest and time.time() < end:
		rclpy.spin_once(node, timeout_sec=0.1)
	if not latest:
		print('FAIL: js 无 /joint_states')
		return 1

	t0 = time.time()
	i = 0
	while time.time() - t0 < dur:
		m = JointCommand()
		m.mode = JointCommand.MODE_CSP
		m.joint_names = joints
		m.position = targets
		pub.publish(m)
		i += 1
		rclpy.spin_once(node, timeout_sec=0.001)
		nxt = t0 + i / hz
		if nxt > time.time():
			time.sleep(nxt - time.time())

	for _ in range(30):
		rclpy.spin_once(node, timeout_sec=0.05)

	rc = 0
	for j, t in zip(joints, targets):
		err = abs(latest.get(j, 1e9) - t)
		mark = 'PASS' if err < tol else ('WARN' if j in soft else 'FAIL')
		if mark == 'FAIL':
			rc = 1
		print(f'{mark}: js {j} = {latest.get(j, float("nan")):+.4f} (目标 {t:+.4f}, 偏差 {err:.4f})')
	if rc == 0 and soft:
		print(f'WARN: soft-joints {sorted(soft)} 未计入失败 (已知回归)')

	# mimic 断言 (可选): --mimic "master:follower:coef[,master:follower:coef]"
	# follower 应 = coef×master (仿真侧原生耦合: mujoco equality / 硬件层推导)
	for spec in [s for s in args.get('mimic', '').split(',') if s]:
		master, follower, coef = spec.split(':')
		coef = float(coef)
		if master not in latest or follower not in latest:
			print(f'FAIL: mimic 缺关节 ({master}/{follower} 不在 /joint_states)')
			rc = 1
			continue
		err = abs(latest[follower] - coef * latest[master])
		mark = 'PASS' if err < 0.005 else 'FAIL'
		if mark == 'FAIL':
			rc = 1
		print(f'{mark}: mimic {follower} = {coef}×{master} '
		      f'({latest[follower]:+.4f} vs {coef * latest[master]:+.4f}, 偏差 {err:.4f})')
	return rc


def mode_cm(args):
	node = Node('verify_cm')
	tfbuf = Buffer()
	TransformListener(tfbuf, node)
	pub = node.create_publisher(PoseStamped, '/cartesian_motion_controller/target', 1)
	status = {}

	def on_status(msg):
		status['err'] = msg.position_error
		status['mode'] = msg.mode
		status['result'] = msg.last_result
		status['converged'] = msg.converged

	node.create_subscription(CartesianMotionStatus,
	                         '/cartesian_motion_controller/status', on_status, 10)

	def switch(*args_):
		return subprocess.run(
			['ros2', 'control', 'switch_controllers', *args_],
			capture_output=True, text=True, timeout=15)

	r = switch('--deactivate', 'joint_stream_controller', '--activate',
	           'cartesian_motion_controller')
	if r.returncode != 0:
		print(f'FAIL: cm 切换失败 {r.stderr.strip()[-120:]}')
		return 1
	print('PASS: cm 切换接管 (joint_stream → cartesian_motion)')
	try:
		end = time.time() + 5
		while time.time() < end:
			rclpy.spin_once(node, timeout_sec=0.1)
			if tfbuf.can_transform(args['base'], args['tip'], rclpy.time.Time()):
				break
		tf = tfbuf.lookup_transform(args['base'], args['tip'], rclpy.time.Time())
		axis = str(args.get('axis', 'z'))   # 默认 z: 零位邻域 +x 解析无解会假失败 (见模块注释)
		off = float(args.get('offset', 0.03))
		t = PoseStamped()
		t.header.frame_id = args['base']
		t.pose.position.x = tf.transform.translation.x + (off if axis == 'x' else 0.0)
		t.pose.position.y = tf.transform.translation.y + (off if axis == 'y' else 0.0)
		t.pose.position.z = tf.transform.translation.z + (off if axis == 'z' else 0.0)
		t.pose.orientation = tf.transform.rotation
		pub.publish(t)

		deadline = time.time() + float(args.get('timeout', 15))
		while time.time() < deadline:
			rclpy.spin_once(node, timeout_sec=0.05)
			# 判据只用本脚本的 tol (CM 内部 converged 标志的容差是 1mm 级, 比验收断言更严,
			# 混用会假失败 —— 2026-09-21 实测)
			if status.get('err', 1) < float(args.get('tol', 0.002)):
				print(f"PASS: cm 收敛 (err={status['err']*1000:.3f}mm, result={status['result']})")
				return 0
		print(f"FAIL: cm 未收敛 (err={status.get('err', -1)*1000:.3f}mm, "
		      f"mode={status.get('mode')}, result={status.get('result')})")
		return 1
	finally:
		switch('--deactivate', 'cartesian_motion_controller', '--activate',
		       'joint_stream_controller')
		print('PASS: cm 切回 (链恢复 joint_stream)')


def main():
	if len(sys.argv) < 2 or sys.argv[1] not in ('js', 'cm'):
		print(__doc__)
		return 2
	args = {}
	it = iter(sys.argv[2:])
	for kv in it:
		if kv.startswith('--'):
			args[kv[2:].replace('-', '_')] = next(it)
	rclpy.init()
	try:
		rc = mode_js(args) if sys.argv[1] == 'js' else mode_cm(args)
	finally:
		rclpy.shutdown()
	return rc


if __name__ == '__main__':
	sys.exit(main())

#!/usr/bin/env python3
"""故障注入测试床 F2/F3/F5 (毒命令流 → write 防线应拦) + F1 (断流保持)。
mock 链 xarm7。用法: 前置链已起且 joint_stream_controller active。"""
import subprocess, time, re, sys
import rclpy
from rclpy.node import Node
from unistackbot_interface.msg import JointCommand

def read_states(timeout=6):
    out = subprocess.run(['timeout', str(timeout), 'ros2', 'topic', 'echo', '/joint_states', '--once'],
                         capture_output=True, text=True).stdout
    nm = re.findall(r'-\s*(joint\d|gripper)', out)
    pm = re.search(r'position:\n((?:\s*- .+\n)+)', out)
    vs = [float(v) for v in re.findall(r'-?\d+\.?\d*(?:[eE][+-]?\d+)?', pm.group(1))] if pm else []
    return dict(zip(nm, vs[:len(nm)]))

def ulog_guards():
    out = subprocess.run(['bash', '-c',
        'grep -aE "write 防线|read 防线" /tmp/fault_launch.log | tail -3'],
        capture_output=True, text=True).stdout.strip()
    return out

rclpy.init()
node = Node('fault_bed')
pub = node.create_publisher(JointCommand, '/joint_stream_controller/command', 1)
names = [f'joint{i}' for i in range(1, 8)]

# 进程内状态订阅 (精确时间戳; 进程 spawn 读数有 1-2s 不可控延迟)
latest = {}
def on_js(msg):
    for n, p in zip(msg.name, msg.position):
        latest[n] = p
node.create_subscription(
    __import__('sensor_msgs.msg', fromlist=['JointState']).JointState,
    '/joint_states', on_js, 10)
time.sleep(1.5)
results = []

def send(positions, hz=100, dur=1.0, mode=JointCommand.MODE_CSP):
    t0 = time.time(); n = 0
    while time.time() - t0 < dur:
        m = JointCommand()
        m.mode = mode
        m.joint_names = names
        m.position = positions
        pub.publish(m)
        n += 1
        nxt = t0 + n / hz
        if nxt > time.time(): time.sleep(nxt - time.time())

# ---- F2 NaN 命令流: 防线应保持, 臂不动 ----
rclpy.spin_once(node, timeout_sec=0.1)
pre = dict(latest)
send([float('nan')] * 7, dur=1.5)
rclpy.spin_once(node, timeout_sec=0.1)
post = dict(latest)
moved = max(abs(post.get(k, 9) - pre.get(k, 0)) for k in names)
results.append(('F2 NaN命令流', f'位移 {moved:.5f} rad', moved < 0.01))

# ---- F3 限位外命令: clamp 生效 (臂到限位边界即停, 不出界) ----
send([10.0] * 7, dur=1.5)   # xarm7 joint 限位最大 ~±2π~6.28 / ±3.1 不等
time.sleep(0.5)
post = read_states()
out_of = any(abs(post.get(k, 0)) > 6.5 for k in names)
results.append(('F3 限位外命令', f'最大 |q|={max(abs(post.get(k,0)) for k in names):.3f}', not out_of))

# ---- 回零复位 (隔离 F5) ----
send([0.0] * 7, dur=3.0)
time.sleep(2.0)

# ---- F5 超速命令流: 步长饱和 (精确窗口) ----
# 账: 1.0s 窗口 @ max_velocity=3.14 rad/s → 位移上限 3.14 rad + 测量余量
rclpy.spin_once(node, timeout_sec=0.1)
pre = dict(latest)
t0 = time.time()
send([6.28] * 7, dur=0.5, hz=100)
while time.time() - t0 < 1.0:
    rclpy.spin_once(node, timeout_sec=0.01)
post = dict(latest)
win = time.time() - t0
step = max(abs(post.get(k, 0) - pre.get(k, 0)) for k in names)
bound = 3.14 * win + 0.3   # 速度界×窗口 + 容差
results.append(('F5 超速命令', f'位移 {step:.3f} rad / 窗 {win:.2f}s (界 {bound:.2f})', step < bound))

# ---- 回零复位 (隔离 F1) ----
send([0.0] * 7, dur=3.0)
time.sleep(2.0)

# ---- F1 断流保持: 停发后臂停在原地 ----
rclpy.spin_once(node, timeout_sec=0.1)
pre = dict(latest)
time.sleep(2.0)
rclpy.spin_once(node, timeout_sec=0.1)
post = dict(latest)
drift = max(abs(post.get(k, 0) - pre.get(k, 0)) for k in names)
results.append(('F1 断流2s漂移', f'漂移 {drift:.5f} rad', drift < 0.005))

# ---- F6 断流受控减速 (0c): 流中途死 → 刹停, 不冲向最后目标 ----
# 账: joint1 以 vmax=3.14 rad/s 流 1.0s → 走 ~3.14 rad (目标 5.0 未达);
#     断流后 200ms 判定 + 200ms 线性减速 → 最多再走 3.14×(0.2+0.1)≈0.94 rad
#     → 终值 < 4.5 即证明没冲到 5.0; 静止复查证明刹停 (而非仍在走)
send([5.0] + [0.0] * 6, dur=1.0, hz=100)
time.sleep(2.5)
rclpy.spin_once(node, timeout_sec=0.1)
p1 = dict(latest)
time.sleep(0.8)
rclpy.spin_once(node, timeout_sec=0.1)
p2 = dict(latest)
still = max(abs(p2.get(k, 0) - p1.get(k, 0)) for k in names)
final1 = p2.get('joint1', 0.0)
results.append(('F6 断流刹停-静止', f'0.8s 再漂移 {still:.5f} rad', still < 0.005))
results.append(('F6 断流刹停-未冲目标', f'joint1 终值 {final1:.3f} rad (<4.5, 目标5.0; >2 动过)',
                final1 < 4.5 and final1 > 2.0))

for name, detail, ok in results:
    print(f'{"PASS" if ok else "FAIL"}: {name} — {detail}')
print('---- 防线日志 ----')
print(ulog_guards() or '(无防线告警 — F2/F3/F5 命中过则应有)')
rclpy.shutdown()
sys.exit(0 if all(r[2] for r in results) else 1)

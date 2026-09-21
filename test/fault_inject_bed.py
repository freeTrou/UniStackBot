#!/usr/bin/env python3
"""故障注入测试床 F2/F3/F5 (毒命令流 → write 防线应拦) + F1 (断流保持) + F6 (断流受控减速) + F7 (mimic 耦合)。

链参数化 (2026-09-21, 批次3): 关节表/阈值经环境变量注入 (由 fault_injection.sh 按
--chain 设置), 默认值 = mock xarm7 (向后兼容):

  BED_JOINTS    逗号分隔关节表 (默认 joint1..joint7)
  BED_VMAX      F5 速度界基数 rad/s (默认 3.14 = xarm7)
  BED_F6_TARGET F6 流目标 (首关节; 默认 5.0)
  BED_F6_STREAM F6 流时长 s (默认 1.0)
  BED_F6_HI/LO  F6 终值区间 (默认 4.5/2.0)
  BED_MIMIC     mimic 断言 "master:follower:coef,..." (默认空 = 跳过 F7)

前置: 链已起且 joint_stream_controller active。"""
import subprocess, time, re, sys, os
import rclpy
from rclpy.node import Node
from unistackbot_interface.msg import JointCommand

JOINTS = (os.environ.get('BED_JOINTS') or ','.join(f'joint{i}' for i in range(1, 8))).split(',')
VMAX = float(os.environ.get('BED_VMAX', '3.14'))
F6_TARGET = float(os.environ.get('BED_F6_TARGET', '5.0'))
F6_STREAM = float(os.environ.get('BED_F6_STREAM', '1.0'))
F6_HI = float(os.environ.get('BED_F6_HI', '4.5'))
F6_LO = float(os.environ.get('BED_F6_LO', '2.0'))
MIMIC = [s for s in os.environ.get('BED_MIMIC', '').split(',') if s]
N = len(JOINTS)

def read_states(timeout=6):
    out = subprocess.run(['timeout', str(timeout), 'ros2', 'topic', 'echo', '/joint_states', '--once'],
                         capture_output=True, text=True).stdout
    nm = re.findall(r'-\s*(joint\d|gripper\w*)', out)
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

# 进程内状态订阅 (精确时间戳; 进程 spawn 读数有 1-2s 不可控延迟)
latest = {}
def on_js(msg):
    for n, p in zip(msg.name, msg.position):
        latest[n] = p
node.create_subscription(
    __import__('sensor_msgs.msg', fromlist=['JointState']).JointState,
    '/joint_states', on_js, 10)
time.sleep(1.5)
# 死链硬门: 5s 无 /joint_states 直接失败退出 (空洞 PASS 比失败更糟)
_deadline = time.time() + 5.0
while not latest and time.time() < _deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if not latest:
    print('FAIL: 5s 无 /joint_states (链未就绪) — 中止, 不产出空洞结果')
    rclpy.shutdown()
    sys.exit(1)
results = []

def send(positions, hz=100, dur=1.0, mode=JointCommand.MODE_CSP):
    t0 = time.time(); n = 0
    while time.time() - t0 < dur:
        m = JointCommand()
        m.mode = mode
        m.joint_names = JOINTS
        m.position = positions
        pub.publish(m)
        n += 1
        nxt = t0 + n / hz
        if nxt > time.time(): time.sleep(nxt - time.time())

# ---- F2 NaN 命令流: 防线应保持, 臂不动 ----
rclpy.spin_once(node, timeout_sec=0.1)
pre = dict(latest)
send([float('nan')] * N, dur=1.5)
rclpy.spin_once(node, timeout_sec=0.1)
post = dict(latest)
moved = max(abs(post.get(k, 9) - pre.get(k, 0)) for k in JOINTS)
results.append(('F2 NaN命令流', f'位移 {moved:.5f} rad', moved < 0.01))

# ---- F3 限位外命令: clamp 生效 (臂到限位边界即停, 不出界) ----
send([10.0] * N, dur=1.5)   # 各机型限位不一, 出界判定用宽松上界
time.sleep(0.5)
post = read_states()
out_of = any(abs(post.get(k, 0)) > 6.5 for k in JOINTS)
results.append(('F3 限位外命令', f'最大 |q|={max(abs(post.get(k,0)) for k in JOINTS):.3f}', not out_of))

# ---- 回零复位 (隔离 F5) ----
send([0.0] * N, dur=3.0)
time.sleep(2.0)

# ---- F5 超速命令流: 步长饱和 (精确窗口) ----
# 账: 窗口时长 × vmax + 容差 (vmax 链参数)
rclpy.spin_once(node, timeout_sec=0.1)
pre = dict(latest)
t0 = time.time()
send([6.28] * N, dur=0.5, hz=100)
while time.time() - t0 < 1.0:
    rclpy.spin_once(node, timeout_sec=0.01)
post = dict(latest)
win = time.time() - t0
step = max(abs(post.get(k, 0) - pre.get(k, 0)) for k in JOINTS)
bound = VMAX * win + 0.3   # 速度界×窗口 + 容差
results.append(('F5 超速命令', f'位移 {step:.3f} rad / 窗 {win:.2f}s (界 {bound:.2f})', step < bound))

# ---- 回零复位 (隔离 F1) ----
send([0.0] * N, dur=3.0)
time.sleep(2.0)

# ---- F1 断流保持: 停发后臂停在原地 ----
rclpy.spin_once(node, timeout_sec=0.1)
pre = dict(latest)
time.sleep(2.0)
rclpy.spin_once(node, timeout_sec=0.1)
post = dict(latest)
drift = max(abs(post.get(k, 0) - pre.get(k, 0)) for k in JOINTS)
results.append(('F1 断流2s漂移', f'漂移 {drift:.5f} rad', drift < 0.005))

# ---- F6 断流受控减速 (0c): 流中途死 → 刹停, 不冲向最后目标 ----
# 账: 流 F6_STREAM 秒 @vmax → 位姿 p0; 断流 200ms 判定 + 200ms 线性减速
#     → 终值 ≤ p0 + vmax*(0.2+0.1) + 容差; F6_HI 即证明没冲到目标, F6_LO 证明动过
send([F6_TARGET] + [0.0] * (N - 1), dur=F6_STREAM, hz=100)
time.sleep(2.5)
rclpy.spin_once(node, timeout_sec=0.1)
p1 = dict(latest)
time.sleep(0.8)
rclpy.spin_once(node, timeout_sec=0.1)
p2 = dict(latest)
still = max(abs(p2.get(k, 0) - p1.get(k, 0)) for k in JOINTS)
final1 = p2.get(JOINTS[0], 0.0)
results.append(('F6 断流刹停-静止', f'0.8s 再漂移 {still:.5f} rad', still < 0.005))
results.append((f'F6 断流刹停-未冲目标',
                f'{JOINTS[0]} 终值 {final1:.3f} rad (<{F6_HI}, 目标{F6_TARGET}; >{F6_LO} 动过)',
                final1 < F6_HI and final1 > F6_LO))

# ---- F7 mimic 耦合 (可选, 仿真侧原生跟随: mujoco equality): 从动=系数×主 ----
if MIMIC:
    send([0.0] * (N - 1) + [0.08], dur=2.5)   # 主关节(表尾)收放, 其余零
    time.sleep(1.5)
    rclpy.spin_once(node, timeout_sec=0.1)
    for spec in MIMIC:
        master, follower, coef = spec.split(':')
        coef = float(coef)
        mv, fv = latest.get(master), latest.get(follower)
        if mv is None or fv is None:
            results.append((f'F7 mimic {follower}', '缺关节读数', False))
            continue
        err = abs(fv - coef * mv)
        results.append((f'F7 mimic {follower}',
                        f'{follower}={fv:.4f} vs {coef}×{master}={coef*mv:.4f} (偏差 {err:.4f})',
                        err < 0.005))

for name, detail, ok in results:
    print(f'{"PASS" if ok else "FAIL"}: {name} — {detail}')
print('---- 防线日志 ----')
print(ulog_guards() or '(无防线告警 — F2/F3/F5 命中过则应有)')
rclpy.shutdown()
sys.exit(0 if all(r[2] for r in results) else 1)

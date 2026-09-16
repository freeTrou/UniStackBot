#!/usr/bin/env bash
# FK vs TF 交叉对拍 (sim 验证核心)。
# 原理: robot_state_publisher 用独立实现 (kdl_parser 建树) 从同一 URDF 算 /tf,
#       我们的 UrdfFk 用手搓链 —— 两套代码同源数据, 差值应到机器精度。
#       错一处 (轴向/origin/旋转序) 就对不上 —— 这是 FK 正确性的工程裁判。
# 流程: 起 mock 链 -> pause (JTC 保持命令不干扰瞬移) -> 10 个随机位姿:
#       set_joint_state 瞬移 -> 读 /tf link7 位姿 vs fk_tool 算的位姿 -> 比对 <1e-9。
# 用法: bash test/check_fk_tf.sh <robot>    (需已 colcon build + source)
set -u
ROBOT="${1:?用法: check_fk_tf.sh <robot>}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WS_ROOT="$(dirname "$(dirname "$REPO_ROOT")")"

set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
[ -f "$HOME/cyclonedds.xml" ] && export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

PASS=0; FAIL=0
say() { echo "[$(date +%H:%M:%S)] $*"; }

say "清理残留"
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1

say "起 mock 链 ($ROBOT)"
LOG=/tmp/check_fk_tf.log
ros2 launch unistackbot_bringup control.launch.py "robot:=$ROBOT" > "$LOG" 2>&1 &
LPID=$!
for _ in $(seq 1 40); do
	C=$(timeout 5 ros2 control list_controllers 2>/dev/null)
	echo "$C" | grep -q "joint_state_broadcaster.*active" && \
	echo "$C" | grep -q "joint_trajectory_controller.*active" && break
	sleep 1
done
C=$(timeout 5 ros2 control list_controllers 2>/dev/null)
if ! echo "$C" | grep -q "joint_trajectory_controller.*active"; then
	say "FAIL: 控制器未激活"; kill $LPID 2>/dev/null; exit 1
fi

# pause + reset 到零位起点
timeout 8 ros2 service call /sim_control/pause std_srvs/srv/Trigger >/dev/null 2>&1
timeout 8 ros2 service call /sim_control/reset std_srvs/srv/Trigger >/dev/null 2>&1
sleep 0.5

# 10 个确定性随机位姿 (LCG, 覆盖限位 ±40% 区间)
say "对拍 10 个随机位姿 (pause 下瞬移, 全精度 TF, 容差 1e-9)"
RESULTS=$(python3 - "$ROBOT" <<'PYEOF'
import subprocess, sys, time, re

robot = sys.argv[1]
SETSTATE = "unistackbot_sim_control/srv/SetJointState"

# 与单测同源的 LCG 随机数 (确定性, 失败可复现)
seed = 20260917
def rnd():
    global seed
    seed = (seed * 1103515245 + 12345) % (2**31)
    return seed / 2**31

# 读限位 (从展开 URDF)
import xml.etree.ElementTree as ET
urdf = open(f"/tmp/verify_{robot}.urdf").read()
lim = {}
root = ET.fromstring(urdf)
for j in root.iter("joint"):
    if j.get("name") is None or j.find("command_interface") is None:
        continue
    ps = {p.get("name"): p.text for p in j.findall("param")}
    if "min" in ps and "max" in ps:
        lim[j.get("name")] = (float(ps["min"]), float(ps["max"]))
names = list(lim)

results = []
for trial in range(10):
    # 随机目标 (限位区间 ±40% 中心带, 避开极限)
    targets = []
    for n in names:
        lo, hi = lim[n]
        c, half = (lo + hi) / 2, (hi - lo) * 0.2
        targets.append(round(c + (rnd() - 0.5) * 2 * half, 4))
    yaml_arr = "[" + ",".join(str(t) for t in targets) + "]"
    yaml_names = "[" + ",".join(names) + "]"

    # 瞬移
    subprocess.run(["timeout", "8", "ros2", "service", "call", "/sim_control/set_joint_state",
        SETSTATE, f"{{joint_names: {yaml_names}, positions: {yaml_arr}}}"],
        capture_output=True)
    time.sleep(0.4)

    # 读 TF (全精度: tf2_echo 只打 3 位小数, 舍入 ~5e-4 会淹没 1e-9 容差)
    py_code = (
        "import rclpy, tf2_ros\n"
        "rclpy.init()\n"
        "node = rclpy.node.Node('tf_probe')\n"
        "buf = tf2_ros.Buffer()\n"
        "tf2_ros.TransformListener(buf, node)\n"
        "import time\n"
        "end = time.time() + 3\n"
        "t = None\n"
        "while time.time() < end:\n"
        "    rclpy.spin_once(node, timeout_sec=0.2)\n"
        "    if buf.can_transform('link_base', 'link7', rclpy.time.Time()):\n"
        "        t = buf.lookup_transform('link_base', 'link7', rclpy.time.Time())\n"
        "        break\n"
        "if t is None:\n"
        "    raise SystemExit('NO TF')\n"
        "tr = t.transform.translation\n"
        "print(f'{tr.x:.15f} {tr.y:.15f} {tr.z:.15f}')\n"
    )
    tf = subprocess.run(["timeout", "12", "python3", "-c", py_code],
        capture_output=True, text=True)
    vals = tf.stdout.split()
    if len(vals) != 3:
        results.append(("FAIL", trial + 1, f"TF 无数据: {tf.stderr[-80:]}"))
        continue
    tx, ty, tz = map(float, vals)

    # fk_tool 算同一位姿
    joint_args = ",".join(f"{n}={t}" for n, t in zip(names, targets))
    fk = subprocess.run(["timeout", "6", "ros2", "run", "unistackbot_controller",
        "fk_tool", "--joints", joint_args], capture_output=True, text=True)
    mf = re.search(r"position \[([-\d.e+]+), ([-\d.e+]+), ([-\d.e+]+)\]", fk.stdout)
    if not mf:
        results.append(("FAIL", trial + 1, f"fk_tool 无输出: {fk.stdout[:100]}"))
        continue
    fx, fy, fz = map(float, mf.groups())

    err = max(abs(tx - fx), abs(ty - fy), abs(tz - fz))
    ok = err < 1e-9
    results.append(("PASS" if ok else "FAIL", trial + 1, f"err={err:.2e} tf=({tx:.6f},{ty:.6f},{tz:.6f}) fk=({fx:.6f},{fy:.6f},{fz:.6f})"))

for r in results:
    print(f"{r[0]} #{r[1]}: {r[2]}")
PYEOF
)
echo "$RESULTS"
NP=$(echo "$RESULTS" | grep -c "^PASS")
NF=$(echo "$RESULTS" | grep -c "^FAIL")
PASS=$((PASS + NP)); FAIL=$((FAIL + NF))

kill $LPID 2>/dev/null; wait $LPID 2>/dev/null
ros2 run unistackbot_gazebo gz_clean.sh >/dev/null 2>&1

say "结果: PASS=$PASS FAIL=$FAIL (容差 1e-9)"
[ "$FAIL" -eq 0 ]

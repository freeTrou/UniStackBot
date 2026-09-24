#!/usr/bin/env bash
# UniStackBot 三链 (mock/gz/mujoco) 通用清场 —— 每次起链前 / 链收场后调用这一个脚本。
# (仿 gz_clean.sh; gz 家族模式与 unistackbot_sim_control/gazebo/scripts/gz_clean.sh 镜像,
#  改一处记得同步另一处)
#
# 模式安全规则 (2026-09-24 实锤教训: 手打 pgrep "spawner" 误杀系统进程 gvfsd-*;
#   同日实锤第二课: 与 launch 链在同一命令行执行 → pkill "ros2 launch" 杀死宿主 shell):
#   ① pkill/pgrep 一律配 [x] 方括号: 防交互 shell 的命令行字面量命中模式;
#   ② 凡 -f 模式必须路径锚定/带前缀: 裸词会撞系统进程参数 (gvfsd --spawner);
#      demo 用 "lib/unistackbot_demo" 锚定安装路径 —— 编辑器开 src/ 下同名文件不误伤;
#   ③ 唯一且 ≤15 字符的名字用 comm 精确匹配 (内核截断 comm 到 15 字符,
#      超长名如 robot_state_publisher 的 -x 全名匹配不上, 必须走 -f);
#   ④ **祖先链保护**: 绝不杀自己的祖先进程 —— 链式调用 (chain_clean.sh && ros2 launch ...)
#      时宿主命令行含 "ros2 launch" 字样, 按模式该杀, 但它是"即将执行的调用者",
#      跳过并告警 (仍建议单独执行, 被跳过的宿主可能是别的命令);
#   ⑤ 杀完复查并给 exit code: 0=干净可起链 (祖先除外), 1=仍有残留 (可作流程门禁)。
#
# 用法 (建议单独执行; 链式调用已由 ④ 兜底不死, 但会跳过含匹配字样的宿主):
#   ros2 run unistackbot_bringup chain_clean.sh
# 注意: 会杀掉本机所有链相关进程 (含其他终端里挂着的 ros2 control/topic/bag CLI),
#   并停止 ros2 daemon (清 DDS 陈旧参与者缓存); 单链开发机可放心使用。
#   rviz2 故意不在清单 —— 只读观察者, 不毒化链路。

# 祖先链 (含自身): 任何匹配模式的进程若在此链上则跳过 (④)
ANCESTORS=" $$"
_p=$PPID
while [ "$_p" -gt 1 ] 2>/dev/null; do
	ANCESTORS="$ANCESTORS $_p"
	_p=$(awk '/^PPid:/{print $2}' "/proc/$_p/status" 2>/dev/null)
	case " $ANCESTORS " in *" $_p "*) break;; esac   # 环防护
done
is_ancestor()
{
	case "$ANCESTORS" in *" $1 "*) return 0;; *) return 1;; esac
}
kill_regex()   # kill_regex <ERE>: -f 全命令行匹配, 排除祖先链
{
	for _pid in $(pgrep -f "$1"); do
		if is_ancestor "$_pid"; then
			echo "chain_clean: 跳过祖先进程 $_pid (命令行含模式 '$1' —— 与本脚本链在同一命令? 建议单独执行)" >&2
			continue
		fi
		kill -9 "$_pid" 2>/dev/null
	done
}
kill_comm()    # kill_comm <name>: comm 精确匹配, 排除祖先链
{
	for _pid in $(pgrep -x "$1"); do
		is_ancestor "$_pid" && continue
		kill -9 "$_pid" 2>/dev/null
	done
}
left_regex()   # left_regex <ERE>: 复查用, 排除祖先链
{
	pgrep -af "$1" | while IFS= read -r _l; do
		_pid=${_l%% *}
		is_ancestor "$_pid" || echo "$_l"
	done
}

# ① launch 宿主 (先杀它, spawner 随进程组收场)
kill_regex "ros2 launc[h]"

# ② 三链 ROS 侧节点
kill_regex "ros2_control_nod[e]"          # mock 链 CM / mujoco 定制 CM 宿主 (同名可执行)
kill_regex "robot_state_publishe[r]"      # 三链 rsp (孤儿抢答 robot_description = 分裂脑)
kill_regex "controller_manager/spawne[r]" # 控制器 spawner (路径锚定, 排除 gvfsd --spawner)
kill_regex "rsp_params.yam[l]"            # rsp 参数文件形态
kill_comm  "supervisor_node"              # 编排层诊断节点 (comm 恰 15 字符, 精确匹配)
kill_regex "sim_control_gz_nod[e]"        # gz /sim_control 适配器 (残留占 DDS 参与者)
kill_regex "sim_control_mujoco_nod[e]"    # mujoco /sim_control 适配器
kill_regex "lib/unistackbot_dem[o]"       # 三个 demo (安装路径锚定)

# ③ gz 家族 (镜像 gz_clean.sh)
kill_regex "ign[-]gazebo"
kill_regex "ros_gz_sim/creat[e]"
kill_regex "parameter_bridg[e]"
kill_comm "ruby"
kill_comm "gzserver"
kill_comm "gzclient"

# ④ ros2 CLI 残留 (挂死的 control/topic/bag 各占一个 DDS 参与者; CM 死后常挂)
kill_regex "ros2 contro[l]"
kill_regex "ros2 topic ech[o]"
kill_regex "ros2 bag recor[d]"

# ⑤ ros2 daemon: 先礼后兵 (陈旧缓存会假造"服务不可达", 单链复测前必停)
ros2 daemon stop >/dev/null 2>&1
kill_regex "ros2cli.daemon.daemoniz[e]"

sleep 2

# ⑥ 复查 (同款模式 + gz 精确 comm; 祖先链除外; rviz2 不查)
CHECK="ros2 launc[h]|ros2_control_nod[e]|robot_state_publishe[r]|controller_manager/spawne[r]|rsp_params.yam[l]|sim_control_gz_nod[e]|sim_control_mujoco_nod[e]|lib/unistackbot_dem[o]|ign[-]gazebo|ros_gz_sim/creat[e]|parameter_bridg[e]|ros2 contro[l]|ros2 topic ech[o]|ros2 bag recor[d]|ros2cli.daemon.daemoniz[e]"
LEFT=$({ left_regex "$CHECK"; pgrep -x supervisor_node; pgrep -x ruby; pgrep -x gzserver; pgrep -x gzclient; } 2>/dev/null)

if [ -z "$LEFT" ]; then
	echo "chain_clean: 已清理干净 (三链节点 + gz 家族 + ros2 CLI + daemon), 可以起链"
	exit 0
fi
echo "chain_clean: 仍有 $(echo "$LEFT" | wc -l) 个残留进程:" >&2
echo "$LEFT" >&2
exit 1

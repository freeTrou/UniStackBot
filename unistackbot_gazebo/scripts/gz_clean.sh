#!/usr/bin/env bash
# 清理 Gazebo / ros2_control 链路的全部残留进程。
# ign 服务进程经常在 launch 退出后残留, 会用旧 controller_manager 毒化下一次启动
# (症状: Controller already loaded / 拿到旧 robot_description / 仿真起不来)。
#
# 注意一律用 pkill -f + 方括号技巧匹配: 很多进程名 (robot_state_pub,
# ros2_control_nod, ign-gazebo-serv) 被 comm 截断到 15 字符, `pkill -x 全名`
# 永远匹配不上; 方括号 [x] 则防止 pkill 匹配到自己的命令行。
#
# 用法: 每次启动仿真前单独执行一次 (不要与 launch 命令粘贴在同一行执行)
#   ros2 run unistackbot_gazebo gz_clean.sh
# 注意: 会杀掉本机所有 ros2 launch / Gazebo 实例, 单仿真开发机可放心使用。

pkill -9 -f "ros2 launc[h]" 2>/dev/null
pkill -9 -f "ign[-]gazebo" 2>/dev/null
pkill -9 -x ruby 2>/dev/null
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x gzclient 2>/dev/null
pkill -9 -f "ros2_control_nod[e]" 2>/dev/null
pkill -9 -f "robot_state_publishe[r]" 2>/dev/null
pkill -9 -f "rsp_params.yam[l]" 2>/dev/null
pkill -9 -f "parameter_bridg[e]" 2>/dev/null
pkill -9 -f "controller_manager/spawne[r]" 2>/dev/null
pkill -9 -f "ros_gz_sim/creat[e]" 2>/dev/null
sleep 1

LEFT=$(pgrep -af "ign[-]gazebo|gzserve[r]|ros2_control_nod[e]|robot_state_publishe[r]" | wc -l)
if [ "$LEFT" -eq 0 ]; then
    echo "gz_clean: 已清理干净, 可以启动仿真"
else
    echo "gz_clean: 仍有 $LEFT 个残留进程:"
    pgrep -af "ign[-]gazebo|gzserve[r]|ros2_control_nod[e]|robot_state_publishe[r]"
fi

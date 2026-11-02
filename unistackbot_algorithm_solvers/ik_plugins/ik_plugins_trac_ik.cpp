/*
 * TracIk 的 pluginlib 注册件 (缝③, 2026-10-26 审核修 F1 拆出) —— 单独 TU:
 * 仅 UNISTACKBOT_BUILD_TRAC_IK=ON 时以条件源并入 ik_solvers 库, 与无 ROS 世界解耦。
 * 头链带出 rclcpp (vendor trac_ik.hpp), 对应 CMake 条件 ament 依赖。
 */
#include "pluginlib/class_list_macros.hpp"

#include "trac_ik_solver/trac_ik_solver.hpp"

PLUGINLIB_EXPORT_CLASS(unistackbot_algorithm::TracIk, unistackbot_algorithm::IkSolver)

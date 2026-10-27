/*
 * IkSolver 实现的 pluginlib 注册库 (ROS-thin, 2026-10-22 缝③) —— 本包第二个
 * 可选子库 (unistackbot_trac_ik 同款组织: 核心库零 ROS 纯度是库级不是包级)。
 *
 * 本 TU = 无 trac_ik 依赖的三件注册 (dls/analytic_piper/analytic_stationlite),
 * 实现符号经核心库同包直链; trac_ik 注册件住 ik_plugins_trac_ik.cpp (条件并入,
 * 2026-10-26 审核修 F1)。yaml 的 ik_solver 值与 xml 的 class name 一一对应
 * (裸短名, 机型 yaml 零改动)。
 */
#include "pluginlib/class_list_macros.hpp"

#include "analytic_piper/analytic_piper.hpp"
#include "analytic_stationlite/analytic_stationlite.hpp"
#include "dls_ik/dls_ik.hpp"

PLUGINLIB_EXPORT_CLASS(unistackbot_algorithm::DlsIk, unistackbot_algorithm::IkSolver)
PLUGINLIB_EXPORT_CLASS(unistackbot_algorithm::AnalyticPiper, unistackbot_algorithm::IkSolver)
PLUGINLIB_EXPORT_CLASS(unistackbot_algorithm::AnalyticStationlite, unistackbot_algorithm::IkSolver)

/*
 * WholeBodySolver 实现的 pluginlib 注册库 (Phase 1, 2026-10-30) —— ik_plugins 同款组织。
 * 算法位 = 接口+示例 (mos 数学不迁移裁决): 本 TU 注册仓内占位/示例实现;
 * 算法团队 HQP 到位后自包 PLUGINLIB_EXPORT_CLASS 即接入, 零仓库改动。
 */
#include "pluginlib/class_list_macros.hpp"

#include "whole_body/dualarm_dls.hpp"

PLUGINLIB_EXPORT_CLASS(DualArmDls, unistackbot_algorithm::WholeBodySolver)

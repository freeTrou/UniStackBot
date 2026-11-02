#ifndef UNISTACKBOT_ALGORITHM__REDUNDANCY_HPP_
#define UNISTACKBOT_ALGORITHM__REDUNDANCY_HPP_

#include <cstdint>

namespace unistackbot_algorithm
{

/*
 * ================== 冗余消解 (7 轴臂特有; 6 轴构型整体忽略本节) ==================
 *
 * 【什么是冗余】任务空间 6 维 (位置 3 + 姿态 3), 7 轴臂有 7 个关节 ——
 * 方程比未知数少一个。后果: 同一个末端位姿对应**无穷多组关节角**。
 * 直观图像: 手的位置和朝向钉死不动, 肘部仍能整圈摆 ("自运动")。
 * 本仓实测过 xarm7 的解曲线: 闭合一圈, 期间 j1+j5 缠绕整圈、j4 只漂 5.7°。
 *
 * 【为什么放在命令帧里】"求 IK" 在 7 轴上是欠定问题, 求解器必须知道
 * "无穷多个解里要哪一个"。这个选择若藏在求解器内部 = 行为不可复现、
 * 不可测试; 显式挂在命令上 = 每条命令自带选解意图。
 * (行业同型做法: xArm 控制器的 ref_angles 种子参数 —— 全行业都在做,
 *  只是各家隐式, 本框架把它做成一等公民。)
 *
 * 【三种选解方式 = 在解曲线上选点的三种策略】
 *
 *   PRESERVE   "停在老地方": 从上一时刻的解出发, 在解曲线上取离它最近
 *              的点。流式跟踪 / 拖动示教用 —— 构型随时间平滑演化,
 *              绝不跳分支 (跳分支 = 关节瞬间大位移, 实机上是急停或更糟)。
 *              默认值: 流式是主导场景, 默认行为 = 连续性最好 = 最安全。
 *
 *   LOCK_JOINT "锁死一个关节": 把第 joint_index 个关节钉在命令值上,
 *              剩 6 个关节解 6 维任务 —— 方程变方阵, 解从无穷多收敛为
 *              有限组。锁定值取本帧 position[joint_index] (旋钮只声明
 *              "锁谁", 值走主载荷通道, 不重复表达)。
 *              用途示例: 把贴限位的关节锁在安全角再解其余 (xarm7 的 j4
 *              家位距下限仅 11°, 正是该场景); 肘部避障时锁肩部姿态。
 *
 *   ARM_ANGLE  "按臂角挑": psi = 肘部绕"肩-腕连线"的摆角 [rad], 一个数
 *              在解曲线上定位一个点。语义最直观 (0 = 肘正前方, ±90° =
 *              肘最侧), 但**仅在肩/腕三轴汇交 (S-R-S 类) 构型上严格成立**;
 *              带偏置构型 (如 xarm7, 实测三处偏置) 上只是近似。
 *              求解器不支持时应报 IkResult 拒因, 不许假装支持。
 *              为公司臂预留: 若机械侧采纳"腕三轴汇交"设计输入
 *              (ik_decision_card §5.4), 此旋钮立刻获得严格语义。
 *
 * 【字段有效性】type 决定哪个参数字段有意义 (同一时刻至多一个):
 *   PRESERVE   -> 两参数均忽略
 *   LOCK_JOINT -> joint_index 有效 (0..joint_count-1); 锁定值 = position[joint_index]
 *   ARM_ANGLE  -> psi 有效 [rad]
 * 失败语义: LOCK_JOINT 的锁定值越限 -> IkResult::LIMIT_CONFLICT。
 *
 * (2026-11-02 自 interface/robot_command.hpp 迁入算法域——IkSolver::solve 的
 *  参数词汇, 契约归契约的主人; 运行面 RobotCommand 经 include 引用本类型。)
 */
enum class RedundancyType : uint8_t
{
	PRESERVE = 0,    // 停在老地方: 取解曲线上离上一解最近的点 (流式连续)
	LOCK_JOINT = 1,  // 锁死一个关节: joint_index 钉在 position[joint_index], 余 6 关节解任务
	ARM_ANGLE = 2,   // 按臂角挑: psi 定位解曲线上的点 (仅 S-R-S 类构型严格)
};

// 冗余偏好 —— POD 形式的带标签联合 (不用 std::variant, 保持定长平凡拷贝, RT 可用)
struct RedundancyPreference
{
	RedundancyType type{RedundancyType::PRESERVE};   // 选解策略 (默认 PRESERVE = 最安全)
	uint8_t joint_index{};    // [LOCK_JOINT] 被锁关节索引; 锁定值 = 同帧 position[joint_index]
	double psi{0.0};          // [ARM_ANGLE] 臂角 [rad], 肘部绕肩-腕连线的摆角
};

}  // namespace unistackbot_algorithm
#endif  // UNISTACKBOT_ALGORITHM__REDUNDANCY_HPP_

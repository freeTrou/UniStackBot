#ifndef UNISTACKBOT_INTERFACE__ROBOT_COMMAND_HPP_
#define UNISTACKBOT_INTERFACE__ROBOT_COMMAND_HPP_

#include <array>
#include <cstdint>
#include <type_traits>

#include "unistackbot_interface/joint_capacity.hpp"

namespace unistackbot_interface
{

/*
 * 指令帧 —— 控制回路里的"控制"一侧 (2026-09-17 命名裁决: 原 JointCmd 更名)。
 * 与 RobotFeedback 对称; 全机器人级 (非单关节), 与 SimCommand 同纪律: 定长 POD。
 *
 * 冗余偏好在接口层显式表达 (IK 决策卡 §5.2, 四个独立证据支撑参数化):
 * 不藏在种子里的隐性魔法, 行为可复现、可测试。
 */
/*
 * 命令模式 —— 词汇对齐 CiA402 周期同步伺服族 (行业专业命名)。
 * mode 决定主载荷数组; 前馈列按 CiA402 对象语义可选携带。
 *
 * 刻意不收录的模式 (边界声明):
 *   - PROFILE_POSITION/VELOCITY/TORQUE (CiA402 0x6060=1/3/4): 驱动器内部轨迹发生器,
 *     与本框架"插值只长在速率/语义边界"的纪律冲突 (插值归 OTG/JTC); 黑盒平台
 *     (vendor SDK) 由此类模式承接, 但对外仍统一翻译为本帧的 CSP 词汇;
 *   - INTERPOLATED_POSITION (0x6060=7): 驱动器侧缓冲插补方案, CSP 已覆盖其用途;
 *   - HOMING (0x6060=6): 是过程/服务 (零点标定), 不是值流模式, 走服务接口。
 */
enum class CmdMode : uint8_t
{
	NONE = 0,        // 无命令: 消费端保持现状 (零初始化帧的安全默认; seq 陈旧看门狗同义)
	CSP = 1,         // 周期同步位置 (CiA402 0x6060=8): 主载荷 position[];
	                 //   velocity[]/effort[] 可作前馈 (对应 v-ff/t-ff 对象)
	CSV = 2,         // 周期同步速度 (0x6060=9): 主载荷 velocity[]; effort[] 可作前馈
	CST = 3,         // 周期同步力矩 (0x6060=10): 主载荷 effort[]
	                 //   (CiA402 称 torque, ROS 称 effort, 同一物理量 N·m)
	MIT = 4,         // MIT 模式 (Mini Cheetah 执行器命令格式, 电机行业通用名):
	                 //   τ = kp·(q_des−q) + kd·(q̇_des−q̇) + τ_ff —— 控制理论即关节阻抗式;
	                 //   主载荷 = position[] + velocity[](前馈) + effort[](前馈) + kp[]/kd[]
	                 //   (协作二期柔顺 / RL policy 输出的标准命令形态)
};

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

struct RobotCommand
{
	uint64_t seq{};                                  // 写次数单调递增
	double stamp_s{};                                // steady_clock 域时戳 (秒)
	CmdMode mode{CmdMode::NONE};                     // 命令模式 (默认无命令 = 零初始化即安全)
	uint32_t joint_count{};                          // 有效关节数 (<= kMaxJoints)
	std::array<uint8_t, kMaxJoints> mask{};          // 关节 i 生效 = 1
	std::array<double, kMaxJoints> position{};       // 目标位置 [rad]
	std::array<double, kMaxJoints> velocity{};       // 目标速度 [rad/s]
	std::array<double, kMaxJoints> effort{};         // 目标力矩 [N·m]
	std::array<double, kMaxJoints> max_delta{};      // 每关节步长限幅 (流式安全; 0 = 不限)
	std::array<double, kMaxJoints> kp{};             // MIT 比例增益 [N·m/rad] (仅 MIT 模式; 0 = 不启用)
	std::array<double, kMaxJoints> kd{};             // MIT 阻尼增益 [N·m·s/rad] (仅 MIT 模式; 0 = 不启用; ki 有 windup 风险不留)
	RedundancyPreference redundancy{};               // 冗余偏好 (非冗余构型忽略)
};
static_assert(std::is_trivially_copyable_v<RobotCommand>, "RobotCommand must be trivially copyable for the lock-free value channel");

}  // namespace unistackbot_interface
#endif  // UNISTACKBOT_INTERFACE__ROBOT_COMMAND_HPP_

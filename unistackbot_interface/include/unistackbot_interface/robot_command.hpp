/*
 * **形态挂账 (2026-10-21 组合模型复审)**: 当前定长 POD + trivially copyable = 为
 * SpLatest 值通道设计的自洽形态 (零消费者)。总线段交换已动态化 (DynamicSpLatest,
 * 装配期定容); 本头**动态化挂账, 触发器 = 第一个真实 RL 契约消费者**——届时按实际
 * 需求定形态 (vector 帧 + 对象版双缓冲交换原语, 非 POD 帧不进现有 seqlock), 与
 * 消费者共同定稿, 不预先抽象 (最小集纪律)。
 */
#ifndef UNISTACKBOT_INTERFACE__ROBOT_COMMAND_HPP_
#define UNISTACKBOT_INTERFACE__ROBOT_COMMAND_HPP_

#include <array>
#include <cstdint>
#include <type_traits>

#include "ik_solver/redundancy.hpp"   // unistackbot_algorithm::RedundancyPreference (2026-11-02 迁算法域, 此处引用)
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
 * 冗余消解词汇 (RedundancyType/RedundancyPreference) 2026-11-02 迁入算法域
 * (unistackbot_algorithm/ik_solver/redundancy.hpp——IkSolver::solve 的参数,
 * 契约归契约的主人); RobotCommand.redundancy 字段经上方 include 引用之。
 * 选解策略的完整文档随类型迁往算法包。
 */
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
	unistackbot_algorithm::RedundancyPreference redundancy{};   // 冗余偏好 (非冗余构型忽略; 类型住算法域)
};
static_assert(std::is_trivially_copyable_v<RobotCommand>, "RobotCommand must be trivially copyable for the lock-free value channel");

}  // namespace unistackbot_interface
#endif  // UNISTACKBOT_INTERFACE__ROBOT_COMMAND_HPP_

#ifndef UNISTACKBOT_INTERFACE__ROBOT_FEEDBACK_HPP_
#define UNISTACKBOT_INTERFACE__ROBOT_FEEDBACK_HPP_

#include <array>
#include <cstdint>
#include <type_traits>

#include "unistackbot_interface/joint_capacity.hpp"

namespace unistackbot_interface
{

/*
 * 反馈帧 —— 控制回路里的"反馈"一侧 (2026-09-17 命名裁决: 原 RobotStateSnapshot 更名)。
 *
 * 分层归属: 设备/驱动层的**状态**住这里 (含故障) —— 电机的"错误"是反馈流的一部分,
 * 随每拍状态进来, 不是任何命令结果枚举的成员。
 *
 * 交换纪律 (对齐设计文档 §3.2, 文档仅作参考):
 *   - 覆盖写 + 取最新 (值通道); seq 按写次数递增, 不按值变化;
 *   - 生死判据: seq 停止递增 = 生产者死亡;
 *   - stamp_s 为 steady_clock 域 (RT 单调钟), 不用 ROS 时间;
 *   - 定长 POD, 无堆分配, RT 路径可平凡拷贝。
 */
// 实际控制模式 (CiA402 0x6061 语义) —— 纯模式词汇, 与 CmdMode 同表:
// 反馈报告"实际在什么模式", 命令声明"要求什么模式"。状态(使能/故障)不在此, 见 DeviceState。
enum class RobotMode : uint8_t
{
	NONE = 0,       // 未承载控制模式
	CSP = 1,        // 周期同步位置 (CiA402 0x6060=8)
	CSV = 2,        // 周期同步速度 (0x6060=9)
	CST = 3,        // 周期同步力矩 (0x6060=10)
	MIT = 4,        // MIT 模式 (Mini Cheetah 执行器命令格式, 即关节阻抗式)
};

// 设备状态 (生命周期/健康) —— 与模式正交的另一维度 (CiA402 statusword 语义的形态盲抽象);
// 硬件层负责把协议状态机聚合映射进来 (臂是刚体链, 取全组最低态)
enum class DeviceState : uint8_t
{
	OFFLINE = 0,     // 不可达 (主站未连/进程亡; 零初始化的安全默认)
	DISABLED = 1,    // 已连接未使能 (CiA402: Switch On Disabled)
	READY = 2,       // 就绪待运行 (Ready To Switch On / Switched On)
	OPERATIONAL = 3, // 运行中 (Operation Enabled)
	FAULT = 4,       // 故障 (细节看 fault_mask; 恢复语义属硬件层)
};

struct RobotFeedback
{
	uint64_t seq{};                                   // 写次数单调递增 (双缓冲生死判据)
	double stamp_s{};                                 // steady_clock 域时戳 (秒)
	uint32_t joint_count{};                           // 有效关节数 (<= kMaxJoints)
	std::array<double, kMaxJoints> position{};        // 关节位置 [rad]
	std::array<double, kMaxJoints> velocity{};        // 关节速度 [rad/s]
	std::array<double, kMaxJoints> effort{};          // 关节力矩 [N·m]
	RobotMode mode{RobotMode::NONE};                  // 实际控制模式 (0x6061 语义)
	DeviceState state{DeviceState::OFFLINE};          // 设备状态 (与模式正交; 零初始化=离线=安全默认)
	uint64_t fault_mask{};                            // 轴 i 故障 = bit i (CiA402 fault 语义的家)
};
static_assert(std::is_trivially_copyable_v<RobotFeedback>, "RobotFeedback must be trivially copyable for the lock-free value channel");

}  // namespace unistackbot_interface
#endif  // UNISTACKBOT_INTERFACE__ROBOT_FEEDBACK_HPP_

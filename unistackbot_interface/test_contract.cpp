/*
 * unistackbot_interface 契约头单元测试 (g++ 直编, common 组件同型, 不进 colcon)。
 * 编译运行:
 *   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wconversion -Iinclude \
 *       test_contract.cpp -o /tmp/test_interface && /tmp/test_interface
 */
#include <cstdint>
#include <cstdio>
#include <type_traits>

#include "unistackbot_interface/ik_result.hpp"
#include "unistackbot_interface/joint_capacity.hpp"
#include "unistackbot_interface/robot_command.hpp"
#include "unistackbot_interface/robot_feedback.hpp"
#include "unistackbot_interface/sim_result.hpp"

using namespace unistackbot_interface;

static int g_fail = 0;
static int g_pass = 0;
#define CHECK(cond) do { if (cond) { ++g_pass; } else { ++g_fail; std::printf("FAIL: %s\n", #cond); } } while (0)

// ---- 编译期断言: POD 纪律 (与 SimCommand 同型) ----
static_assert(std::is_trivially_copyable_v<RobotFeedback>, "RobotFeedback POD");
static_assert(std::is_trivially_copyable_v<RobotCommand>, "RobotCommand POD");
static_assert(std::is_trivially_copyable_v<RedundancyPreference>, "RedundancyPreference POD");

// ---- 编译期断言: 定长数组容量 = 唯一常量 ----
static_assert(RobotFeedback{}.position.size() == kMaxJoints, "feedback capacity");
static_assert(RobotCommand{}.position.size() == kMaxJoints, "command capacity");
static_assert(RobotCommand{}.max_delta.size() == kMaxJoints, "max_delta capacity");

int main()
{
	// ---- 反馈帧: 默认值 + 模式/状态正交 ----
	RobotFeedback fb;
	CHECK(fb.seq == 0);
	CHECK(fb.mode == RobotMode::NONE);        // 未承载模式
	CHECK(fb.state == DeviceState::OFFLINE);  // 零初始化 = 离线 = 安全默认
	CHECK(fb.fault_mask == 0);
	fb.mode = RobotMode::CSP;
	fb.state = DeviceState::OPERATIONAL;      // 两维独立取值, 互不耦合
	fb.state = DeviceState::FAULT;            // 故障是状态, 不是模式
	CHECK(fb.mode == RobotMode::CSP);         // 状态变化不影响模式维度
	CHECK(fb.joint_count <= kMaxJoints);
	fb.position[3] = 1.5;
	RobotFeedback copy = fb;   // 平凡拷贝可用 (无堆/无虚表)
	CHECK(copy.position[3] == 1.5);

	// ---- 指令帧: 默认值与模式词汇 ----
	RobotCommand cmd;
	CHECK(cmd.mode == CmdMode::NONE);   // 零初始化 = 无命令 = 安全默认
	cmd.mode = CmdMode::CSP;            // CiA402 词汇: 周期同步位置
	cmd.mode = CmdMode::MIT;            // 行业词汇: MIT 模式 (关节阻抗式)
	CHECK(cmd.kp[0] == 0.0);            // MIT 增益默认不启用 (纯模式无意义)
	CHECK(cmd.kd[0] == 0.0);
	cmd.kp[3] = 120.0;                  // MIT 模式: 单关节设增益
	cmd.kd[3] = 8.0;
	CHECK(cmd.kp[3] > 119.0 && cmd.kd[3] < 9.0);
	CHECK(cmd.redundancy.type == RedundancyType::PRESERVE);
	CHECK(cmd.redundancy.joint_index == 0);
	cmd.redundancy = {RedundancyType::LOCK_JOINT, 4, 0.0};
	CHECK(cmd.redundancy.joint_index == 4);
	cmd.redundancy = {RedundancyType::ARM_ANGLE, 0, 0.785};
	CHECK(cmd.redundancy.psi > 0.78 && cmd.redundancy.psi < 0.79);

	// ---- IK 结果: 码值互异 + 每码有非空信息 ----
	CHECK(IkResult::OK != IkResult::UNREACHABLE);
	for (uint8_t v = 0; v <= 6; ++v)
	{
		CHECK(ik_result_message(static_cast<IkResult>(v)) != nullptr);
	}
	CHECK(ik_result_message(IkResult::UNREACHABLE)[0] != '\0');

	// ---- 仿真结果: 每码有非空信息 ----
	for (uint8_t v = 0; v <= 5; ++v)
	{
		CHECK(sim_result_message(static_cast<SimResult>(v)) != nullptr);
	}
	CHECK(sim_result_message(SimResult::RESET_DANGEROUS)[0] != '\0');

	// ---- 容量常量: 反馈/指令帧共享 (16 = 现役机型覆盖, 见 joint_capacity.hpp 的账) ----
	CHECK(kMaxJoints == 16);
	CHECK(kMaxJoints >= 9);   // piper 含 mimic 手指

	std::printf("结果: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}

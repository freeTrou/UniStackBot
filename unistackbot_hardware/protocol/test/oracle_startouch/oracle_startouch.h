// oracle_startouch —— startouchlib 达妙 codec 甲骨文 (test-only)。
// 来源: /home/work/project/startouchlib (Apache-2.0) src/startouch/damiao_motor/
// dm_motor_control.cpp 的 pack_mit_control_data / parse_motor_state_data /
// double_to_uint / uint_to_double 与 dm_motor_constants.hpp 的 MOTOR_LIMIT_PARAMS,
// 2026-10-21 提取, 仅改: 去 ROS/类依赖改为自由函数 + 命名去 namespace。
// 逐字节对拍用, 勿入生产编译 (README 记录来源与提取点)。
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace oracle_startouch
{

enum Type
{
	DM4310 = 1,
	DM4340 = 3,
};

struct Limit
{
	double pMax;
	double vMax;
	double tMax;
};

inline constexpr std::array<Limit, 4> kLimits = {
	Limit{12.5, 50, 5},    // DM3507 (index 0, 未用)
	Limit{12.5, 30, 10},   // DM4310
	Limit{12.5, 50, 10},   // DM4310_48V (index 2, 未用)
	Limit{12.5, 8, 28},    // DM4340
};

struct MitParam
{
	double kp;
	double kd;
	double q;
	double dq;
	double tau;
};

struct StateResult
{
	double position;
	double velocity;
	double torque;
	int t_mos;
	int t_rotor;
	int feedback_id;
	int error_code;
	bool valid;
};

double limit_min_max(double x, double min, double max);
std::uint16_t double_to_uint(double x, double x_min, double x_max, int bits);
double uint_to_double(std::uint16_t x, double min, double max, int bits);
std::vector<std::uint8_t> pack_mit(int type, const MitParam &p);
StateResult parse_state(int type, const std::vector<std::uint8_t> &data);

}  // namespace oracle_startouch

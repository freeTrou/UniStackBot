// 甲骨文实现 —— 逐行保持 startouchlib 语义 (提取点见 .h 注释), 改动即失甲骨文资格。
#include "oracle_startouch.h"

#include <algorithm>
#include <cmath>

namespace oracle_startouch
{

double limit_min_max(double x, double min, double max)
{
	return std::max(min, std::min(x, max));
}

std::uint16_t double_to_uint(double x, double x_min, double x_max, int bits)
{
	x = limit_min_max(x, x_min, x_max);
	double span = x_max - x_min;
	double data_norm = (x - x_min) / span;
	return static_cast<std::uint16_t>(data_norm * ((1 << bits) - 1));
}

double uint_to_double(std::uint16_t x, double min, double max, int bits)
{
	double span = max - min;
	double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
	return data_norm * span + min;
}

std::vector<std::uint8_t> pack_mit(int type, const MitParam &p)
{
	std::uint16_t kp_uint = double_to_uint(p.kp, 0, 500, 12);
	std::uint16_t kd_uint = double_to_uint(p.kd, 0, 5, 12);
	Limit limits = kLimits[static_cast<std::size_t>(type)];
	std::uint16_t q_uint = double_to_uint(p.q, -limits.pMax, limits.pMax, 16);
	std::uint16_t dq_uint = double_to_uint(p.dq, -limits.vMax, limits.vMax, 12);
	std::uint16_t tau_uint = double_to_uint(p.tau, -limits.tMax, limits.tMax, 12);
	return {static_cast<std::uint8_t>((q_uint >> 8) & 0xFF),
			static_cast<std::uint8_t>(q_uint & 0xFF),
			static_cast<std::uint8_t>(dq_uint >> 4),
			static_cast<std::uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
			static_cast<std::uint8_t>(kp_uint & 0xFF),
			static_cast<std::uint8_t>(kd_uint >> 4),
			static_cast<std::uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
			static_cast<std::uint8_t>(tau_uint & 0xFF)};
}

StateResult parse_state(int type, const std::vector<std::uint8_t> &data)
{
	if (data.size() < 8)
	{
		return {0, 0, 0, 0, 0, 0, 0, false};
	}
	const std::uint8_t status0 = data[0];
	const int feedback_id = static_cast<int>(status0 & 0x0F);
	const int error_code = static_cast<int>((status0 >> 4) & 0x0F);
	std::uint16_t q_uint = (static_cast<std::uint16_t>(data[1]) << 8) | data[2];
	std::uint16_t dq_uint = (static_cast<std::uint16_t>(data[3]) << 4) | (static_cast<std::uint16_t>(data[4]) >> 4);
	std::uint16_t tau_uint = (static_cast<std::uint16_t>(data[4] & 0xf) << 8) | data[5];
	int t_mos = static_cast<int>(data[6]);
	int t_rotor = static_cast<int>(data[7]);
	Limit limits = kLimits[static_cast<std::size_t>(type)];
	double recv_q = uint_to_double(q_uint, -limits.pMax, limits.pMax, 16);
	double recv_dq = uint_to_double(dq_uint, -limits.vMax, limits.vMax, 12);
	double recv_tau = uint_to_double(tau_uint, -limits.tMax, limits.tMax, 12);
	return {recv_q, recv_dq, recv_tau, t_mos, t_rotor, feedback_id, error_code, true};
}

}  // namespace oracle_startouch

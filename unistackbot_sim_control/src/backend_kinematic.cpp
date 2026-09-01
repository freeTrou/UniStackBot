#include "unistackbot_sim_control/backend_kinematic.hpp"

#include <algorithm>
#include <cmath>

namespace unistackbot_sim_control
{

bool BackendKinematic::init(const std::vector<JointMeta> & joints, std::string & message)
{
	joints_ = joints;
	message = "kinematic backend initialized with " + std::to_string(joints_.size()) + " joints";
	return true;
}

const std::string & BackendKinematic::name() const
{
	static const std::string kName = "kinematic";
	return kName;
}

void BackendKinematic::step(
	const std::vector<double> & cmd_position,
	std::vector<double> & state_position,
	std::vector<double> & state_velocity,
	double dt, bool integrate)
{
	const size_t n = state_position.size();
	for (size_t i = 0; i < n; ++i) {
		const JointMeta & joint = joints_[i];

		// mimic 关节: 源关节按 multiplier/offset 推导, 无积分
		if (joint.is_mimic()) {
			const size_t src = static_cast<size_t>(joint.mimic_source);
			state_position[i] = joint.mimic_multiplier * state_position[src] + joint.mimic_offset;
			state_velocity[i] = joint.mimic_multiplier * state_velocity[src];
			continue;
		}

		if (!integrate) {
			continue;
		}

		// 理想执行器: 限位 clamp + 最大角速度饱和的一阶逼近
		const double target = std::clamp(cmd_position[i], joint.min, joint.max);
		const double error = target - state_position[i];
		const double max_step = joint.max_velocity * dt;
		const double step_delta =
			(std::abs(error) <= max_step) ? error : std::copysign(max_step, error);

		state_position[i] = std::clamp(state_position[i] + step_delta, joint.min, joint.max);
		state_velocity[i] = (dt > 0.0) ? step_delta / dt : 0.0;
	}
}

}  // namespace unistackbot_sim_control

#ifndef UNISTACKBOT_SIM_CONTROL__BACKEND_KINEMATIC_HPP_
#define UNISTACKBOT_SIM_CONTROL__BACKEND_KINEMATIC_HPP_

#include <string>
#include <vector>

#include "unistackbot_sim_control/backend.hpp"

namespace unistackbot_sim_control
{

/// 运动学后端 (原 mock 逻辑): 理想执行器模型。
/// 独立关节 = 速度受限一阶逼近 (限位 clamp + 最大角速度饱和);
/// mimic 关节 = 源关节按 multiplier/offset 推导 (无积分)。
class BackendKinematic : public unistackbot_sim_control::SimBackend
{
public:
	bool init(const std::vector<unistackbot_sim_control::JointMeta> & joints,
		std::string & message) override;

	const std::string & name() const override;

	void step(const std::vector<double> & cmd_position,
		std::vector<double> & state_position,
		std::vector<double> & state_velocity,
		double dt, bool integrate) override;

private:
	std::vector<unistackbot_sim_control::JointMeta> joints_;
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__BACKEND_KINEMATIC_HPP_

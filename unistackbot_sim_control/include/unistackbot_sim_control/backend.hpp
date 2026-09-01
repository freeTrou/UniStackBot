#ifndef UNISTACKBOT_SIM_CONTROL__BACKEND_HPP_
#define UNISTACKBOT_SIM_CONTROL__BACKEND_HPP_

#include <string>
#include <vector>

namespace unistackbot_sim_control
{

/// 关节元数据: 由插件从 <ros2_control> 块解析, 所有后端共用
struct JointMeta
{
	std::string name;
	double min{0.0};
	double max{0.0};
	double max_velocity{0.0};        ///< rad/s, 独立关节的执行器速度上限
	int mimic_source{-1};            ///< -1 = 独立关节; 否则源关节索引
	double mimic_multiplier{1.0};
	double mimic_offset{0.0};

	bool is_mimic() const {return mimic_source >= 0;}
};

/// 仿真后端接口: 统一仿真控制层的"对内分类"。
/// 新仿真平台接入 = 实现本接口 + 在插件工厂注册一个名字。
class SimBackend
{
public:
	virtual ~SimBackend() = default;

	/// 后端初始化 (限位/mimic 等元数据由插件解析好传入)
	virtual bool init(const std::vector<JointMeta> & joints, std::string & message) = 0;

	virtual const std::string & name() const = 0;

	/// 推进仿真一个控制周期。
	/// integrate=false 时只刷新派生量 (mimic 关节), 用于 pause 冻结与状态瞬移后的同步。
	virtual void step(
		const std::vector<double> & cmd_position,
		std::vector<double> & state_position,
		std::vector<double> & state_velocity,
		double dt, bool integrate) = 0;
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__BACKEND_HPP_

#ifndef UNISTACKBOT_SIM_CONTROL__SIM_BACKEND_FACTORY_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_BACKEND_FACTORY_HPP_

#include <memory>
#include <string>

#include "unistackbot_sim_control/backend_kinematic.hpp"
#include "unistackbot_sim_control/backend_threaded.hpp"
#include "unistackbot_sim_control/sim_backend.hpp"

namespace unistackbot_sim_control
{

/*
 * 后端工厂: 全工程唯一认识具体后端类的地方 (宿主只 include 本文件, 不 include 任何后端)。
 * 新仿真接入 = 实现一个 SimBackend 子类 + 在此登记一行。
 * 候选 ≤3 时 if-chain 最直白; 超过后换"名字→构造"注册表, 只动本文件。
 */
[[nodiscard]] inline std::unique_ptr<SimBackend> createBackend(const std::string & name, std::string & message)
{
	if (name == "kinematic")
	{
		return std::make_unique<BackendKinematic>();
	}
	if (name == "threaded")
	{
		return std::make_unique<BackendThreaded>();
	}
	// if (name == "mujoco")    { return std::make_unique<BackendMujoco>(); }     // 未来: MuJoCo 引擎
	message = "unknown backend '" + name + "' (supported: kinematic, threaded)";
	return nullptr;
}

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_BACKEND_FACTORY_HPP_

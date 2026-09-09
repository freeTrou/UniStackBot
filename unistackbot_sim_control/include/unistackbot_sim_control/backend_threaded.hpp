#ifndef UNISTACKBOT_SIM_CONTROL__BACKEND_THREADED_HPP_
#define UNISTACKBOT_SIM_CONTROL__BACKEND_THREADED_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "sp_latest/sp_latest.hpp"

#include "unistackbot_sim_control/backend_kinematic.hpp"
#include "unistackbot_sim_control/sim_backend.hpp"
#include "unistackbot_sim_control/sim_command_queue.hpp"

namespace unistackbot_sim_control
{

using unistackbot_common::SpLatest;   // 交换原语已上收 common, 保持非限定用法

// plant 交换 POD (值通道载荷, 可平凡拷贝)
struct PlantSnapshot
{
	uint32_t count{0};
	std::array<double, kMaxJoints> position{};
	std::array<double, kMaxJoints> velocity{};   // 容量引用本包统一上限 kMaxJoints (sim_command_queue.hpp)
};
static_assert(std::is_trivially_copyable_v<PlantSnapshot>, "PlantSnapshot must be trivially copyable");

/*
 * 异步 plant 后端: 仿真物理在自带节拍 (1kHz) 的独立线程里推进,
 * 宿主 transmit() 下发命令 / step() 取最新快照 —— 与真机"发帧/收帧"同构。
 * 积分数学组合复用 BackendKinematic (不重写); pause/单步经原子门控下发 plant。
 */
class BackendThreaded : public SimBackend
{
public:
	[[nodiscard]] bool init(const std::vector<JointMeta> & joints, std::string & message) override;
	const std::string & name() const override;
	[[nodiscard]] bool activate() override;
	void deactivate() override;
	void transmit(const std::vector<double> & cmd_position) override;
	void requestState(const std::vector<double> & state_position, const std::vector<double> & state_velocity) override;
	void step(const std::vector<double> & cmd_position, std::vector<double> & state_position,
		std::vector<double> & state_velocity, double dt, bool integrate) override;

private:
	//plant 线程主循环 (绝对时间节拍; 退出由 running_ 门控, join 至多 1 拍)
	void plantLoop();

	BackendKinematic core_;         // 积分数学 (同步后端同款, 组合复用)
	uint32_t count_{0};             // 关节数快照 (init 后只读)
	std::vector<double> core_cmd_;  // plant 线程私有工作数组 (仅 plant 线程触碰)
	std::vector<double> core_pos_;
	std::vector<double> core_vel_;

	SpLatest<PlantSnapshot> cmd_latest_;       // 宿主→plant 最新命令 (值通道)
	SpLatest<PlantSnapshot> state_latest_;     // plant→宿主 最新状态快照
	SpLatest<PlantSnapshot> override_latest_;  // 宿主→plant 状态直写 (瞬移/回零)
	std::atomic<bool> running_{false};       // plant 线程退出旗标
	std::atomic<bool> freeze_{false};        // true = 冻结推进 (pause; mimic 仍刷)
	std::atomic<uint32_t> step_tokens_{0};   // 暂停下单步请求 (plant 拍粒度)
	std::thread plant_thread_;
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__BACKEND_THREADED_HPP_

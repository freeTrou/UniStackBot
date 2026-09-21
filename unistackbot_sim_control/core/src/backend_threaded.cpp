#include "unistackbot_sim_control/backend_threaded.hpp"

#include <chrono>

namespace unistackbot_sim_control
{

namespace
{

constexpr int kPlantRateHz = 1000;   // plant 自带节拍 (真机伺服环同量级)
constexpr double kPlantDt = 0.001;   // = 1 / kPlantRateHz

}  // namespace

bool BackendThreaded::init(const std::vector<JointMeta> & joints, std::string & message)
{
	count_ = static_cast<uint32_t>(joints.size());
	core_cmd_.assign(count_, 0.0);
	core_pos_.assign(count_, 0.0);
	core_vel_.assign(count_, 0.0);
	if (!core_.init(joints, message))
	{
		return false;
	}
	message = "threaded backend initialized with " + std::to_string(count_) + " joints (plant @ 1kHz)";
	return true;
}

const std::string & BackendThreaded::name() const
{
	static const std::string kName = "threaded";
	return kName;
}

bool BackendThreaded::activate()
{
	if (plant_thread_.joinable())
	{
		return true;   // 重复 activate 幂等
	}
	running_.store(true);
	freeze_.store(false);
	try
	{
		plant_thread_ = std::thread([this]() { plantLoop(); });
	}
	catch (const std::exception &)   // 边界兜底: 线程资源耗尽会抛 (规范 27)
	{
		running_.store(false);
		return false;
	}
	return true;
}

void BackendThreaded::deactivate()
{
	running_.store(false);
	if (plant_thread_.joinable())
	{
		plant_thread_.join();   // 有界: 循环每拍自查退出旗标, 至多 1 拍
	}
}

void BackendThreaded::transmit(const std::vector<double> & cmd_position)
{
	PlantSnapshot c;
	c.count = count_;
	for (uint32_t i = 0; i < count_; ++i)
	{
		c.position[i] = cmd_position[i];
	}
	cmd_latest_.publish(c);   // 值通道: 覆盖写, 无满/丢概念
}

void BackendThreaded::requestState(const std::vector<double> & state_position, const std::vector<double> & state_velocity)
{
	PlantSnapshot s;
	s.count = count_;
	for (uint32_t i = 0; i < count_; ++i)
	{
		s.position[i] = state_position[i];
		s.velocity[i] = state_velocity[i];
	}
	override_latest_.publish(s);
}

void BackendThreaded::step(const std::vector<double> & cmd_position, std::vector<double> & state_position,
	std::vector<double> & state_velocity, double dt, bool integrate)
{
	(void)cmd_position;   // 命令经 transmit() 发布
	(void)dt;             // 节拍归 plant 自带时钟
	freeze_.store(!integrate, std::memory_order_relaxed);

	// 取最新快照; 无快照(尚无数据)时保持现值。
	// 注意: 状态直写 (requestState) 会在 1~2 个宿主周期内经 plant 生效, 此间的旧快照被新快照覆盖
	PlantSnapshot s;
	uint64_t seq = 0;
	if (state_latest_.read(s, seq))
	{
		for (uint32_t i = 0; i < s.count && i < count_; ++i)
		{
			state_position[i] = s.position[i];
			state_velocity[i] = s.velocity[i];
		}
	}
}

void BackendThreaded::plantLoop()
{
	const auto period = std::chrono::milliseconds(1000 / kPlantRateHz);
	auto next = std::chrono::steady_clock::now();
	while (running_.load(std::memory_order_relaxed))
	{
		next += period;

		// 1. 取最新命令 (值通道: 单次 read 即最新, 无排空概念)
		PlantSnapshot c;
		uint64_t cmd_seq = 0;
		if (cmd_latest_.read(c, cmd_seq))
		{
			for (uint32_t i = 0; i < c.count && i < count_; ++i)
			{
				core_cmd_[i] = c.position[i];
			}
		}

		// 2. 应用状态直写 (瞬移/回零, 取最新一次; mimic 派生量由随后的 step 刷新)
		PlantSnapshot o;
		uint64_t ovr_seq = 0;
		if (override_latest_.read(o, ovr_seq))
		{
			for (uint32_t i = 0; i < o.count && i < count_; ++i)
			{
				core_pos_[i] = o.position[i];
				core_vel_[i] = o.velocity[i];
			}
		}

		// 3. 推进或冻结 (冻结时 mimic 派生量仍刷新)
		if (!freeze_.load(std::memory_order_relaxed) || step_tokens_.load(std::memory_order_acquire) > 0)
		{
			if (freeze_.load(std::memory_order_relaxed))
			{
				step_tokens_.fetch_sub(1);   // 消费一枚单步令牌
			}
			core_.step(core_cmd_, core_pos_, core_vel_, kPlantDt, true);
		}
		else
		{
			core_.step(core_cmd_, core_pos_, core_vel_, 0.0, false);
		}

		// 4. 发布快照 (覆盖写)
		PlantSnapshot out;
		out.count = count_;
		for (uint32_t i = 0; i < count_; ++i)
		{
			out.position[i] = core_pos_[i];
			out.velocity[i] = core_vel_[i];
		}
		state_latest_.publish(out);

		std::this_thread::sleep_until(next);
	}
}

}  // namespace unistackbot_sim_control

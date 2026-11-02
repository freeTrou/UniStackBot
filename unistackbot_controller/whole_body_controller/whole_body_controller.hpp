#ifndef UNISTACKBOT_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_
#define UNISTACKBOT_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pluginlib/class_loader.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sp_latest/sp_latest.hpp"
#include "std_msgs/msg/string.hpp"
#include "whole_body/whole_body.hpp"

namespace unistackbot_controller
{
/*
 * WholeBodyController (Phase 1, 2026-10-30) —— 双臂全身解算壳 (M6 最小版提前):
 * 解算域 = worker —— 本壳持有 worker 线程按 dt_s 节拍调 WholeBodySolver::solve(),
 * 三条无锁值通道交接 (硬规则③: 解算域是壳的属性, 求解器零线程零锁)。
 *
 * 对外契约 (Phase 2 MosCompat 门面是唯一设计消费者):
 *   ~/command  geometry_msgs/PoseArray —— 2 个位姿按 end_frames 声明序, base_frame 系
 *              (reliable+KeepLast(1), 点流语义: 每条 = 最新目标)
 *
 * 数据路径 (ROS I/O 纪律):
 *   回调线程: PoseArray → GoalsPod → SpLatest<GoalsPod>
 *   RT update: state 接口 → MeasPod → SpLatest<MeasPod>; 最新解 → 安全落地 → command 接口
 *   worker:   最新 meas+goals → solver->solve → SpLatest<MeasPod> (解的 q 段, 同 POD 形态)
 *
 * 安全落地 (裁2 建议口径: 透传+增限+看门狗):
 *   NaN 门 → 限位 clamp (URDF) → 步长饱和 (max_delta_rad/拍);
 *   worker 心跳看门狗: 心跳超时 = 保持最后命令 (解算停供不动作, 恢复自动续供)。
 *
 * 算法位: pluginlib 装载 WholeBodySolver (yaml solver 必填无默认——ik_solver 同款纪律)。
 */
class WholeBodyController : public controller_interface::ControllerInterface
{
public:
	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	// ---- 值通道载荷 (POD, sp_latest seqlock 宣告式要求 trivially copyable) ----
	static constexpr size_t kMaxJointsWb = 32;   // MeasPod 定容上限 (装配期校验 n ≤ 此值)

	struct GoalsPod
	{
		unistackbot_algorithm::CartesianPose l;
		unistackbot_algorithm::CartesianPose r;
		uint64_t stamp_ms{0};
	};
	struct MeasPod
	{
		double q[kMaxJointsWb] = {0};
		double dq[kMaxJointsWb] = {0};
		size_t n{0};
	};

	// worker 线程体 (节拍 = dt_s, sleep_until 周期)
	void workerLoop();

	// 命令消毒: NaN 门 → 限位 clamp → 步长饱和; 返回 false = NaN (整拍保持)
	bool sanitize(std::vector<double> & q);

	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr command_sub_;

	std::unique_ptr<pluginlib::ClassLoader<unistackbot_algorithm::WholeBodySolver>> solver_loader_;
	std::shared_ptr<unistackbot_algorithm::WholeBodySolver> solver_;   // ClassLoader 返回形态

	// 三条值通道 (装配期 init 定容一次)
	unistackbot_common::SpLatest<GoalsPod> goals_ch_;
	unistackbot_common::SpLatest<MeasPod> meas_ch_;
	unistackbot_common::SpLatest<MeasPod> sol_ch_;

	// worker
	std::thread worker_;
	std::atomic<bool> worker_running_{false};
	std::atomic<uint64_t> worker_stamp_ms_{0};   // 心跳 (RT 看门狗判据)
	double dt_s_{0.002};

	// 装配态
	std::vector<std::string> joint_names_;
	std::vector<double> q_min_, q_max_;
	std::vector<std::string> end_frames_;
	std::string base_frame_;
	double max_delta_rad_{0.01};   // 步长上限/拍 (安全落地增限; 定稿入 vendor 参数面)

	// RT scratch (activate 期定容, update 零分配)
	std::vector<double> cmd_last_;   // 最后下发命令 (看门狗/消毒失败的保持值)
	std::vector<double> cmd_try_;
	std::vector<int> idx_q_, idx_v_, idx_cmd_;   // 激活期按名映射的接口索引 (RT 纯索引)
	bool have_last_{false};

	// ~/status (RealtimePublisher trylock, 20Hz 节流)
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
	std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> rt_status_pub_;
	uint64_t status_count_{0};

	// worker scratch (worker 线程私有)
	unistackbot_algorithm::WholeBodyInput wb_in_;
	unistackbot_algorithm::WholeBodyOutput wb_out_;
	MeasPod sol_tx_;
	MeasPod meas_rx_;
	uint64_t meas_seq_{0};
	uint64_t goals_seq_{0};
	GoalsPod goals_rx_;
};

}  // namespace unistackbot_controller

#endif  // UNISTACKBOT_CONTROLLER__WHOLE_BODY_CONTROLLER_HPP_

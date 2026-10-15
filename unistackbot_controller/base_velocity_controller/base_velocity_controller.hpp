#ifndef UNISTACKBOT_CONTROLLER__BASE_VELOCITY_CONTROLLER_HPP_
#define UNISTACKBOT_CONTROLLER__BASE_VELOCITY_CONTROLLER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_latest/sp_latest.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "base_kinematics/base_kinematics.hpp"
#include "ruckig/otg_stream.hpp"

namespace unistackbot_controller
{
/*
 * BaseVelocityController (2026-10-15, 底盘域 A4) —— 底盘速度域控制器 (四件式第 4 件)。
 *
 * 对外契约 (生态标准零自创, 2026-10-15 定案):
 *   /cmd_vel   geometry_msgs/Twist   (base 系; reliable+KeepLast(1) 点流)
 *   /odom      nav_msgs/Odometry     (@50Hz 默认)
 *   /tf        odom→base_link        (REP-105; RealtimePublisher<TFMessage>, 官方
 *                                     diff_drive 同款先例 —— 控制器发自定义 TF)
 *   轮关节反馈经 JSB 进 /joint_states (本控制器只读 state 接口)
 *
 * 断流语义 (与臂域刻意相反, 契约显式不对称): cmd_vel 静默 > cmd_timeout (默认
 * 0.5s) → 目标零速 → VelocityStream 整形减速停车 —— 臂保持位姿安全, 底盘保持
 * 速度=跑飞。真死由驱动器看门狗兜底 (安全终点同款)。
 *
 * odom 真值源 = 控制器积分 (2026-10-15 裁决①: 轮里程计语义、三链统一、真机
 * 同构; 物理链上与引擎真值的差 = 打滑+积分误差 = 真实 odom 漂移, 引擎真值
 * 永远可另取作离线对照, 不冒充进契约)。
 *
 * 管道: 订阅回调 (POD 快照经 SpLatest) → RT 环【超时检测 → VelocityStream<3>
 * 速度整形 (v/a/j 全受限, OtgStream 姊妹件) → BaseKinematics 逆解 → 写
 * position/velocity 命令接口】+ 正解反馈 → odom 积分 → 发布。
 * BaseKinematics 插件住 unistackbot_algorithm (yaml base_kinematics 必填无默认)。
 *
 * 常驻 active (JSB 同生态位); 底盘运动不进 mock 链 (无世界语义, 2026-10-15 裁决)。
 */
class BaseVelocityController : public controller_interface::ControllerInterface
{
public:
	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	// 值通道快照 (POD): 回调线程翻译 Twist → RT 侧零分配 (rosidl 非 POD 不进 SpLatest)
	struct TwistSnapshot
	{
		double vx{0.0};
		double vy{0.0};
		double wz{0.0};
		double pad_[5]{0.0};   // 64 倍数补齐 (slots_[1] 对齐前提, sp_latest 纪律)
	};
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
	unistackbot_common::SpLatest<TwistSnapshot> rt_cmd_;

	// ---- 运动学插件 (on_configure 装配; specs = 接口认领清单) ----
	std::unique_ptr<unistackbot_algorithm::BaseKinematics> kin_;
	bool holonomic_{true};   // 非全向: vy 指令钳零 (形态知识归插件)
	uint64_t dropped_vy_{0};   // 非全向收到非零 vy 的计数 (遥测)
	std::vector<unistackbot_algorithm::BaseActuatorSpec> specs_;
	std::vector<double> cmd_values_;   // 逆解输出 (specs 序)
	std::vector<double> feedback_;     // state 接口实测 (specs 序)

	// ---- 速度整形 (VelocityStream<3>: vx, vy, wz) ----
	unistackbot_common::VelocityStream<3> shaper_;
	double max_lin_x_{0.0}, max_lin_y_{0.0}, max_ang_{0.0};               // 机型资产 (必填>0)
	double max_lin_accel_{0.0}, max_lin_jerk_{0.0}, max_ang_accel_{0.0}, max_ang_jerk_{0.0};
	double cmd_timeout_{0.5};
	bool shaper_ready_{false};
	double calib_dt_{0.005};
	// 断流判活 = SpLatest seq 轮询 (时钟域无关: 控制器节点钟 vs CM 仿真钟不可
	// 互比 —— gz 插件语境 use_sim_time 不保证; stale_watch 血统: seq 不变累计拍数)
	uint64_t last_seq_{0};
	uint32_t age_periods_{0};
	static constexpr uint32_t kMaxAge = 100000u;
	std::array<double, 3> last_target_{0.0, 0.0, 0.0};
	// update_rate 定源 (JS 同款: Humble configure 期 get_update_rate 不可靠 →
	// 激活后首 16 拍中位数校准; 校准前输出零命令)
	static constexpr uint32_t kPeriodSamples = 16u;
	double period_samples_[kPeriodSamples] = {0};
	uint32_t period_n_{0};

	// ---- odom 积分 (控制器积分 = 裁决①) ----
	double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};

	// ---- 发布 (ROS I/O 纪律①: trylock 非阻塞 + 预分配复用载体) ----
	std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_;
	nav_msgs::msg::Odometry odom_msg_;
	std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_;
	tf2_msgs::msg::TFMessage tf_msg_;
	double publish_hz_{50.0};
	uint32_t pub_div_{1};
	uint32_t pub_count_{0};
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__BASE_VELOCITY_CONTROLLER_HPP_

#ifndef UNISTACKBOT_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_
#define UNISTACKBOT_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <type_traits>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rt_tune/rt_tune.hpp"
#include "sp_latest/sp_latest.hpp"
#include "stale_watch/stale_watch.hpp"
#include "unistackbot_interface/joint_capacity.hpp"
#include "unistackbot_interface/msg/cartesian_control.hpp"
#include "unistackbot_interface/msg/cartesian_motion_status.hpp"

#include "dls_ik/dls_ik.hpp"
#include "urdf_fk/urdf_fk.hpp"

// 算法库住 unistackbot_algorithm 包 (2026-09-17 拆分: 算法与控制器集成解耦);
// 控制器只消费类型, 命名空间仍属本包
using unistackbot_algorithm::CartesianPose;
using unistackbot_algorithm::DlsIk;
using unistackbot_algorithm::DlsIkConfig;
using unistackbot_algorithm::DlsIkStats;
using unistackbot_algorithm::RedundancyPreference;
using unistackbot_algorithm::UrdfFk;

namespace unistackbot_controller
{

/*
 * 笛卡尔流式运动控制器 (P1.5) —— IK 算法上环的宿主 (算法=策略层自研,
 * 控制器壳=生态 ControllerInterface 模板)。
 *
 * 对外契约 (冻结于骨架期, 2026-09-17):
 *   ~/target  geometry_msgs/PoseStamped   值通道 (base 系期望末端位姿)
 *   ~/control unistackbot_interface/CartesianControl   事件通道 (TRACKING/HOLD)
 *   ~/status  unistackbot_interface/CartesianMotionStatus   位姿反馈 + 求解器健康
 *   关节反馈不新增: /joint_states (joint_state_broadcaster) 是关节空间单一事实源
 *
 * 与 JTC 的解耦: position 命令接口独占由 controller_manager 强制 (互斥不是纪律
 * 是不可能); 本控制器以 inactive 注册, switch_controllers 切换。JTC 的
 * action/插值/容差在此零使用 —— 流式语义不经过轨迹管线。
 *
 * 骨架版范围 (P1.5 第 1 步): 生命周期 + 接口认领 + 命令保持 (update 透传当前
 * 关节态); IK/限幅/worker/降级在第 2-4 步填充, 对外契约不再变。
 *
 * 实例隔离: KDL 求解器持有迭代暂存成员, 跨线程并发互踩 (urdf_fk.hpp 用法契约)
 * —— update 线程与 worker 各持一套 UrdfFk+DlsIk; 骨架版仅建 update 侧一套。
 */
class CartesianMotionController : public controller_interface::ControllerInterface
{
public:
	[[nodiscard]] controller_interface::CallbackReturn on_init() override;
	[[nodiscard]] controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
	[[nodiscard]] controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

	[[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
	[[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
	// 发布一次状态 (生命周期回调内调用; update 内的周期发布走 RealtimePublisher)
	void publishStatus(uint8_t mode);
	// RT 周期状态发布 (update 内, ~20Hz, tryPublish)
	void publishStatusRt(const rclcpp::Time & time, uint8_t mode,
		const CartesianPose & meas, bool meas_ok);
	// 激活末尾预热: 触达全部首触路径 (缺页/惰性绑定/线程私有 syscall), 见设计 §3
	void warmup();
	// WCET 记账 (每拍; 停用时终报 p50/p99/max)
	void recordWcet(const std::chrono::steady_clock::time_point & t0);

	// IK 解走安全层 (NaN 门 → 限位 clamp → 步长饱和), 两条路径共用 (流式/冷启动回灌)
	bool applySolution(const double * q);
	// worker 线程体 (第 3 步, 防线2): 低优线程跑 COLD_START, 结果经值通道回灌 update
	void workerLoop();

	// ---- 参数 (on_init 声明冻结, 第 2-4 步消费) ----
	std::string base_link_, tip_link_;
	std::string seed_lib_package_, seed_lib_relpath_;
	int64_t update_timeout_ns_{500000};   // update() 内流式求解预算 (防线1)
	int64_t cold_timeout_ns_{50000000};   // worker 冷启动预算 (出环线程不占周期; 无库机型阶梯需数十 ms)
	double max_cart_step_{0.02};          // 单周期笛卡尔步长限幅 [m]
	int degraded_n_{50};                  // 连续失败降级阈值 (第 4 步)
	int ik_max_iterations_{40};           // 流式求解迭代上限 (收敛常态 <5, 40 保险)
	int worker_cpu_{-1};                  // worker 冷启动线程绑核 (-1=不绑; CM 主线程归官方参数)
	int worker_nice_{10};                 // worker nice (让路姿态, 默认 +10)
	int cold_after_fails_{3};             // 流式连续失败 N 拍后请求冷启动
	double converge_pos_tol_{0.001};      // converged 判定: 位置容差 [m]
	double converge_rot_tol_{0.01};       // converged 判定: 姿态容差 [rad]

	// ---- 运动学 (update 线程私有; worker 侧实例第 3 步另建) ----
	std::unique_ptr<UrdfFk> fk_;
	std::unique_ptr<DlsIk> ik_;
	UrdfFk::Scratch fk_scratch_;

	// ---- 接口映射: 接口名序 (CM 声明序) != 链序 (fk.jointNames), 按名映射 ----
	std::vector<std::string> iface_joint_names_;   // state_interfaces_ 顺序的关节名
	std::vector<std::size_t> chain_from_iface_;    // [接口序] -> 链序下标

	// ---- RT 路径状态 (update 线程私有, 零周期分配) ----
	std::vector<double> cmd_;            // 上一拍命令 (链序, IK 种子/连续性根)
	std::vector<double> q_meas_;         // 实测关节 (链序, 误差基准)
	std::vector<double> q_out_;          // IK 解缓冲 (链序)
	std::vector<double> step_limits_;    // [链序] 单拍步长上限 (URDF max_velocity/update_rate)
	bool has_target_{false};
	CartesianPose target_pose_{};        // 当前采纳目标 (base 系)
	uint64_t target_seq_{0};
	int consecutive_fail_{0};            // 连续求解失败计数 (冷启动触发 + 分流停重试判定)
	bool give_up_{false};                // 分流停重试: NEAR_SINGULAR×非超时 = 物理不可解,
	                                     // 目标变化前不再烧流式预算 (2026-09-18 B 案: 无降级状态)
	uint8_t last_result_{0};             // IkResult (status 透传)
	bool last_timed_out_{false};
	double last_min_sigma_{-1.0};
	std::string warned_frame_;           // 非本帧目标的单次警告去重

	// ---- StaleWatch 断流受控减速 (0c; 设计 §6.1 同 JointStream) ----
	// 默认关 (stale_cycles_=0): ~/target 的 --once 单发目标是合法用法, 断流策略
	// 只服务流式跟踪场景 (RL/VLA ingress, 上层连续目标流) —— yaml 显式开启
	double stale_ms_cfg_{0.0};           // stale_timeout_ms 原始配置 (周期数首拍校准)
	double stale_decel_ms_cfg_{200.0};
	uint32_t stale_cycles_{0};           // 断流判定阈值 (周期数; 0=关闭)
	uint32_t stale_decel_cycles_{0};     // 刹停线性减速窗 (周期数)
	unistackbot_common::StaleWatch watch_;
	bool stream_stale_{false};           // 本周期断流态 (status.stream_stale 源)
	bool was_stale_{false};              // 边沿检测 (转换即日志)
	std::vector<double> prev_cmd_;       // 上拍命令快照 (速度估计基准)
	std::vector<double> vel_;            // 上一完整周期的实际关节速度 (链序)
	std::vector<double> decel_rate_;     // 断流进入时刻的每周期速度减量 (= vel/N)
	bool rate_calibrated_{false};        // update_rate 校准 (16 拍中位数; Humble 坑见 cpp)
	static constexpr uint32_t kPeriodSamples = 16u;
	double period_samples_[kPeriodSamples] = {0};
	uint32_t period_n_{0};
	std::vector<double> vmax_;           // URDF max_velocity 原始值 (校准换算基准)

	// ---- WCET 统计 (会话级, 停用时终报; rdtsc 级成本, 每拍两次时钟读取) ----
	std::chrono::steady_clock::time_point update_enter_{};
	double wcet_sum_us_{0.0}, wcet_max_us_{0.0};
	std::vector<double> wcet_samples_;   // 会话样本 (停用时算分位; 500Hz×60s=3万, ~240KB 可接受)
	uint64_t update_count_{0};

	// ---- worker 冷启动 (防线2): 请求/结果都是值通道 (SpLatest POD), 两侧 seq 对账 ----
	// 流式连续失败 cold_after_fails_ 拍 → update 发请求 (目标+实测种子); worker 自建
	// 一套 UrdfFk+DlsIk (KDL 实例不跨线程) 大预算 COLD_START, 结果按请求 seq 回灌 ——
	// 目标已变 (seq 不匹配) 的陈旧结果自然作废
	struct ColdRequest
	{
		CartesianPose target;
		double q[unistackbot_interface::kMaxJoints];
		uint32_t n;
		uint64_t seq;
	};
	struct ColdResult
	{
		double q[unistackbot_interface::kMaxJoints];
		uint32_t n;
		bool ok;
		double min_sigma;
		uint64_t seq;
	};
	static_assert(std::is_trivially_copyable_v<ColdRequest>, "ColdRequest 须 POD (SpLatest 契约)");
	static_assert(std::is_trivially_copyable_v<ColdResult>, "ColdResult 须 POD (SpLatest 契约)");
	std::thread worker_;
	std::atomic<bool> worker_run_{false};
	unistackbot_common::SpLatest<ColdRequest> cold_req_;   // update → worker
	unistackbot_common::SpLatest<ColdResult> cold_res_;    // worker → update
	uint64_t last_cold_req_seq_{0};   // 防重复请求 (同目标只求一次)
	std::string worker_urdf_, worker_lib_;   // worker 线程自建实例的原料 (configure 期快照)

	// ---- 契约通道 (值通道 = sp_latest 自家组件, POD 覆盖写 + seq 变更检测;
	//      事件通道 mode = 原子标志) ----
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
	rclcpp::Subscription<unistackbot_interface::msg::CartesianControl>::SharedPtr control_sub_;
	rclcpp::Publisher<unistackbot_interface::msg::CartesianMotionStatus>::SharedPtr status_pub_;
	std::shared_ptr<realtime_tools::RealtimePublisher<unistackbot_interface::msg::CartesianMotionStatus>> rt_status_; // update 内 trylock 发布
	unistackbot_common::SpLatest<CartesianPose> rt_target_;
	std::atomic<uint8_t> control_mode_{unistackbot_interface::msg::CartesianControl::TRACKING};
	rclcpp::Time last_status_time_{};   // 状态 ~20Hz 节流
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_

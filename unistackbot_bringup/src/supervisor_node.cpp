/*
 * supervisor_node —— 编排层最小版 (0d, 2026-09-21)。
 *
 * 非RT诊断节点 (设计裁决: 诊断通道走 diagnostic_msgs, hardware_framework_design §P):
 * 汇聚链上健康信号 → /supervisor/alerts (diagnostic_msgs/DiagnosticArray, 1Hz)。
 * 纯订阅+定时器, 零服务调用零轮询 —— 任何被监控源死亡都表现为"话题静默", 由
 * 陈旧检测捕获 (编排层不活跃于控制路径, 它死了不影响链)。
 *
 * 监控面 (最小版):
 *   cm            /cartesian_motion_controller/status: mode/stream_stale/last_result/
 *                 timed_out + 话题静默检测 (active 后 >2s 无帧 = WARN)
 *   joint_states  /joint_states: >1s 无帧 = ERROR (链路核心反馈断流)
 *   ee_state      /ee_state_broadcaster/ee_state: >1s 无帧 = WARN (反馈流降级,
 *                 /tf 仍在不算 ERROR)
 *   alerts        汇总帧本身 1Hz —— 消费者用它做 supervisor 活性心跳
 *
 * 后续扩展位 (非本版): write 防线 fault 计数器 (需 sim_control 暴露诊断话题)、
 * 控制器生命周期事件 (需 CM 提供事件或本节点轮询 list_controllers)。
 *
 * 三链通用 (mock/gz/mujoco), 由各链 launch 拉起。
 */

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ulog/ulog.hpp"

#include "unistackbot_interface/msg/cartesian_motion_status.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using unistackbot_interface::msg::CartesianMotionStatus;

namespace
{

constexpr int kLevelOk = DiagnosticStatus::OK;
constexpr int kLevelWarn = DiagnosticStatus::WARN;
constexpr int kLevelError = DiagnosticStatus::ERROR;

// 状态码翻译 (CartesianMotionStatus 常量 → 人话; 消费者也看原始值)
const char * cmModeName(uint8_t mode)
{
	switch (mode)
	{
		case CartesianMotionStatus::INACTIVE: {return "INACTIVE";}
		case CartesianMotionStatus::IDLE: {return "IDLE";}
		case CartesianMotionStatus::TRACKING: {return "TRACKING";}
		case CartesianMotionStatus::HOLD: {return "HOLD";}
		case CartesianMotionStatus::DEGRADED: {return "DEGRADED";}
		default: {return "?";}
	}
}

}  // namespace

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("supervisor");

	unistackbot_common::ulog_config log_cfg;
	log_cfg.console = true;
	log_cfg.file_path = nullptr;
	log_cfg.level = unistackbot_common::ulog_level::info;
	log_cfg.max_bytes = 0;
	log_cfg.backups = 0;
	unistackbot_common::ulog_init(log_cfg);

	// ---- 监控状态 (全部本节点线程私有, 订阅回调与定时器同默认回调组串行) ----
	CartesianMotionStatus cm_last;
	bool cm_seen = false;
	bool cm_was_active = false;   // active 过之后静默才算异常 (未激活时无 status 帧是正常)
	rclcpp::Time cm_last_stamp = node->now();
	rclcpp::Time js_last_time = node->now();
	bool js_seen = false;
	rclcpp::Time ee_last_time = node->now();
	bool ee_seen = false;

	auto sub_cm = node->create_subscription<CartesianMotionStatus>(
		"/cartesian_motion_controller/status", 10,
		[&](const CartesianMotionStatus::SharedPtr m)
		{
			cm_last = *m;
			cm_seen = true;
			if (m->mode != CartesianMotionStatus::INACTIVE)
			{
				cm_was_active = true;
			}
			cm_last_stamp = node->now();
		});
	auto sub_js = node->create_subscription<sensor_msgs::msg::JointState>(
		"/joint_states", 10,
		[&](const sensor_msgs::msg::JointState::SharedPtr)
		{
			js_last_time = node->now();
			js_seen = true;
		});
	auto sub_ee = node->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/ee_state_broadcaster/ee_state", 10,
		[&](const geometry_msgs::msg::PoseStamped::SharedPtr)
		{
			ee_last_time = node->now();
			ee_seen = true;
		});

	auto alerts = node->create_publisher<DiagnosticArray>("/supervisor/alerts", 10);

	const auto kv = [](const std::string & k, const std::string & v)
	{
		KeyValue e;
		e.key = k;
		e.value = v;
		return e;
	};

	// ---- 1Hz 汇总发布 ----
	auto publish = [&]()
	{
		const auto now = node->now();
		DiagnosticArray msg;
		msg.header.stamp = now;

		// cm
		{
			DiagnosticStatus st;
			st.name = "cm";
			st.hardware_id = "cartesian_motion_controller";
			if (!cm_seen)
			{
				st.level = kLevelOk;
				st.message = "未激活 (无 status 帧, CM 以 inactive 注册属正常)";
			}
			else if (cm_was_active && (now - cm_last_stamp).seconds() > 2.0)
			{
				st.level = kLevelWarn;
				st.message = "status 静默 >2s (active 后断流; 上次 mode 见键值)";
			}
			else if (cm_last.stream_stale)
			{
				st.level = kLevelWarn;
				st.message = "目标流断流受控减速中 (stream_stale=true)";
			}
			else if (cm_last.mode == CartesianMotionStatus::DEGRADED)
			{
				st.level = kLevelWarn;
				st.message = "求解连续失败降级";
			}
			else
			{
				st.level = kLevelOk;
				st.message = std::string("mode=") + cmModeName(cm_last.mode);
			}
			st.values.push_back(kv("mode", cmModeName(cm_last.mode)));
			st.values.push_back(kv("last_result", std::to_string(cm_last.last_result)));
			st.values.push_back(kv("timed_out", cm_last.timed_out ? "1" : "0"));
			st.values.push_back(kv("stream_stale", cm_last.stream_stale ? "1" : "0"));
			st.values.push_back(kv("position_error_m", std::to_string(cm_last.position_error)));
			msg.status.push_back(st);
		}

		// joint_states (核心反馈: 断 = ERROR)
		{
			DiagnosticStatus st;
			st.name = "joint_states";
			st.hardware_id = "joint_state_broadcaster";
			const double quiet = js_seen ? (now - js_last_time).seconds() : -1.0;
			if (!js_seen || quiet > 1.0)
			{
				st.level = kLevelError;
				st.message = !js_seen ? "从未收到 /joint_states (链未起?)"
					: "断流 >1s (核心反馈丢失)";
			}
			else
			{
				st.level = kLevelOk;
				st.message = "流正常";
			}
			st.values.push_back(kv("quiet_s", std::to_string(quiet)));
			msg.status.push_back(st);
		}

		// ee_state (反馈流增强: 断 = WARN, /tf 仍在)
		{
			DiagnosticStatus st;
			st.name = "ee_state";
			st.hardware_id = "ee_state_broadcaster";
			const double quiet = ee_seen ? (now - ee_last_time).seconds() : -1.0;
			if (ee_seen && quiet > 1.0)
			{
				st.level = kLevelWarn;
				st.message = "断流 >1s (EE 独立反馈降级; /tf 仍可用)";
			}
			else
			{
				st.level = kLevelOk;
				st.message = ee_seen ? "流正常" : "未见 /ee_state (ee_state_broadcaster 未起?)";
			}
			st.values.push_back(kv("quiet_s", std::to_string(quiet)));
			msg.status.push_back(st);
		}

		alerts->publish(msg);
	};

	auto timer = node->create_wall_timer(std::chrono::seconds(1), publish);
	ULOG_INFO("supervisor 就绪: 监控 cm/joint_states/ee_state → /supervisor/alerts @1Hz");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

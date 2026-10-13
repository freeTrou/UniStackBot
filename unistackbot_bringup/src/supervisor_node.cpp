/*
 * supervisor_node —— 编排层诊断 (0d 2026-09-21; 2026-10-12 升级统一错误契约)。
 *
 * 非RT诊断节点: 汇聚链上健康信号 → /supervisor/alerts (SystemAlert 单板, 50Hz)。
 * 纯订阅+定时器, 零服务调用零轮询 —— 任何被监控源死亡都表现为"话题静默", 由
 * 陈旧检测捕获 (编排层不活跃于控制路径, 它死了不影响链; 自身死亡 = 话题静默,
 * 订阅方 20ms 级发现 —— 心跳语义)。
 *
 * 三错误域 (SystemAlert 三区块; 本节点仿真链只产 SYSTEM 域三源):
 *   DEVICE  关节电机/驱动器 —— 真机批次接入 (协议轴 error_bits 映射, per-joint 槽)
 *   BUS     总线通信        —— 真机批次接入 (serial 遥测 tx_failed/rx_nofit)
 *   SYSTEM  控制器/软件域   —— 本版: cm/js/ee 三源固定字段
 *
 * state 聚合 = worst-of(err_code_severity), 唯一裁决处; js 从未收到 = UNKNOWN
 * (链未起窗口), seen 后断流才升级 (FAULT 级)。人话不进消息 —— 消费端查
 * err_code_message(code) (码表 hpp 单一事实源)。
 *
 * 扩展位 (非本版): write 防线计数器 (SYSTEM 域新字段)、DEVICE/BUS 域真机源、
 * 控制器生命周期事件。
 *
 * 三链通用 (mock/gz/mujoco), 由各链 launch 拉起。
 */

#include <algorithm>
#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ulog/ulog.hpp"

#include "unistackbot_interface/error_codes.hpp"
#include "unistackbot_interface/msg/cartesian_motion_status.hpp"
#include "unistackbot_interface/msg/system_alert.hpp"

using unistackbot_interface::ErrCode;
using unistackbot_interface::msg::CartesianMotionStatus;
using unistackbot_interface::msg::SystemAlert;

namespace
{

constexpr std::chrono::milliseconds kPublishPeriod{20};   // 50Hz 心跳
constexpr double kCmSilentS = 2.0;    // cm status 静默阈值 (active 后)
constexpr double kJsDropS = 1.0;      // /joint_states 断流阈值 (核心反馈)
constexpr double kEeDropS = 1.0;      // /ee_state 断流阈值

// 状态码翻译 (CartesianMotionStatus 常量 → 人话; 仅本节点日志用)
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
	// 异常累计拍数 (按源; 正常拍不清零 —— 语义 = 上电以来异常总量, 趋势可差分)
	uint64_t cm_count = 0, js_count = 0, ee_count = 0;

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
		"/ee_state", 10,
		[&](const geometry_msgs::msg::PoseStamped::SharedPtr)
		{
			ee_last_time = node->now();
			ee_seen = true;
		});

	auto alerts = node->create_publisher<SystemAlert>("/supervisor/alerts", 10);

	// ---- 50Hz 状态板发布 (三源 → SYSTEM 域平行数组 + worst-of state) ----
	auto publish = [&]()
	{
		const auto now = node->now();
		SystemAlert msg;
		msg.header.stamp = now;
		// DEVICE/BUS 域: 仿真链恒空数组 (真机批次接入; 变长模式零写死)
		uint8_t worst = 0;   // worst-of severity: 0=无贡献 1=DEGRADED 级 3=FAULT 级
		const auto pushSystem = [&](const char * loc, ErrCode code, uint64_t & count)
		{
			msg.system_locations.emplace_back(loc);
			msg.system_codes.push_back(static_cast<uint16_t>(code));
			if (code != ErrCode::NONE) {++count;}
			msg.system_counts.push_back(count);
			worst = std::max(worst, unistackbot_interface::err_code_severity(code));
		};

		// cm (DEGRADED/静默 = DEGRADED 级)
		ErrCode cm_code = ErrCode::NONE;
		if (cm_seen && cm_was_active && (now - cm_last_stamp).seconds() > kCmSilentS)
		{
			cm_code = ErrCode::CM_SILENT;
		}
		else if (cm_last.mode == CartesianMotionStatus::DEGRADED)
		{
			cm_code = ErrCode::CM_DEGRADED;
		}
		pushSystem("cm", cm_code, cm_count);

		// joint_states (断流 = FAULT 级 —— 核心反馈丢失)
		ErrCode js_code = ErrCode::NONE;
		if (js_seen && (now - js_last_time).seconds() > kJsDropS)
		{
			js_code = ErrCode::JS_DROPOUT;
		}
		pushSystem("joint_states", js_code, js_count);

		// ee_state (断流 = DEGRADED 级 —— /tf 仍可用)
		ErrCode ee_code = ErrCode::NONE;
		if (ee_seen && (now - ee_last_time).seconds() > kEeDropS)
		{
			ee_code = ErrCode::EE_DROPOUT;
		}
		pushSystem("ee_state", ee_code, ee_count);

		// state 聚合 (唯一裁决处; js 从未收到 = 源数据不足窗口)
		if (!js_seen)
		{
			msg.state = SystemAlert::STATE_UNKNOWN;
		}
		else if (worst >= 3)
		{
			msg.state = SystemAlert::STATE_FAULT;
		}
		else if (worst >= 1)
		{
			msg.state = SystemAlert::STATE_DEGRADED;
		}
		else
		{
			msg.state = SystemAlert::STATE_OK;
		}

		alerts->publish(msg);
	};

	auto timer = node->create_wall_timer(kPublishPeriod, publish);
	ULOG_INFO("supervisor 就绪: 监控 cm/joint_states/ee_state → /supervisor/alerts "
		"@50Hz (SystemAlert 单板: DEVICE/BUS 真机批次接入, SYSTEM=cm/js/ee)");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

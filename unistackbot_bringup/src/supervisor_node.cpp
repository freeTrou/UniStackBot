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
 * 操作态输入 (2026-10-15, 第一期=仿真暂停): 订阅 kPausedTopic (契约住 sim_control,
 * 生产者=各链适配器; mock 缺席=默认运行)。暂停期间静默类裁决整体挂起 (codes 全 0,
 * state=PAUSED) —— "故意停"不是故障, 假 FAULT 会毁板子信用。心跳式: 暂停流断
 * (适配器死) 即回落正常裁决 (误报方向安全)。【挂账】通用操作态框架 (真机软停/
 * 维护模式=第二三个生产者) 待需求落地再定形状 (用户标记待更优想法)。
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
#include "std_msgs/msg/bool.hpp"
#include "ulog/ulog.hpp"

#include "unistackbot_interface/error_codes.hpp"
#include "unistackbot_interface/msg/cartesian_motion_status.hpp"
#include "unistackbot_interface/msg/system_alert.hpp"
#include "unistackbot_sim_control/sim_control_contract.hpp"

using unistackbot_interface::ErrCode;
using unistackbot_interface::msg::CartesianMotionStatus;
using unistackbot_interface::msg::SystemAlert;

namespace
{

constexpr std::chrono::milliseconds kPublishPeriod{20};   // 50Hz 心跳
constexpr double kCmSilentS = 2.0;    // cm status 静默阈值 (active 后)
constexpr double kJsDropS = 1.0;      // /joint_states 断流阈值 (核心反馈)
constexpr double kEeDropS = 1.0;      // /ee_state 断流阈值
constexpr double kPausedStaleS = 1.0;   // 暂停心跳静默阈值 (@10Hz 流, 10 拍余量)

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
	// "当前 active" 闩锁: 随帧翻转而非只置位 (2026-10-15 修)。CM on_deactivate 发
	// 最后一帧 mode=INACTIVE 的 status —— 收到即清闩锁, 之后静默是 inactive 的正常
	// 形态。旧实现只置位不清 → CM 用过一次切回 JS 后 CM_SILENT 误报永驻 (保守告警
	// 变成常态降级, JS↔CM 交替作业的板子永远 DEGRADED)。
	bool cm_was_active = false;
	rclcpp::Time cm_last_stamp = node->now();
	rclcpp::Time js_last_time = node->now();
	bool js_seen = false;
	rclcpp::Time ee_last_time = node->now();
	bool ee_seen = false;
	// 异常累计拍数 (按源; 正常拍不清零 —— 语义 = 上电以来异常总量, 趋势可差分)
	uint64_t cm_count = 0, js_count = 0, ee_count = 0;

	auto sub_cm = node->create_subscription<CartesianMotionStatus>("/cartesian_motion_controller/status", 10, [&](const CartesianMotionStatus::SharedPtr m)
		{
			cm_last = *m;
			cm_seen = true;
			cm_was_active = m->mode != CartesianMotionStatus::INACTIVE;   // 随帧翻转, 见声明处注释
			cm_last_stamp = node->now();
		});
	auto sub_js = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [&](const sensor_msgs::msg::JointState::SharedPtr)
		{
			js_last_time = node->now();
			js_seen = true;
		});
	auto sub_ee = node->create_subscription<geometry_msgs::msg::PoseStamped>("/ee_state", 10, [&](const geometry_msgs::msg::PoseStamped::SharedPtr)
		{
			ee_last_time = node->now();
			ee_seen = true;
		});

	auto alerts = node->create_publisher<SystemAlert>("/supervisor/alerts", 10);

	// ---- 操作态输入: 暂停心跳 (生产者=各链适配器; mock 缺席=默认运行) ----
	bool paused_value = false;
	bool paused_seen = false;
	rclcpp::Time paused_last_time = node->now();
	// 句柄必须持有 (裸 create_* 立即析构 → 订阅静默, 2026-10-15 实锤)
	auto paused_sub = node->create_subscription<std_msgs::msg::Bool>(unistackbot_sim_control::kPausedTopic, 10, [&](const std_msgs::msg::Bool::SharedPtr m)
		{
			paused_value = m->data;
			paused_seen = true;
			paused_last_time = node->now();
		});

	// ---- 50Hz 状态板发布 (三源 → SYSTEM 域平行数组 + worst-of state) ----
	bool was_paused = false;   // 暂停→运行沿: 三源时基重锚 (见下)
	auto publish = [&]()
	{
		const auto now = node->now();
		SystemAlert msg;
		msg.header.stamp = now;

		// 操作态挂起: 暂停心跳有效 (值=true 且未超 kPausedStaleS) = 故意停 ——
		// 静默类裁决整体挂起 (codes 全 0 保持板形), state=PAUSED。心跳断 (适配器
		// 死) 即失效回落正常裁决 —— 宁误报不漏报
		const bool paused = paused_seen && paused_value && (now - paused_last_time).seconds() <= kPausedStaleS;
		if (paused)
		{
			was_paused = true;
			// 板形保持 (三行 code=NONE), counts 显示冻结的累计值 (挂起拍不计数)
			const uint64_t frozen[3] = {cm_count, js_count, ee_count};
			const char * const loc[3] = {"cm", "joint_states", "ee_state"};
			for (int i = 0; i < 3; ++i)
			{
				msg.system_locations.emplace_back(loc[i]);
				msg.system_codes.push_back(static_cast<uint16_t>(ErrCode::NONE));
				msg.system_counts.push_back(frozen[i]);
			}
			msg.state = SystemAlert::STATE_PAUSED;
			alerts->publish(msg);
			return;
		}

		// 暂停→运行沿: 暂停期间三源无帧、时基停在暂停前 —— 恢复瞬间首帧未到时
		// 陈旧差值 = 暂停时长, 会闪 1-2 拍假断流。重锚三源时基到当前时刻,
		// 静默窗从恢复点重新起算 (真实断流 1s 后照常捕获, 不放宽判据)
		if (was_paused)
		{
			was_paused = false;
			cm_last_stamp = now;
			js_last_time = now;
			ee_last_time = now;
		}

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

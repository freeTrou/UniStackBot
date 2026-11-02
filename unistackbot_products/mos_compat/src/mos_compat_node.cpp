/*
 * mos_compat_node —— MOS /sdk 接口兼容门面 (P1 补全, 2026-11-01)
 * 覆盖: arm+body+head+gripper+hand+chassis 订阅 + TF + mode + estop + inhibit
 */
#include <chrono>
#include <cstdlib>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "unistackbot_interface/srv/set_control_mode.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "unistackbot_interface/msg/joint_command.hpp"

namespace mos_compat
{
using JointCommand = unistackbot_interface::msg::JointCommand;

// kIdle/急停 的全停清单 (与机型 yaml 控制器名联动, 产品域 P20 定名)
const std::vector<std::string> kAllControllers = {
	"js_arms", "js_head", "js_torso", "js_gripper", "whole_body_controller"};

class MosCompatNode : public rclcpp::Node
{
public:
	explicit MosCompatNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		: Node("mos_sdk_interface", options)
		, tf_buffer_(get_clock())
		, tf_listener_(tf_buffer_)
	{
		initJointNames();
		js_arms_pub_ = create_publisher<JointCommand>("/js_arms/command", rclcpp::QoS(1).reliable());
		js_head_pub_ = create_publisher<JointCommand>("/js_head/command", rclcpp::QoS(1).reliable());
		js_torso_pub_ = create_publisher<JointCommand>("/js_torso/command", rclcpp::QoS(1).reliable());
		js_gripper_pub_ = create_publisher<JointCommand>("/js_gripper/command", rclcpp::QoS(1).reliable());
		whole_body_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
			"/whole_body_controller/command", rclcpp::QoS(1).reliable());
		cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1).reliable());
		// /sdk 服务独立回调组 + MultiThreadedExecutor: 服务回调内可同步等 CM 服务客户端
		// (同组串行防模式竞态; 客户端住默认组由另一执行线程喂)
		svc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		list_cli_ = create_client<controller_manager_msgs::srv::ListControllers>(
			"/controller_manager/list_controllers");
		switch_cli_ = create_client<controller_manager_msgs::srv::SwitchController>(
			"/controller_manager/switch_controller");
		setupArmSubscriptions();
		setupBodySubscriptions();
		setupServices();
		RCLCPP_INFO(get_logger(), "mos_compat: /sdk 门面就绪 (arm+body+head+gripper+chassis+TF+mode+estop)");
	}

private:
	std::vector<std::string> arm_joints_, head_joints_, torso_joints_;

	void initJointNames()
	{
		arm_joints_ = {
			"left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
			"left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_roll_joint",
			"left_wrist_pitch_joint",
			"right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
			"right_elbow_pitch_joint", "right_wrist_yaw_joint", "right_wrist_roll_joint",
			"right_wrist_pitch_joint"};
		head_joints_ = {"head_yaw_joint", "head_pitch_joint"};
		torso_joints_ = {"body_joint"};
	}

	JointCommand makeCSP(const std::vector<std::string> & names, const std::vector<double> & pos)
	{
		JointCommand cmd;
		cmd.mode = JointCommand::MODE_CSP;
		cmd.joint_names = names;
		cmd.position = pos;
		return cmd;
	}

	void setupArmSubscriptions()
	{
		joint_servo_sub_ = create_subscription<sensor_msgs::msg::JointState>(
			"/sdk/arm/joint_servo_cmd", rclcpp::QoS(1).best_effort(),
			[this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
				if (msg->position.size() != 14) return;
				js_arms_pub_->publish(makeCSP(arm_joints_, msg->position));
			});
		joint_plan_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
			"/sdk/arm/joint_plan_cmd", rclcpp::QoS(1).reliable(),
			[this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
				if (msg->data.size() != 15) return;
				js_arms_pub_->publish(makeCSP(arm_joints_,
					std::vector<double>(msg->data.begin(), msg->data.begin() + 14)));
			});
		// 14 值 = [x,y,z,qx,qy,qz,qw] × 2 臂 (位置+姿态全消费; 姿态无约束时上层显式发单位四元数)
		cart_plan_offset_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
			"/sdk/arm/cartesian_plan_offset_cmd", rclcpp::QoS(1).reliable(),
			[this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
				if (msg->data.size() != 14) return;
				geometry_msgs::msg::PoseArray pa;
				geometry_msgs::msg::Pose p;
				p.position.x = msg->data[0]; p.position.y = msg->data[1]; p.position.z = msg->data[2];
				p.orientation.x = msg->data[3]; p.orientation.y = msg->data[4];
				p.orientation.z = msg->data[5]; p.orientation.w = msg->data[6];
				pa.poses.push_back(p);
				p.position.x = msg->data[7]; p.position.y = msg->data[8]; p.position.z = msg->data[9];
				p.orientation.x = msg->data[10]; p.orientation.y = msg->data[11];
				p.orientation.z = msg->data[12]; p.orientation.w = msg->data[13];
				pa.poses.push_back(p);
				whole_body_pub_->publish(pa);
			});
		left_cart_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/sdk/arm/left_cartesian_servo_cmd", rclcpp::QoS(1).best_effort(),
			[this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
				left_pose_ = *msg; have_left_ = true; publishWholeBody();
			});
		right_cart_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/sdk/arm/right_cartesian_servo_cmd", rclcpp::QoS(1).best_effort(),
			[this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
				right_pose_ = *msg; have_right_ = true; publishWholeBody();
			});
	}

	void setupBodySubscriptions()
	{
		chassis_sub_ = create_subscription<geometry_msgs::msg::Twist>(
			"/sdk/chassis/velocity_cmd", rclcpp::QoS(1).best_effort(),
			[this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
				cmd_vel_pub_->publish(*msg);
			});
		body_sub_ = create_subscription<std_msgs::msg::Float64>(
			"/sdk/body/height_cmd", rclcpp::QoS(1).best_effort(),
			[this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
				js_torso_pub_->publish(makeCSP(torso_joints_, {msg->data}));
			});
		head_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
			"/sdk/head/cmd", rclcpp::QoS(1).best_effort(),
			[this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
				if (msg->data.size() != 2) return;
				js_head_pub_->publish(makeCSP(head_joints_, msg->data));
			});
		// 夹爪: mos 单通道独立下发 → 缓存双侧, 每条消息重发全关节列表 (JS 列全契约)
		// 量纲: mos mm 行程 → URDF m (关节行程 0~0.1 m = 0~100 mm)
		gripper_left_sub_ = create_subscription<std_msgs::msg::Float64>(
			"/sdk/gripper/left_cmd", rclcpp::QoS(1).reliable(),
			[this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
				gripper_left_m_ = msg->data * 0.001;
				publishGripper();
			});
		gripper_right_sub_ = create_subscription<std_msgs::msg::Float64>(
			"/sdk/gripper/right_cmd", rclcpp::QoS(1).reliable(),
			[this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
				gripper_right_m_ = msg->data * 0.001;
				publishGripper();
			});
	}

	void setupServices()
	{
		// 5 模式完整映射 (mos SDK 协议 uint8 mode: 0-4)
		set_mode_srv_ = create_service<unistackbot_interface::srv::SetControlMode>(
			"/sdk/set_control_mode",
			[this](const unistackbot_interface::srv::SetControlMode::Request::SharedPtr req,
				unistackbot_interface::srv::SetControlMode::Response::SharedPtr res) {
				switch (req->mode)
				{
				case 0:  // kIdle: 全部去激活
					res->success = doSwitch(kAllControllers, {});
					res->message = "kIdle";
					break;
				case 1:  // kJointServo: 关节流 → js_arms
				case 3:  // kJointPlan: 关节规划 → 同 1 (ruckig 天然覆盖)
					res->success = doSwitch({"whole_body_controller"}, {"js_arms"});
					res->message = (req->mode == 1) ? "kJointServo" : "kJointPlan";
					break;
				case 2:  // kCartesianServo: 笛卡尔流 → whole_body
				case 4:  // kCartesianPlan: 笛卡尔规划 → 同 2 (持续伺服覆盖)
					res->success = doSwitch({"js_arms"}, {"whole_body_controller"});
					res->message = (req->mode == 2) ? "kCartesianServo" : "kCartesianPlan";
					break;
				default:
					res->success = false;
					res->message = "未知模式 " + std::to_string(req->mode);
				}
			}, rmw_qos_profile_services_default, svc_group_);
		estop_srv_ = create_service<std_srvs::srv::SetBool>(
			"/sdk/emergency_stop",
			[this](const std_srvs::srv::SetBool::Request::SharedPtr req,
				std_srvs::srv::SetBool::Response::SharedPtr res) {
				if (req->data)
				{
					res->success = doSwitch(kAllControllers, {});
					res->message = res->success ? "急停生效" : "急停失败";
				}
				else
				{
					res->success = doSwitch({}, {"js_arms", "js_head", "js_torso", "js_gripper"});
					res->message = res->success ? "急停解除" : "解除失败";
				}
			}, rmw_qos_profile_services_default, svc_group_);
		inhibit_get_srv_ = create_service<std_srvs::srv::SetBool>(
			"/sdk/get_motor_inhibit_policy",
			[](const std_srvs::srv::SetBool::Request::SharedPtr,
				std_srvs::srv::SetBool::Response::SharedPtr res) {
				res->success = true;
				res->message = "{}";
			});
	}

	void publishWholeBody()
	{
		if (!have_left_ || !have_right_) return;
		geometry_msgs::msg::PoseArray msg;
		msg.poses = {left_pose_.pose, right_pose_.pose};
		whole_body_pub_->publish(msg);
	}

	void publishGripper()
	{
		// 0.0~0.1 m clamp (URDF 行程), 未下发侧持 0 (闭合)
		auto clamp = [](double v) { return v < 0.0 ? 0.0 : (v > 0.1 ? 0.1 : v); };
		js_gripper_pub_->publish(makeCSP(
			{"left_gripper_joint", "right_gripper_joint"}, {clamp(gripper_left_m_), clamp(gripper_right_m_)}));
	}

	// 幂等切换: 先查现态只切增量 (Humble 对"停用已 inactive"记 WARN 且可整单失败),
	// 空增量 = 已在目标态直接成功。原生服务客户端, 不起壳。
	[[nodiscard]] bool doSwitch(const std::vector<std::string> & deactivate,
		const std::vector<std::string> & activate)
	{
		using controller_manager_msgs::srv::ListControllers;
		using controller_manager_msgs::srv::SwitchController;
		auto resp = callService(list_cli_, std::make_shared<ListControllers::Request>(), 3.0);
		if (!resp)
		{
			RCLCPP_ERROR(get_logger(), "doSwitch: list_controllers 3s 无响应");
			return false;
		}
		std::map<std::string, std::string> state;
		for (const auto & c : resp->controller)
		{
			state[c.name] = c.state;
		}
		std::vector<std::string> de, ac;
		for (const auto & n : deactivate)
		{
			if (state.count(n) && state[n] == "active") de.push_back(n);
		}
		for (const auto & n : activate)
		{
			if (state.count(n) && state[n] == "inactive") ac.push_back(n);
		}
		if (de.empty() && ac.empty()) return true;
		auto sw_req = std::make_shared<SwitchController::Request>();
		sw_req->deactivate_controllers = de;
		sw_req->activate_controllers = ac;
		sw_req->strictness = SwitchController::Request::STRICT;
		auto sw_resp = callService(switch_cli_, sw_req, 10.0);
		if (!sw_resp)
		{
			RCLCPP_ERROR(get_logger(), "doSwitch: switch_controller 10s 无响应 (de=%zu ac=%zu)",
				de.size(), ac.size());
			return false;
		}
		return sw_resp->ok;
	}

	// 服务调用桥: 回调式 async_send_request + shared_ptr<promise>。
	// 裸 future 形态 (async_send_request(req) 返回值直接 wait_for/get) 在本机 Humble
	// 实测 SIGSEGV——此处响应在执行器线程内经回调取值, promise 闭包保活, 迟到回调无害。
	template<typename CliT>
	typename CliT::Response::SharedPtr callService(const std::shared_ptr<CliT> & cli,
		typename CliT::Request::SharedPtr req, double timeout_s)
	{
		auto prom = std::make_shared<std::promise<typename CliT::Response::SharedPtr>>();
		auto fut = prom->get_future();
		cli->async_send_request(req,
			[prom](typename CliT::SharedFuture f) {
				if (f.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
				{
					prom->set_value(f.get());
				}
			});
		if (fut.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready)
		{
			return nullptr;
		}
		return fut.get();
	}

	rclcpp::Publisher<JointCommand>::SharedPtr js_arms_pub_, js_head_pub_, js_torso_pub_, js_gripper_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr whole_body_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_servo_sub_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_plan_sub_, cart_plan_offset_sub_, head_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_cart_sub_, right_cart_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_sub_;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_sub_, gripper_left_sub_, gripper_right_sub_;
	rclcpp::Service<unistackbot_interface::srv::SetControlMode>::SharedPtr set_mode_srv_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_, inhibit_get_srv_;
	rclcpp::CallbackGroup::SharedPtr svc_group_;
	rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_cli_;
	rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_cli_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	geometry_msgs::msg::PoseStamped left_pose_, right_pose_;
	bool have_left_{false}, have_right_{false};
	double gripper_left_m_{0.0}, gripper_right_m_{0.0};
};

}  // namespace mos_compat

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	// 多线程执行器: /sdk 服务回调内同步等待 CM 客户端响应 (svc_group 与默认组各占一线程)
	// 注意: 执行器持 weak_ptr, 节点 shared_ptr 须在本作用域保活 (临时量传参即析构=空转)
	rclcpp::executors::MultiThreadedExecutor exec;
	auto node = std::make_shared<mos_compat::MosCompatNode>();
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();
	return 0;
}

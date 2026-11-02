/*
 * mos_compat_node —— 产品线 /sdk 接口网关 (P2 落地, 2026-11-02 机型参数化)
 * 本质 = 词汇翻译层 (零控制逻辑, 安全全在控制器); 存在理由 = 框架不学产品词汇。
 * 机型差异全部住参数 (随机型 yaml 走): 关节表/夹爪量纲/控制器实例名——
 * 新机型 = 新 yaml 段, 网关代码零改动 (框架"新机型不改代码"不变式的产品域镜像)。
 * /sdk 面是产品资产 (冻结纪律: 只加不改不删, App 零感知)。
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

class MosCompatNode : public rclcpp::Node
{
public:
	explicit MosCompatNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		: Node("mos_sdk_interface", options)
		, tf_buffer_(get_clock())
		, tf_listener_(tf_buffer_)
	{
		loadMachineParams();
		js_arms_pub_ = create_publisher<JointCommand>("/" + js_arms_ + "/command", rclcpp::QoS(1).reliable());
		whole_body_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
			"/" + whole_body_ + "/command", rclcpp::QoS(1).reliable());
		cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1).reliable());
		if (!head_joints_.empty())
		{
			js_head_pub_ = create_publisher<JointCommand>("/" + js_head_ + "/command", rclcpp::QoS(1).reliable());
		}
		if (!torso_joints_.empty())
		{
			js_torso_pub_ = create_publisher<JointCommand>("/" + js_torso_ + "/command", rclcpp::QoS(1).reliable());
		}
		if (gripper_joints_.size() == 2)
		{
			js_gripper_pub_ = create_publisher<JointCommand>("/" + js_gripper_ + "/command", rclcpp::QoS(1).reliable());
		}
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
		RCLCPP_INFO(get_logger(),
			"sdk 网关就绪 (arm=%zu head=%zu torso=%zu gripper=%zu; ctrl: %s/%s/%s/%s/%s)",
			arm_joints_.size(), head_joints_.size(), torso_joints_.size(), gripper_joints_.size(),
			js_arms_.c_str(), js_head_.c_str(), js_torso_.c_str(), js_gripper_.c_str(), whole_body_.c_str());
	}

private:
	// ---- 机型参数 (显式声明, 关节表无默认=fail-fast; 子系统缺席=空表) ----
	std::vector<std::string> arm_joints_, head_joints_, torso_joints_, gripper_joints_;
	std::string js_arms_{"js_arms"}, js_head_{"js_head"}, js_torso_{"js_torso"};
	std::string js_gripper_{"js_gripper"}, whole_body_{"whole_body_controller"};
	double gripper_scale_{0.001};   // /sdk 夹爪输入单位 → m
	double gripper_max_m_{0.1};     // clamp 上界 (与 URDF 行程一致)

	void loadMachineParams()
	{
		arm_joints_ = declare_parameter<std::vector<std::string>>("arm_joints", {});
		head_joints_ = declare_parameter<std::vector<std::string>>("head_joints", {});
		torso_joints_ = declare_parameter<std::vector<std::string>>("torso_joints", {});
		gripper_joints_ = declare_parameter<std::vector<std::string>>("gripper_joints", {});
		gripper_scale_ = declare_parameter<double>("gripper_scale", 0.001);
		gripper_max_m_ = declare_parameter<double>("gripper_max_m", 0.1);
		js_arms_ = declare_parameter<std::string>("js_arms_ctrl", js_arms_);
		js_head_ = declare_parameter<std::string>("js_head_ctrl", js_head_);
		js_torso_ = declare_parameter<std::string>("js_torso_ctrl", js_torso_);
		js_gripper_ = declare_parameter<std::string>("js_gripper_ctrl", js_gripper_);
		whole_body_ = declare_parameter<std::string>("whole_body_ctrl", whole_body_);
		if (arm_joints_.empty())
		{
			RCLCPP_FATAL(get_logger(), "缺 arm_joints 参数 (机型 yaml 段 mos_sdk_interface 未加载?)");
			throw std::runtime_error("arm_joints is required");
		}
		if (!gripper_joints_.empty() && gripper_joints_.size() != 2)
		{
			RCLCPP_FATAL(get_logger(), "gripper_joints 须空(无夹爪)或恰 2 个 [左,右], 实得 %zu",
				gripper_joints_.size());
			throw std::runtime_error("gripper_joints must be empty or 2");
		}
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
		const size_t n_arm = arm_joints_.size();
		joint_servo_sub_ = create_subscription<sensor_msgs::msg::JointState>(
			"/sdk/arm/joint_servo_cmd", rclcpp::QoS(1).best_effort(),
			[this, n_arm](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
				if (msg->position.size() != n_arm) return;
				js_arms_pub_->publish(makeCSP(arm_joints_, msg->position));
			});
		joint_plan_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
			"/sdk/arm/joint_plan_cmd", rclcpp::QoS(1).reliable(),
			[this, n_arm](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
				if (msg->data.size() != n_arm + 1) return;   // n 关节 + 时长
				js_arms_pub_->publish(makeCSP(arm_joints_,
					std::vector<double>(msg->data.begin(), msg->data.begin() + static_cast<long>(n_arm))));
			});
		// 14 值布局 = [x,y,z,qx,qy,qz,qw] × 2 臂 (位置+姿态全消费; 姿态无约束时上层显式发单位四元数)
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
				cmd_vel_pub_->publish(*msg);   // 无底盘机型: /cmd_vel 无消费者, 转发无害
			});
		if (!torso_joints_.empty())
		{
			body_sub_ = create_subscription<std_msgs::msg::Float64>(
				"/sdk/body/height_cmd", rclcpp::QoS(1).best_effort(),
				[this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
					js_torso_pub_->publish(makeCSP(torso_joints_, {msg->data}));
				});
		}
		if (!head_joints_.empty())
		{
			head_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
				"/sdk/head/cmd", rclcpp::QoS(1).best_effort(),
				[this, n = head_joints_.size()](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
					if (msg->data.size() != n) return;
					js_head_pub_->publish(makeCSP(head_joints_, msg->data));
				});
		}
		// 夹爪: /sdk 单通道独立下发 → 缓存双侧, 每条消息重发全关节列表 (JS 列全契约)
		if (gripper_joints_.size() == 2)
		{
			gripper_left_sub_ = create_subscription<std_msgs::msg::Float64>(
				"/sdk/gripper/left_cmd", rclcpp::QoS(1).reliable(),
				[this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
					gripper_left_m_ = msg->data * gripper_scale_;
					publishGripper();
				});
			gripper_right_sub_ = create_subscription<std_msgs::msg::Float64>(
				"/sdk/gripper/right_cmd", rclcpp::QoS(1).reliable(),
				[this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
					gripper_right_m_ = msg->data * gripper_scale_;
					publishGripper();
				});
		}
	}

	// 全停清单 = 本机型在册的运动控制器 (缺席子系统自动不含)
	std::vector<std::string> allControllers() const
	{
		std::vector<std::string> all = {js_arms_, whole_body_};
		if (!head_joints_.empty()) all.push_back(js_head_);
		if (!torso_joints_.empty()) all.push_back(js_torso_);
		if (gripper_joints_.size() == 2) all.push_back(js_gripper_);
		return all;
	}

	void setupServices()
	{
		// 5 模式完整映射 (/sdk 协议 uint8 mode: 0-4; 面冻结, 语义永不变)
		set_mode_srv_ = create_service<unistackbot_interface::srv::SetControlMode>(
			"/sdk/set_control_mode",
			[this](const unistackbot_interface::srv::SetControlMode::Request::SharedPtr req,
				unistackbot_interface::srv::SetControlMode::Response::SharedPtr res) {
				switch (req->mode)
				{
				case 0:  // kIdle: 全部去激活
					res->success = doSwitch(allControllers(), {});
					res->message = "kIdle";
					break;
				case 1:  // kJointServo: 关节流 → js_arms
				case 3:  // kJointPlan: 关节规划 → 同 1 (ruckig 天然覆盖)
					res->success = doSwitch({whole_body_}, {js_arms_});
					res->message = (req->mode == 1) ? "kJointServo" : "kJointPlan";
					break;
				case 2:  // kCartesianServo: 笛卡尔流 → whole_body
				case 4:  // kCartesianPlan: 笛卡尔规划 → 同 2 (持续伺服覆盖)
					res->success = doSwitch({js_arms_}, {whole_body_});
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
					res->success = doSwitch(allControllers(), {});
					res->message = res->success ? "急停生效" : "急停失败";
				}
				else
				{
					// 解除 = 关节域全恢复 (笛卡尔域不自动恢复——重进须显式 set_control_mode)
					std::vector<std::string> js_all;
					for (const auto & c : allControllers())
					{
						if (c != whole_body_) js_all.push_back(c);
					}
					res->success = doSwitch({}, js_all);
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
		// [0, gripper_max_m] clamp; 未下发侧持 0 (闭合)
		auto clamp = [max = gripper_max_m_](double v) { return v < 0.0 ? 0.0 : (v > max ? max : v); };
		js_gripper_pub_->publish(makeCSP(gripper_joints_, {clamp(gripper_left_m_), clamp(gripper_right_m_)}));
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

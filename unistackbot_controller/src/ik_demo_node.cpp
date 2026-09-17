/*
 * ik_demo_node —— RViz 交互式 IK 演示 (P1.4 演示素材)。
 *
 * RViz 中出现一个 6-DOF 可拖动标记 (末端目标)。拖动它:
 *   目标位姿 -> DLS IK (种子=上一解, 流式连续) -> 解经 max_delta 限幅
 *   -> JTC 单点轨迹 -> 臂平滑跟随你的手。
 *
 * 冗余可见: 手的位置画圈时, 肘部会自主摆动 (解流形上滑行)。
 *
 * 前置: control.launch.py robot:=xarm7 已起 + RViz 已开
 *       (RViz: Add -> InteractiveMarkers -> Update Topic: /ik_target/update)
 * 启动: ros2 run unistackbot_controller ik_demo_node
 */
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "unistackbot_controller/dls_ik.hpp"
#include "ulog/ulog.hpp"

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;

using unistackbot_controller::CartesianPose;
using unistackbot_controller::DlsIk;
using unistackbot_controller::IkResult;
using unistackbot_controller::RedundancyPreference;
using unistackbot_controller::UrdfFk;

class IkDemo : public rclcpp::Node, public std::enable_shared_from_this<IkDemo>
{
public:
	IkDemo()
	: Node("ik_demo_node")
	{
		// 构造轻壳: 全部装配在 start() (参数服务调用需要 shared_from_this,
		// 构造函数内对象尚未被 shared_ptr 接管 —— 2026-09-17 bad_weak_ptr 实测)
	}

	// 构造完成后调用: 注入 self (彻底绕开 shared_from_this —— 双继承下
	// Node::shared_from_this() 有歧义且构造内非法, 2026-09-17 实测), 再装配
	void setup(std::shared_ptr<IkDemo> self) {self_ = self;}
	void start()
	{
		// URDF + 关节契约 (运行链路实时读, 不写死机型)
		const auto urdf = getParam("/robot_state_publisher", "robot_description").string_value;
		if (urdf.empty() || !fk_.init(urdf, "link_base", "link7", msg_) || !ik_.init(&fk_, msg_))
		{
			RCLCPP_ERROR(get_logger(), "%s", msg_.empty() ? "robot_description 为空" : msg_.c_str());
			return;
		}
		const unsigned int n = fk_.jointCount();
		for (unsigned int i = 0; i < n; ++i)
		{
			joints_.push_back(fk_.jointNames()[i]);
			q_prev_.push_back((fk_.qMin()[i] + fk_.qMax()[i]) / 2.0);   // 中心种子起步
			max_delta_.push_back(0.15);   // 每拍限幅 [rad] (出口速度物理受限)
		}

		jtc_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
			this, "/joint_trajectory_controller/follow_joint_trajectory");
		// 目标标记位姿 = 中心种子 FK (标记与臂重合起步)
		fk_.fk(q_prev_, goal_);

		// 标准 InteractiveMarkerServer: 自带 RESET/KEEP_ALIVE 协议 (RViz 显示项
		// 吃的是 InteractiveMarkerUpdate, 裸 publisher 类型不匹配连不上)
		server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
			"ik_target", self_);
		// feedback 走裸订阅 (server 的 setCallback 分发链实测未触发, 2026-09-17;
		// 裸订阅早期验证可行。同一话题两个订阅者互不干扰)
		feedback_sub_ = create_subscription<InteractiveMarkerFeedback>(
			"/ik_target/feedback", 10,
			[this](const InteractiveMarkerFeedback::SharedPtr fb)
			{
				if (fb->event_type != InteractiveMarkerFeedback::POSE_UPDATE) {return;}
				// 拖动目标节流记录 (500ms 一条; 全记录会 20Hz 刷屏)
				const auto now = this->now();
				if ((now - last_fb_log_).seconds() > 0.5)
				{
					ULOG_INFO("拖动目标: (%.3f, %.3f, %.3f)",
						fb->pose.position.x, fb->pose.position.y, fb->pose.position.z);
					last_fb_log_ = now;
				}
				goal_.x = fb->pose.position.x;
				goal_.y = fb->pose.position.y;
				goal_.z = fb->pose.position.z;
				goal_.qw = fb->pose.orientation.w;
				goal_.qx = fb->pose.orientation.x;
				goal_.qy = fb->pose.orientation.y;
				goal_.qz = fb->pose.orientation.z;
				has_goal_ = true;
			});
		// 发布标记 (RViz: Add -> InteractiveMarkers -> /ik_target/update)
		InteractiveMarker im;
		im.header.frame_id = "link_base";
		im.name = "ik_target";
		im.description = "拖动我 -> IK";
		im.scale = 0.35;
		im.pose.position.x = goal_.x;
		im.pose.position.y = goal_.y;
		im.pose.position.z = goal_.z;
		im.pose.orientation.w = 1.0;
		// 可视化本体: 空心 controls 在 RViz 里几乎不可见 (2026-09-17 实测),
		// 必须给 control 挂几何 marker; 整体 MOVE_ROTATE_3D + 球体 = 拖球即走
		{
			visualization_msgs::msg::Marker sphere;
			sphere.type = visualization_msgs::msg::Marker::SPHERE;
			sphere.scale.x = 0.08;
			sphere.scale.y = 0.08;
			sphere.scale.z = 0.08;
			sphere.color.r = 0.2f;
			sphere.color.g = 0.9f;
			sphere.color.b = 0.3f;
			sphere.color.a = 0.9f;
			InteractiveMarkerControl main;
			main.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
			main.always_visible = true;
			main.markers.push_back(sphere);
			main.name = "move_rotate";
			im.controls.push_back(main);

			for (int axis = 0; axis < 3; ++axis)
			{
				InteractiveMarkerControl c;
				c.orientation.w = 1.0;
				c.orientation.x = (axis == 0) ? 1.0 : 0.0;
				c.orientation.y = (axis == 1) ? 1.0 : 0.0;
				c.orientation.z = (axis == 2) ? 1.0 : 0.0;
				c.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
				c.name = "rot_" + std::to_string(axis);
				im.controls.push_back(c);
				InteractiveMarkerControl m = c;
				m.name = "move_" + std::to_string(axis);
				m.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
				im.controls.push_back(m);
			}
		}
		server_->insert(im);
		server_->applyChanges();

		// 20Hz 求解循环
		solve_timer_ = create_wall_timer(std::chrono::milliseconds(50),
			[this]()
			{
				if (!has_goal_) {return;}
				std::vector<double> q_sol;
				unistackbot_controller::DlsIkStats st;
				const auto r = ik_.solve(goal_, q_prev_, RedundancyPreference(), q_sol, &st);
				++(r == IkResult::OK ? n_ok_ : n_fail_);
				sum_us_ += st.solve_us;
				max_us_ = std::max(max_us_, st.solve_us);
				const auto now = this->now();
				if (last_summary_.nanoseconds() != 0 && (now - last_summary_).seconds() >= 1.0)
				{
					const auto total = n_ok_ + n_fail_;
					ULOG_INFO("[统计] 求解 %u 次: 成功 %u (%.0f%%) 平均 %.0fus 最差 %.0fus",
						total, n_ok_, 100.0 * n_ok_ / std::max<uint32_t>(total, 1),
						sum_us_ / std::max<uint32_t>(total, 1), max_us_);
					n_ok_ = n_fail_ = 0; sum_us_ = 0; max_us_ = 0;
				}
				last_summary_ = now;
				if (r != IkResult::OK) {return;}   // 失败保持上一解 (安全语义)

				// max_delta 限幅后发 JTC (0.2s 到达, 平滑跟随)
				std::vector<double> q_cmd(q_sol.size());
				for (std::size_t i = 0; i < q_sol.size(); ++i)
				{
					const double d = q_sol[i] - q_prev_[i];
					q_cmd[i] = q_prev_[i] + std::clamp(d, -max_delta_[i], max_delta_[i]);
				}
				q_prev_ = q_cmd;

				control_msgs::action::FollowJointTrajectory::Goal goal;
				goal.trajectory.joint_names = joints_;
				trajectory_msgs::msg::JointTrajectoryPoint pt;
				pt.positions = q_cmd;
				pt.time_from_start = rclcpp::Duration::from_seconds(0.2);
				goal.trajectory.points.push_back(pt);
				if (jtc_->action_server_is_ready())
				{
					rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opts;
					jtc_->async_send_goal(goal, opts);
				}
			});
		RCLCPP_INFO(get_logger(), "IK 演示就绪: RViz 里拖动 6D 标记, 臂会跟随 (手画圈看肘部自主摆动)");
	}

private:
	std::shared_ptr<IkDemo> self_;   // 自引用 (setup 注入, 供 spin/server 构造使用)
	std::shared_ptr<rclcpp::Node> shared_from_this_internal() {return self_;}
	rcl_interfaces::msg::ParameterValue getParam(const std::string & node, const std::string & name)
	{
		auto cli = create_client<rcl_interfaces::srv::GetParameters>(node + "/get_parameters");
		if (!cli->wait_for_service(std::chrono::seconds(5)))
		{
			return rcl_interfaces::msg::ParameterValue();
		}
		auto req = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
		req->names = {name};
		auto fut = cli->async_send_request(req);
		if (rclcpp::spin_until_future_complete(self_, fut, std::chrono::seconds(5)) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			return rcl_interfaces::msg::ParameterValue();
		}
		return fut.get()->values[0];
	}

	std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
	rclcpp::Subscription<InteractiveMarkerFeedback>::SharedPtr feedback_sub_;
	rclcpp::TimerBase::SharedPtr solve_timer_;
	rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr jtc_;
	bool has_goal_{false};
	bool ready_{false};
	// ulog 统计 (每秒摘要; 失败明细由 dls_ik 内 ULOG_WARN 记录)
	uint32_t n_ok_{0};
	uint32_t n_fail_{0};
	double sum_us_{0.0};
	double max_us_{0.0};
	rclcpp::Time last_summary_{0, 0, RCL_ROS_TIME};
	rclcpp::Time last_fb_log_{0, 0, RCL_ROS_TIME};
	CartesianPose goal_;
	UrdfFk fk_;
	DlsIk ik_;
	std::string msg_;
	std::vector<std::string> joints_;
	std::vector<double> q_prev_, max_delta_;
};

int main(int argc, char ** argv)
{
	// ulog: 终端 sink 恒开 (launch 捕获 stdout), 文件 sink 记全程 (/tmp/ik_demo.ulog,
	// 10MB×5 轮转); dls_ik 失败明细/WARN 与本节点统计/拖动目标全部落盘
	unistackbot_common::ulog_config cfg{};
	cfg.console = true;
	cfg.file_path = "/tmp/ik_demo.ulog";
	unistackbot_common::ulog_init(cfg);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IkDemo>();
	node->setup(node);
	node->start();
	rclcpp::spin(node);
	rclcpp::shutdown();
	unistackbot_common::ulog_shutdown();
	return 0;
}

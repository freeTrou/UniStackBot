/*
 * ik_demo_node —— RViz 交互式笛卡尔拖动演示。
 *
 * RViz 中出现一个 6-DOF 可拖动标记 (末端目标)。拖动它:
 *   标记位姿 -> /cartesian_motion_controller/target (值通道)
 *   -> CM 在控制环上做流式 IK 伺服 (种子=上一解, 零空间目标, 断流受控减速)
 *   -> 臂平滑跟随你的手。
 *
 * 2026-09-21 重写: 本地 DlsIk + JTC 单点轨迹退役 —— CM 控制器就是 IK 宿主
 * (P1.5 架构), 本节点只做"人手 → 目标流"。冗余可见性不变: 手画圈时肘部自主摆动
 * (CM 零空间限位中心距目标)。
 *
 * 前置: 链已启动 (control/ign/mujoco.launch.py robot:=<机型>) 且 CM 已接管:
 *   ros2 control switch_controllers --deactivate joint_stream_controller \
 *       --activate cartesian_motion_controller
 *   RViz: Add -> InteractiveMarkers -> Update Topic: /ik_target/update
 * 启动: ros2 run unistackbot_controller ik_demo_node
 */
#include <chrono>
#include <memory>
#include <string>

#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

#include "ulog/ulog.hpp"

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;

class IkDemo : public rclcpp::Node, public std::enable_shared_from_this<IkDemo>
{
public:
	IkDemo()
	: Node("ik_demo_node")
	{
		// 构造轻壳: 全部装配在 start() (参数服务调用需要 self_,
		// 构造函数内对象尚未被 shared_ptr 接管 —— 2026-09-17 bad_weak_ptr 实测)
	}

	// 构造完成后调用: 注入 self (彻底绕开 shared_from_this —— 双继承下
	// Node::shared_from_this() 有歧义且构造内非法, 2026-09-17 实测), 再装配
	void setup(std::shared_ptr<IkDemo> self) {self_ = self;}
	void start()
	{
		// base 系名读 CM 参数 (机型无关; 标记与目标帧一致, CM 只收 base 系)
		const auto base = getParam("/cartesian_motion_controller", "base_link").string_value;
		if (base.empty())
		{
			ULOG_ERROR("读 /cartesian_motion_controller 的 base_link 失败 (CM 未加载?)");
			return;
		}

		// 目标值通道: 与 CM 契约同款 QoS (reliable + KeepLast1)
		target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
			"/cartesian_motion_controller/target", 10);

		// 标准 InteractiveMarkerServer: 自带 RESET/KEEP_ALIVE 协议 (RViz 显示项
		// 吃的是 InteractiveMarkerUpdate, 裸 publisher 类型不匹配连不上)
		server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
			"ik_target", self_);
		// feedback 走裸订阅 (server 的 setCallback 分发链实测未触发, 2026-09-17;
		// 裸订阅早期验证可行。同一话题两个订阅者互不干扰)
		feedback_sub_ = create_subscription<InteractiveMarkerFeedback>(
			"/ik_target/feedback", 10,
			[this, base](const InteractiveMarkerFeedback::SharedPtr fb)
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
				geometry_msgs::msg::PoseStamped t;
				t.header.frame_id = base;
				t.header.stamp = now;
				t.pose = fb->pose;
				target_pub_->publish(t);
				++n_sent_;
			});
		// 发布标记 (RViz: Add -> InteractiveMarkers -> /ik_target/update);
		// 初始位姿 = 标记原点 (拖到臂附近再拖动, CM 断流减速兜底无目标漂移)
		InteractiveMarker im;
		im.header.frame_id = base;
		im.name = "ik_target";
		im.description = "拖动我 -> CM target";
		im.scale = 0.35;
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

		// 每秒摘要 (发送计数; 求解统计看 CM 的 status/WCET 终报 —— IK 已不在本节点)
		stat_timer_ = create_wall_timer(std::chrono::seconds(1),
			[this]()
			{
				if (n_sent_ == 0) {return;}
				ULOG_INFO("[统计] 目标流 %u 条/s (IK 伺服在 CM: ~/status 可看误差/奇异遥测)",
					n_sent_);
				n_sent_ = 0;
			});
		ULOG_INFO("IK 演示就绪: RViz 里拖动 6D 标记 -> CM 伺服跟随 (手画圈看肘部自主摆动)");
	}

private:
	std::shared_ptr<IkDemo> self_;   // 自引用 (setup 注入, 供 spin/server 构造使用)
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
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
	rclcpp::TimerBase::SharedPtr stat_timer_;
	uint32_t n_sent_{0};
	rclcpp::Time last_fb_log_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
	// ulog: 终端 sink 恒开 (launch 捕获 stdout)
	unistackbot_common::ulog_config cfg{};
	cfg.console = true;
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

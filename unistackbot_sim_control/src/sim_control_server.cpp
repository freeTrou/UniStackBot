#include "unistackbot_sim_control/sim_control_server.hpp"

#include <memory>

#include "std_srvs/srv/trigger.hpp"
#include "unistackbot_sim_control/srv/set_joint_state.hpp"

namespace unistackbot_sim_control
{

SimControlServer::SimControlServer(const rclcpp::Node::SharedPtr & node,
	Sink sink, SetStateValidator validate_set_state)
{
	callback_group_ = node->create_callback_group(
		rclcpp::CallbackGroupType::MutuallyExclusive);

	auto make_trigger_service = [&](const std::string & name, SimCmdType type) {
			auto cb = [this, sink, type](
				const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
				std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
					SimCommand cmd;
					cmd.type = type;
					response->success = sink(cmd, response->message);
				};
			services_.push_back(node->create_service<std_srvs::srv::Trigger>(
				name, cb, rmw_qos_profile_services_default, callback_group_));
		};

	make_trigger_service("/sim_control/reset", SimCmdType::RESET);
	make_trigger_service("/sim_control/pause", SimCmdType::PAUSE);
	make_trigger_service("/sim_control/resume", SimCmdType::RESUME);
	make_trigger_service("/sim_control/step", SimCmdType::STEP);

	auto set_state_cb = [this, sink, validate_set_state](
		const std::shared_ptr<srv::SetJointState::Request> request,
		std::shared_ptr<srv::SetJointState::Response> response) {
			SimCommand cmd;
			cmd.type = SimCmdType::SET_STATE;
			std::string message;
			if (!validate_set_state(
					request->joint_names, request->positions, request->velocities,
					cmd, message))
			{
				response->success = false;
				response->message = message;
				return;
			}
			response->success = sink(cmd, response->message);
		};
	services_.push_back(node->create_service<srv::SetJointState>(
		"/sim_control/set_joint_state", set_state_cb,
		rmw_qos_profile_services_default, callback_group_));
}

}  // namespace unistackbot_sim_control

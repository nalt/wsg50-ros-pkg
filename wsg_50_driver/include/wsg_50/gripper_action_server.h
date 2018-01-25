#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "wsg_50/gripper_communication.h"
#include "wsg_50/node_state.h"
#include "wsg_50_common/CommandAction.h"

typedef actionlib::ActionServer<wsg_50_common::CommandAction> ActionServer;
typedef ActionServer::GoalHandle GoalHandle;


class GripperActionServer {
	public:
		GripperActionServer(ros::NodeHandle& node_handle, std::string base_name, GripperCommunication& gripper_com, NodeState& node_state, wsg_50_common::Status& gripper_status);
		void start();
		void shutdown();
		void doWork();
	private:
		ros::NodeHandle& node_handle;
		GripperCommunication& gripper_com;
		NodeState& node_state;
		wsg_50_common::Status& gripper_status;
		std::string base_name;
		ActionServer* action_server;
		bool action_server_started;
		std::shared_ptr<CommandSubscription> commandSubscriber;
		GoalHandle current_goal_handle;
		bool has_active_goal;

		void goalCallback(GoalHandle goal_handle);
		void cancelCallback(GoalHandle goal_handle);
		void handleCommand(wsg_50_common::Command command, GoalHandle& goal_handle);
		void moveCommandCallback(msg_t& message);
};



#endif /* ACTION_SERVER_H_ */

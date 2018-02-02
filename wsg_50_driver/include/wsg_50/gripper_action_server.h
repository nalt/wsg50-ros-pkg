#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "wsg_50/gripper_communication.h"
#include "wsg_50/gripper_socket.h"
#include "wsg_50/node_state.h"
#include "wsg_50_common/CommandAction.h"
#include <queue>

typedef actionlib::ActionServer<wsg_50_common::CommandAction> ActionServer;
typedef ActionServer::GoalHandle GoalHandle;

enum class ActionStateCode: uint32_t {
	NO_GOAL = 0,
	AWAIT_COMMAND = 1,
	AWAIT_STATUS_UPDATE = 2,
	DONE = 3
};

class ActionState {
	public:
		ActionStateCode state;
		int return_code;
		int expected_grasping_state;
};

class GripperActionServer {
	public:
		GripperActionServer(ros::NodeHandle& node_handle, std::string base_name, GripperCommunication& gripper_com, NodeState& node_state);
		void start();
		void shutdown();
		void doWork();
	private:
		ros::NodeHandle& node_handle;
		GripperCommunication& gripper_com;
		NodeState& node_state;
		std::string base_name;
		ActionServer* action_server;
		bool action_server_started;
		GoalHandle current_goal_handle;
		CommandSubscription soft_stop_subscription;
		CommandSubscription emergency_stop_subscription;
		CommandSubscription grasping_state_subscription;

		void goalCallback(GoalHandle goal_handle);
		void cancelCallback(GoalHandle goal_handle);
		void handleCommand(wsg_50_common::Command command, GoalHandle& goal_handle);

		void abort();
		void commandCallback(Message& message);
		void stopCallback(Message& message);

		wsg_50_common::Status fillStatus();
		ActionState action_state;
		void graspingStateCallback(Message& message);
};



#endif /* ACTION_SERVER_H_ */

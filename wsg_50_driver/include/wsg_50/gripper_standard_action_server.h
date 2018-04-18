#ifndef ACTION_STANDARD_SERVER_H_
#define ACTION_STANDARD_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "wsg_50/gripper_communication.h"
#include "wsg_50/gripper_socket.h"
#include "wsg_50/node_state.h"
#include <control_msgs/GripperCommandAction.h>
#include <queue>

typedef actionlib::ActionServer<control_msgs::GripperCommandAction> StandardActionServer;
typedef StandardActionServer::GoalHandle GoalHandle;

enum class ActionStateCode : uint32_t
{
  NO_GOAL = 0,
  AWAIT_COMMAND = 1,
  AWAIT_STATUS_UPDATE = 2,
  DONE = 3
};

class ActionState
{
public:
  ActionStateCode state;
  int return_code;
  int expected_grasping_state;
};

class GripperStandardActionServer
{
public:
  GripperStandardActionServer(ros::NodeHandle& node_handle, std::string base_name, GripperCommunication& gripper_com,
                      NodeState& node_state, float speed, bool stop_on_block);
  void start();
  void shutdown();
  void doWork();

private:
  ros::NodeHandle& node_handle;
  GripperCommunication& gripper_com;
  NodeState& node_state;
  std::string base_name;
  StandardActionServer* action_server;
  bool action_server_started;
  GoalHandle current_goal_handle;
  CommandSubscription soft_stop_subscription;
  CommandSubscription emergency_stop_subscription;
  CommandSubscription grasping_state_subscription;
  float speed;
  bool stop_on_block;

  void goalCallback(GoalHandle goal_handle);
  void cancelCallback(GoalHandle goal_handle);
  void handleCommand(control_msgs::GripperCommand command, GoalHandle& goal_handle);

  void abort(std::string message_text);
  void commandCallback(std::shared_ptr<CommandError> error, std::shared_ptr<Message> message);
  void stopCallback(Message& message);

  control_msgs::GripperCommandResult fillStatus();
  ActionState action_state;
  void graspingStateCallback(Message& message);
};

#endif /* ACTION_STANDARD_SERVER_H_ */

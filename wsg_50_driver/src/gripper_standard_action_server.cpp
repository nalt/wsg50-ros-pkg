#include "wsg_50/gripper_standard_action_server.h"
#define NO_EXPECTATION -10

GripperStandardActionServer::GripperStandardActionServer(ros::NodeHandle& node_handle, std::string base_name,
                                         GripperCommunication& gripper_com, NodeState& node_state, float speed)
  : node_handle(node_handle), gripper_com(gripper_com), node_state(node_state), speed(speed * 1000)
{
  this->base_name = base_name;
  this->action_server_started = false;
  this->action_server = nullptr;
  this->action_state.state = ActionStateCode::NO_GOAL;
  this->action_state.return_code = -1;
  this->action_state.expected_grasping_state = NO_EXPECTATION;
  this->action_state.ignore_axis_blocked = true;
  this->stop_on_block = false;
  this->last_command_state = LastCommandState::UNKNOWN;

  grasping_state_subscription =
      this->gripper_com.subscribe((unsigned char)WellKnownMessageId::GRIPPING_STATE,
                                  [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                                    if ((error == nullptr) && (message != nullptr))
                                    {
                                      this->graspingStateCallback(*message.get());
                                    }
                                  });
  soft_stop_subscription =
      this->gripper_com.subscribe((unsigned char)WellKnownMessageId::SOFT_STOP,
                                  [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                                    if ((error == nullptr) && (message != nullptr))
                                    {
                                      this->stopCallback(*message.get());
                                    }
                                  });
  emergency_stop_subscription =
      this->gripper_com.subscribe((unsigned char)WellKnownMessageId::EMERGENCY_STOP,
                                  [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                                    if ((error == nullptr) && (message != nullptr))
                                    {
                                      this->stopCallback(*message.get());
                                    }
                                  });
}

void GripperStandardActionServer::abort(std::string message_text)
{
  ROS_INFO(message_text.c_str());
  if (this->action_state.state != ActionStateCode::NO_GOAL)
  {
    control_msgs::GripperCommandResult result;
    result = this->fillStatus();
    this->current_goal_handle.setAborted(result, message_text);
    this->action_state.state = ActionStateCode::NO_GOAL;
    this->gripper_com.setOverrideForGripperErrorState(false);
  }
}

void GripperStandardActionServer::doWork()
{
  if (this->action_state.state == ActionStateCode::DONE)
  {
    control_msgs::GripperCommandResult result;
    if (this->action_state.return_code == E_SUCCESS)
    {
      this->last_command_state = LastCommandState::SUCCESSFUL;
      result = this->fillStatus();
      result.reached_goal = true;
      this->current_goal_handle.setSucceeded(result, "Goal reached");
    }
    else
    {
      this->last_command_state = LastCommandState::FAILED;
      if (this->action_state.ignore_axis_blocked && this->action_state.return_code == E_AXIS_BLOCKED) {
        // special case in that pre-positioning (move) is used as workaround to pickup parts with an unknown width
        // the gripper goes into an error state even when stop_on_block is false, therefore we acknowledge the error an wait until the reported values are nominal again
        printf("[GripperStandardActionServer::doWork] received E_AXIS_BLOCKED, but stop_on_block was false -> will acknowledge gripper error\n");
        try {
          this->gripper_com.acknowledge_error();
          // it takes a while for the gripper to report the correct force, therefore we wait until we received three updates of the force values
          this->gripper_com.awaitUpdateForMessage((unsigned char)WellKnownMessageId::FORCE_VALUES, nullptr, 3);
          this->gripper_com.awaitUpdateForMessage((unsigned char)WellKnownMessageId::GRIPPING_STATE);
          result = this->fillStatus();
          this->current_goal_handle.setSucceeded(result, "Goal reached");
        } catch (...) {
          this->current_goal_handle.setAborted(result, "Goal aborted, gripper may have reached its goal, but confirmation requests have timed out.");
        }
      } else {
        result = this->fillStatus();
        this->current_goal_handle.setAborted(result, "Goal aborted, command did not return success code");
      }
    }

    this->gripper_com.setOverrideForGripperErrorState(false);
    this->action_state.state = ActionStateCode::NO_GOAL;
    this->action_state.expected_grasping_state = NO_EXPECTATION;
    this->action_state.return_code = -1;
  }
}

void GripperStandardActionServer::start()
{
  this->action_server =
      new StandardActionServer(this->node_handle, this->base_name, boost::bind(&GripperStandardActionServer::goalCallback, this, _1),
                       boost::bind(&GripperStandardActionServer::cancelCallback, this, _1), false);
  this->action_server->start();
  this->action_server_started = true;
}

void GripperStandardActionServer::shutdown()
{
  if (this->action_server_started == true)
  {
    this->gripper_com.unregisterListener(soft_stop_subscription.messageId, soft_stop_subscription.listenerId);
    this->gripper_com.unregisterListener(emergency_stop_subscription.messageId, emergency_stop_subscription.listenerId);
    this->gripper_com.unregisterListener(grasping_state_subscription.messageId, grasping_state_subscription.listenerId);
    delete this->action_server;
    this->action_server = nullptr;
  }
}

void GripperStandardActionServer::goalCallback(GoalHandleStandard goal_handle)
{
  auto goal = goal_handle.getGoal();
  goal->command.max_effort;

  std::string reason_for_rejection = "";
  if (this->gripper_com.acceptsCommands(reason_for_rejection) == false)
  {
    control_msgs::GripperCommandResult result;
    result = this->fillStatus();
    goal_handle.setRejected(result, reason_for_rejection);
  }
  else
  {
    if (this->action_state.state != ActionStateCode::NO_GOAL)
    {
      ROS_WARN("Aborting current goal. This is not supposed to happen.");
      this->abort("Received new goal and the gripper seems to be idle. Aborting the previous action.");
    }
    goal_handle.setAccepted("Accept new gripper command");
    ROS_INFO("Accepted goal %s", goal_handle.getGoalID().id.c_str());

    this->current_goal_handle = goal_handle;
    this->action_state.state = ActionStateCode::AWAIT_COMMAND;
    this->action_state.expected_grasping_state = NO_EXPECTATION;
    this->handleCommand(goal->command, goal_handle);
  }
}

void GripperStandardActionServer::handleCommand(control_msgs::GripperCommand command, GoalHandleStandard& goal_handle)
{
  ROS_INFO("Handle command position: %f, effort: %f", command.position, command.max_effort);

  command.position = command.position * 1000;
  this->command_position = command.position;
  this->command_max_effort = command.max_effort;

  this->gripper_com.setOverrideForGripperErrorState(true);
  try
  {
    this->gripper_com.move(command.position, this->speed, this->stop_on_block,
                            [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
     this->commandCallback(error, message);
     });
  }
  catch (std::runtime_error& ex)
  {
    control_msgs::GripperCommandResult result;
    result = this->fillStatus();
    printf("Error while sending gripper command: %s\n", ex.what());
    goal_handle.setAborted(result, "Error while sending the command to the gripper");
  }
}

void GripperStandardActionServer::commandCallback(std::shared_ptr<CommandError> error, std::shared_ptr<Message> message)
{
  if ((error == nullptr) && (message != nullptr))
  {
    if (this->action_state.state == ActionStateCode::AWAIT_COMMAND)
    {
      this->action_state.return_code = this->gripper_com.decodeStatus(*message.get());
      this->action_state.state = ActionStateCode::AWAIT_STATUS_UPDATE;
    }
  }
  else
  {
    if (this->action_state.state != ActionStateCode::NO_GOAL)
    {
      if ((error != nullptr) && (error.get()->id == CommandError::TIMEOUT))
      {
        this->abort("Command has timed out");
      }
      else
      {
        this->abort("Command has failed with an unkown error");
      }
    }
  }
}

void GripperStandardActionServer::stopCallback(Message& message)
{
  this->abort("Received stop command");
}

void GripperStandardActionServer::cancelCallback(GoalHandleStandard goal_handle)
{
  this->abort("Received cancel callback");
}

control_msgs::GripperCommandResult GripperStandardActionServer::fillStatus()
{
  control_msgs::GripperCommandResult status;
  auto gripperState = this->gripper_com.getState();
  status.position = static_cast<double>(gripperState.width / 1000);
  status.effort = static_cast<double>(gripperState.current_force);
  status.reached_goal = false;

  double position_tolerance = 1; // mm
  if (this->command_position < gripperState.width + position_tolerance && this->command_position > gripperState.width - position_tolerance)
    status.reached_goal = true;
  else
    status.reached_goal = false;

  //check if gripper stalled
  double speed_tolerance = 1; // mm/s
  if (
    (this->last_command_state == LastCommandState::FAILED)
    // || (gripperState.current_force >= this->command_max_effort && gripperState.current_speed < speed_tolerance)
  ) {
    status.stalled = true;
  } else {
    status.stalled = false;
  }

  return status;
}

void GripperStandardActionServer::graspingStateCallback(Message& message)
{
  if (this->action_state.state == ActionStateCode::AWAIT_STATUS_UPDATE)
  {
    this->action_state.state = ActionStateCode::DONE;
  }
}

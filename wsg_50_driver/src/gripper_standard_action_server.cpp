#include "wsg_50/gripper_standard_action_server.h"
#define NO_EXPECTATION -10

//TODO check all this->fillStatus(); after editing it
//TODO implement stalled & reached_goal in result & feedback

GripperStandardActionServer::GripperStandardActionServer(ros::NodeHandle& node_handle, std::string base_name,
                                         GripperCommunication& gripper_com, NodeState& node_state, float speed, bool stop_on_block)
  : node_handle(node_handle), gripper_com(gripper_com), node_state(node_state), speed(speed), stop_on_block(stop_on_block)
{
  this->base_name = base_name;
  this->action_server_started = false;
  this->action_server = nullptr;
  this->action_state.state = ActionStateCode::NO_GOAL;
  this->action_state.return_code = -1;
  this->action_state.expected_grasping_state = NO_EXPECTATION;

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
  }
}

void GripperStandardActionServer::doWork()
{
  if (this->action_state.state == ActionStateCode::DONE)
  {
    control_msgs::GripperCommandResult result;
    result = this->fillStatus();
    //result.status.return_code = this->action_state.return_code;
    if (this->action_state.return_code == E_SUCCESS)
    {
      if (this->action_state.expected_grasping_state == NO_EXPECTATION)
      {
        this->current_goal_handle.setSucceeded(result, "Goal reached");
      }
      else
      {
        if (this->action_state.expected_grasping_state == this->gripper_com.getState().grasping_state)
        {
          this->current_goal_handle.setSucceeded(result, "Goal reached");
        }
        else
        {
          this->current_goal_handle.setAborted(result, "Goal aborted, wrong grasping state");
        }
      }
    }
    else
    {
      this->current_goal_handle.setAborted(result, "Goal aborted, command did not return success code");
    }

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

void GripperStandardActionServer::goalCallback(GoalHandle goal_handle)
{
  auto goal = goal_handle.getGoal();

  std::string reason_for_rejection = "";
  if (this->gripper_com.acceptsCommands(reason_for_rejection) == false)
  {
    control_msgs::GripperCommandResult result;
    result = this->fillStatus(); //TODO check after inplementing fillStatus
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

void GripperStandardActionServer::handleCommand(control_msgs::GripperCommand command, GoalHandle& goal_handle)
{
  printf("handle command\n");

  //TODO why multiply by 1000 when using the custom action version
  //command.width = command.width * 1000;
  //command.speed = command.speed * 1000;

  try
  {
    this->action_state.expected_grasping_state = wsg_50_common::Status::IDLE;
    this->gripper_com.set_force(command.max_effort, nullptr, 1000);
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

void GripperStandardActionServer::cancelCallback(GoalHandle goal_handle)
{
  this->abort("Received cancel callback");
}

control_msgs::GripperCommandResult GripperStandardActionServer::fillStatus()
{
  //TODO
  control_msgs::GripperCommandResult status;
  auto gripperState = this->gripper_com.getState();
  status.position = gripperState.width; //TODO is this the current width ???????
  status.effort = gripperState.current_force;
  return status;
}

void GripperStandardActionServer::graspingStateCallback(Message& message)
{
  if (this->action_state.state == ActionStateCode::AWAIT_STATUS_UPDATE)
  {
    this->action_state.state = ActionStateCode::DONE;
  }
}

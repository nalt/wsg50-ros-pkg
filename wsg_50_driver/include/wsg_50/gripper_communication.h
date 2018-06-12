#ifndef GRIPPER_COMM_H_
#define GRIPPER_COMM_H_

#include <ros/ros.h>
#include <wsg_50/gripper_socket.h>
#include "wsg_50_common/Status.h"
#include "msg.h"
#include "common.h"
#include <map>
#include <functional>
#include <memory>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>

class CommandError
{
public:
  static const int UNKNOWN = 0;
  static const int TIMEOUT = 1;

  CommandError()
  {
    this->id = CommandError::UNKNOWN;
    this->message = "";
  }

  CommandError(int id, std::string message)
  {
    this->id = id;
    this->message = message;
  }
  int id;
  std::string message;
};

typedef std::function<void(std::shared_ptr<CommandError> error, std::shared_ptr<Message> response)> GripperCallback;

class CommandSubscription
{
public:
  char messageId;
  int listenerId;
};

class GripperState
{
public:
  float width;
  float current_speed;
  float current_force;
  float configured_acceleration;
  float configured_force;
  int32_t grasping_state;
  int32_t system_state;
  ConnectionState connection_state;

  GripperState()
  {
    this->current_force = -1;
    this->current_speed = -1;
    this->width = -1;
    this->configured_force = -1;
    this->configured_acceleration = -1;
    this->grasping_state = -1;
    this->system_state = -1;
    this->connection_state = ConnectionState::NOT_CONNECTED;
  }

  std::string getGraspStateText()
  {
    switch (this->grasping_state)
    {
      case (wsg_50_common::Status::IDLE):
        return "IDLE";
      case (wsg_50_common::Status::GRASPING):
        return "GRASPING";
      case (wsg_50_common::Status::NO_PART_FOUND):
        return "NO_PART_FOUND";
      case (wsg_50_common::Status::PART_LOST):
        return "PART_LOST";
      case (wsg_50_common::Status::HOLDING):
        return "HOLDING";
      case (wsg_50_common::Status::RELEASING):
        return "RELEASING";
      case (wsg_50_common::Status::POSITIONING):
        return "POSITIONING";
      case (wsg_50_common::Status::ERROR):
        return "ERROR";
      case (wsg_50_common::Status::UNKNOWN):
        return "UNKNOWN";
    }
  }
};

class CommandState
{
public:
  CommandState();
  CommandState(Message& message, GripperCallback callback, int timeout_in_ms)
  {
    this->message = message;
    this->callback = callback;
    this->created = ros::Time::now();
    this->timeout_in_ms = timeout_in_ms;
  }

  Message message;
  GripperCallback callback;
  ros::Time created;
  int timeout_in_ms;
};

enum class WellKnownMessageId : unsigned char
{
  LOOP_BACK = 0x06,
  ANNOUNCE_DISCONNECT = 0x07,
  HOMING = 0x20,
  MOVE = 0x21,
  SOFT_STOP = 0x22,
  EMERGENCY_STOP = 0x23,
  ACK_ERROR = 0x24,
  GRASP = 0x25,
  RELEASE = 0x26,
  SET_ACCELERATION = 0x30,
  GET_ACCELERATION = 0x31,
  SET_GRASP_FORCE = 0x32,
  GET_GRASP_FORCE = 0x33,
  STATUS_VALUES = 0x40,
  GRIPPING_STATE = 0x41,
  OPENING_VALUES = 0x43,
  SPEED_VALUES = 0x44,
  FORCE_VALUES = 0x45,
  WILDCARD = 0xFA  // Use to register a listener which receives a callback whenever a message is received, regardless of
                   // the message id.
};

enum class CommandStateCode : unsigned char
{
  UNKNOWN = 0,
  PENDING = 1,
  SUCCESS = 2,
  ERROR = 3
};

class GripperCommunication final
{
public:
  static constexpr float TIMEOUT_DELAY = 2.5;
  GripperCommunication(std::string host, int port, int auto_update_frequency = 50, int default_timeout_in_ms = 10000);
  ~GripperCommunication()
  {
  }

  void sendCommand(Message& message, GripperCallback callback = nullptr, int timeout_in_ms = 0);
  void sendCommandSynchronous(Message& message, GripperCallback callback = nullptr, int timeout_in_ms = 1000);
  void awaitUpdateForMessage(unsigned char messageId, GripperCallback callback = nullptr, uint32_t amount = 1, int timeout_in_ms = 1000);
  CommandSubscription subscribe(unsigned char messageId, GripperCallback callback);
  void unregisterListener(unsigned char messageId, int listenerId);
  bool acceptsCommands(std::string& reason);
  void processMessages(int max_number_of_messages = 100);

  // long running, asynchronous gripper commands
  void move(float width, float speed, bool stop_on_block, GripperCallback callback = nullptr, int timeout_in_ms = 0);
  void grasp(float width, float speed, GripperCallback callback = nullptr, int timeout_in_ms = 0);
  void release(float width, float speed, GripperCallback callback = nullptr, int timeout_in_ms = 0);
  void homing(GripperCallback callback = nullptr, int timeout_in_ms = 0);

  // short running, synchronous calls to gripper
  void soft_stop(GripperCallback callback = nullptr, int timeout_in_ms = 1000);
  void fast_stop();
  void acknowledge_error(GripperCallback callback = nullptr, int timeout_in_ms = 1000);
  void set_force(float force, GripperCallback callback = nullptr, int timeout_in_ms = 1000);
  void set_acceleration(float acceleration);
  void requestValueUpdate(const unsigned char messageId, GripperCallback callback);

  void connectToGripper(std::string protocol, std::string ip, int port);
  void disconnectFromGripper(bool announceDisconnect);
  void valueUpdateCallback(std::shared_ptr<CommandError> error, std::shared_ptr<Message> message);
  void shutdown();
  // in the special case that pre-postition (move) is used to pick parts with an unkown width, it is needed that the gripper
  // does not report error states, otherwise our monitoring system might pick the error state up which might interfere with the program execution
  void setOverrideForGripperErrorState(bool active, int32_t alternative_state = wsg_50_common::Status::IDLE);

  GripperState getState();

  void activateAutomaticValueUpdates();

  bool lastCommandReturnedSuccess(const unsigned char messageId);
  int decodeStatus(Message& message);

  // delete copy and move constructors and assign operators
  GripperCommunication(const GripperCommunication&) = delete;             // Copy construct
  GripperCommunication(GripperCommunication&&) = delete;                  // Move construct
  GripperCommunication& operator=(const GripperCommunication&) = delete;  // Copy assign
  GripperCommunication& operator=(GripperCommunication&&) = delete;       // Move assign

  void forceCallback(Message& message);
  void speedCallback(Message& message);

private:
  void activateAutoUpdates(const unsigned char messageId, const int interval_ms);
  void callbackListeners(const unsigned char messageId, Message& message);
  void updateCommandState(const unsigned char messageId, const CommandStateCode state);

  void graspingStateCallback(Message& message);
  void widthCallback(Message& message);
  void requestConfiguredAcceleration();
  void requestConfiguredGraspingForce();

  GripperSocket* gripper_socket;
  std::map<unsigned char, std::map<int, GripperCallback> > callbacks;
  std::map<unsigned char, CommandStateCode> commandStates;
  std::shared_ptr<CommandState> currentCommand;
  std::vector<CommandSubscription> subscriptions;
  GripperState gripper_state;
  bool running;
  bool is_gripper_error_state_overwritten;
  int32_t alternative_gripper_error_state;
  int auto_update_frequency;
  int default_command_timeout_in_ms;
  ros::Time last_received_update;
};

class MessageQueueFull : public std::runtime_error
{
public:
  MessageQueueFull() : std::runtime_error("MessageQueueFull")
  {
  }
};

class MessageSendFailed : public std::runtime_error
{
public:
  MessageSendFailed() : std::runtime_error("MessageSendFailed")
  {
  }
};

class MessageReceiveFailed : public std::runtime_error
{
public:
  MessageReceiveFailed() : std::runtime_error("MessageReceiveFailed")
  {
  }
};

class MessageTimedOut : public std::runtime_error
{
public:
  MessageTimedOut() : std::runtime_error("MessageTimedOut")
  {
  }
};

class ProtocolNotSupported : public std::runtime_error
{
public:
  ProtocolNotSupported() : std::runtime_error("ProtocolNotSupported")
  {
  }
};

class ConnectionError : public std::runtime_error
{
public:
  ConnectionError() : std::runtime_error("ConnectionError")
  {
  }
};

#endif GRIPPER_COMM_H_

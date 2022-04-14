#include "wsg_50/gripper_communication.h"
#include "wsg_50/functions.h"

GripperCommunication::GripperCommunication(std::string host, int port, int auto_update_interval_ms,
                                          int command_timeout_ms, int reconnect_timeout_ms)
{
  gripper_socket = new GripperSocket(host, port);
  gripper_socket->setConnectionStateChangedCallback(
    [&](ConnectionState& new_connection_state)
    {
      this->connectionStateChangedCallback(new_connection_state);
    }
  );

  ROS_INFO("Starting with update interval: %d ms, reconnection delay: %d ms", auto_update_interval_ms, reconnect_timeout_ms);
  this->currentCommand = nullptr;
  this->command_timeout_ms = command_timeout_ms;
  this->auto_update_interval_ms = auto_update_interval_ms;
  this->reconnect_timeout_ms = reconnect_timeout_ms;
  this->is_gripper_error_state_overwritten = false;
  this->alternative_gripper_error_state = 0;
  last_received_update = ros::Time(0);
  running = true;
}

bool GripperCommunication::acceptsCommands(std::string& reason)
{
  if (this->currentCommand != nullptr)
  {
    reason = "Gripper is already executing a command";
    return false;
  }
  else if (this->running == false)
  {
    reason = "Gripper communication is shutting down";
    return false;
  }
  else if (this->getState().connection_state != ConnectionState::CONNECTED)
  {
    reason = "Gripper is not connected";
    return false;
  }

  return true;
}

void GripperCommunication::activateAutomaticValueUpdates()
{
  int interval_ms = this->auto_update_interval_ms;
  this->clearSubscriptions();
  std::chrono::milliseconds timespan(10);

  ROS_INFO("Request updates for grip state");
  auto subscription = this->subscribe((unsigned char)WellKnownMessageId::GRIPPING_STATE,
                                     [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                                       if ((error == nullptr) && (message != nullptr))
                                       {
                                         this->graspingStateCallback(*message.get());
                                       }
                                     });
  this->activateAutoUpdates((unsigned char)WellKnownMessageId::GRIPPING_STATE, interval_ms);
  this->subscriptions.push_back(subscription);
  std::this_thread::sleep_for(timespan);

  ROS_INFO("Request updates for opening values");
  subscription = this->subscribe((unsigned char)WellKnownMessageId::OPENING_VALUES,
                  [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                    if ((error == nullptr) && (message != nullptr))
                    {
                      this->widthCallback(*message.get());
                    }
                  });
  this->activateAutoUpdates((unsigned char)WellKnownMessageId::OPENING_VALUES, interval_ms + 2);
  this->subscriptions.push_back(subscription);
  std::this_thread::sleep_for(timespan);

  ROS_INFO("Request updates for force values");
  subscription = this->subscribe((unsigned char)WellKnownMessageId::FORCE_VALUES,
                  [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                    if ((error == nullptr) && (message != nullptr))
                    {
                      this->forceCallback(*message.get());
                    }
                  });
  this->activateAutoUpdates((unsigned char)WellKnownMessageId::FORCE_VALUES, interval_ms + 4);
  this->subscriptions.push_back(subscription);
  std::this_thread::sleep_for(timespan);

  ROS_INFO("Request updates for speed values");
  subscription = this->subscribe((unsigned char)WellKnownMessageId::SPEED_VALUES,
                  [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
                    if ((error == nullptr) && (message != nullptr))
                    {
                      this->speedCallback(*message.get());
                    }
                  });
  this->activateAutoUpdates((unsigned char)WellKnownMessageId::SPEED_VALUES, interval_ms + 6);
  this->subscriptions.push_back(subscription);
}

int GripperCommunication::decodeStatus(Message& message)
{
  if (message.length > 0)
  {
    auto status = (status_t)make_short(message.data[0], message.data[1]);
    return (int)status;
  }
  else
  {
    return -1;
  }
}

GripperState GripperCommunication::getState()
{
  return this->gripper_state;
}

void GripperCommunication::disconnectFromGripper(bool announceDisconnect)
{
  if (announceDisconnect)
  {
    Message m((unsigned char)WellKnownMessageId::ANNOUNCE_DISCONNECT, 0, nullptr);

    try
    {
      this->sendCommandSynchronous(m);
    }
    catch (...)
    {
      ROS_WARN("Could not announce disconnect to gripper. Will proceed in closing the connection. This might put the "
               "gripper into fast stop mode.");
    }
  }

  msg_close();
}

// @brief Grasps with the gripper
// @param width in mm
// @param speed in mm/s
void GripperCommunication::grasp(float width, float speed, GripperCallback callback, int timeout_in_ms)
{
  unsigned char payload_length = 8;
  unsigned char payload[payload_length];

  // Copy target width and speed
  memcpy(&payload[0], &width, sizeof(float));
  memcpy(&payload[4], &speed, sizeof(float));

  Message message((unsigned char)WellKnownMessageId::GRASP, payload_length, payload);

  this->sendCommand(message, callback, timeout_in_ms);
}

// @brief Releases the gripper
// @param width in mm
// @param speed in mm/s
void GripperCommunication::release(float width, float speed, GripperCallback callback, int timeout_in_ms)
{
  unsigned char payload_length = 8;
  unsigned char payload[payload_length];

  // Copy target width and speed
  memcpy(&payload[0], &width, sizeof(float));
  memcpy(&payload[4], &speed, sizeof(float));

  Message message((unsigned char)WellKnownMessageId::RELEASE, payload_length, payload);

  this->sendCommand(message, callback, timeout_in_ms);
}

// @brief Prepositioning the gripper
// @param width in mm
// @param speed in mm/s
void GripperCommunication::move(float width, float speed, bool stop_on_block, GripperCallback callback,
                                int timeout_in_ms)
{
  ROS_INFO("[GripperCommunication::move] w %f, s %f, stop %d", width, speed, stop_on_block);
  // auto message_ptr = this->createMoveMessage(width, speed, stop_on_block);

  unsigned char payload_length = 9;
  unsigned char payload[payload_length];

  // Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
  payload[0] = 0x00;
  if (stop_on_block)
    payload[0] |= 0x02;

  // Copy target width and speed
  memcpy(&payload[1], &width, sizeof(float));
  memcpy(&payload[5], &speed, sizeof(float));
  Message message((unsigned char)WellKnownMessageId::MOVE, payload_length, payload);

  this->sendCommand(message, callback, timeout_in_ms);
}

void GripperCommunication::requestValueUpdate(const unsigned char messageId, GripperCallback callback)
{
  auto message = Message(messageId, 0, nullptr);

  this->sendCommandSynchronous(message, callback, 1000);
}

CommandSubscription GripperCommunication::subscribe(unsigned char messageId, GripperCallback newCallback)
{
  auto listenersIterator = this->callbacks.find(messageId);

  if (listenersIterator == this->callbacks.end())
  {
    std::map<int, GripperCallback> emptyMap;
    auto pair = make_pair(messageId, emptyMap);
    this->callbacks.insert(pair);
    ROS_INFO("Inserted into callbacks: %d, size: %d", pair.first, this->callbacks.size());
  }

  auto& listeners = this->callbacks.find(messageId)->second;
  int listenerId = listeners.size();

  auto pair = make_pair(listenerId, newCallback);
  listeners.insert(pair);

  CommandSubscription listener;
  listener.messageId = messageId;
  listener.listenerId = listenerId;

  return listener;
}

void GripperCommunication::sendCommand(Message& message, GripperCallback callback, int timeout_in_ms)
{
  bool stop_command = message.id == (unsigned char)WellKnownMessageId::SOFT_STOP ||
                      message.id == (unsigned char)WellKnownMessageId::EMERGENCY_STOP;

  if ((stop_command == false) && (this->currentCommand != nullptr))
  {
    throw MessageQueueFull();
  }

  auto timeout = timeout_in_ms > 0 ? timeout_in_ms : this->command_timeout_ms;
  this->currentCommand = std::make_shared<CommandState>(CommandState(message, callback, timeout));
  this->updateCommandState(message.id, CommandStateCode::PENDING);
  this->gripper_socket->sendMessage(message);

  ROS_INFO("Sent async command: %d, len: %d", message.id, message.length);
}

void GripperCommunication::sendCommandSynchronous(Message& message, GripperCallback callback, int timeout_in_ms)
{
  ROS_INFO("Send command synchronous %d", message.id);
  this->gripper_socket->sendMessage(message);
  this->awaitUpdateForMessage(message.id, callback, 1, timeout_in_ms);
}

void GripperCommunication::awaitUpdateForMessage(unsigned char messageId, GripperCallback callback, uint32_t amount, int timeout_in_ms) {
  Message received;
  auto start_time = ros::Time::now().toSec();
  bool received_response = false;
  uint32_t received_amount = 0;
  auto subscription =
      this->subscribe(messageId, [&](std::shared_ptr<CommandError> error, std::shared_ptr<Message> message) {
        if (callback != nullptr)
        {
          callback(error, message);
        }
        received_amount++;
        received_response = received_amount >= amount;
      });
  do
  {
    processMessages();
    if (received_response == true)
    {
      break;
    }

    std::chrono::milliseconds timespan(10);
    std::this_thread::sleep_for(timespan);

    if ((ros::Time::now().toSec() - start_time) * 1000 > timeout_in_ms)
    {
      this->unregisterListener(subscription.messageId, subscription.listenerId);
      throw MessageTimedOut();
    }
  } while (received_response == false);

  this->unregisterListener(subscription.messageId, subscription.listenerId);
}

void GripperCommunication::unregisterListener(unsigned char messageId, int listenerId)
{
  auto listenersIterator = this->callbacks.find(messageId);

  if (listenersIterator != this->callbacks.end())
  {
    auto& listeners = listenersIterator->second;
    auto it = listeners.find(listenerId);
    if (it != listeners.end())
    {
      ROS_INFO("Remove listener for message id: %d, listener id: %d", messageId, listenerId);
      listeners.erase(it);
    }
  }
}

void GripperCommunication::connectionStateChangedCallback(ConnectionState& new_connection_state)
{
  this->gripper_state.connection_state = new_connection_state;
  this->gripper_state.initialized = false;
}

void GripperCommunication::initializeSubscriptions()
{
  if (this->gripper_state.connection_state == ConnectionState::CONNECTED)
  {
    this->last_received_update = ros::Time::now();
    try
    {
      this->activateAutomaticValueUpdates();
      this->requestConfiguredAcceleration();
      this->requestConfiguredGraspingForce();
      this->gripper_state.initialized = true;
      ROS_INFO("Gripper initialized successfully.");
    }
    catch (std::runtime_error& ex)
    {
      ROS_ERROR("Could not initialize subscriptions: %s", ex.what());
    }
  }
}

void GripperCommunication::doWork()
{
  if (this->gripper_state.connection_state == ConnectionState::CONNECTED)
  {
    if (this->gripper_state.initialized == true)
    {
      this->processMessages();
    }
    else
    {
      this->initializeSubscriptions();
    }

    double time_diff = (ros::Time::now().toSec() - this->last_received_update.toSec()) * 1000;
    if (time_diff > this->reconnect_timeout_ms)
    {
      ROS_WARN("Did not receive any updates from gripper within %f ms. Initiate reconnect.", time_diff);
      this->last_received_update = ros::Time::now();
      this->gripper_socket->reconnect();
    }
  }
}

void GripperCommunication::processMessages(int max_number_of_messages)
{
  int processed_messages = 0;
  std::shared_ptr<Message> message = this->gripper_socket->getMessage();

  while ((message != nullptr) && (processed_messages < max_number_of_messages))
  {  // message available
    GripperCallback callback = nullptr;
    if ((this->currentCommand != nullptr) && (this->currentCommand.get()->message.id == message.get()->id))
    {
      auto status = (status_t)make_short(message.get()->data[0], message.get()->data[1]);
      if (status != E_CMD_PENDING)
      {
        ROS_INFO("Clear message id %d", message.get()->id);
        callback = this->currentCommand.get()->callback;
        this->currentCommand = nullptr;

        if (status == E_SUCCESS)
        {
          this->updateCommandState(message.get()->id, CommandStateCode::SUCCESS);
        }
        else
        {
          this->updateCommandState(message.get()->id, CommandStateCode::ERROR);
        }
      }
    }

    this->callbackListeners((unsigned char)WellKnownMessageId::WILDCARD, *message.get());
    this->callbackListeners(message.get()->id, *message.get());

    if (callback != nullptr)
    {
      callback(nullptr, message);
    }

    processed_messages += 1;
    message = this->gripper_socket->getMessage();
  }

  if (this->currentCommand != nullptr)
  {
    if ((ros::Time::now().toSec() - this->currentCommand.get()->created.toSec()) * 1000 >
        this->currentCommand.get()->timeout_in_ms)
    {
      ROS_WARN("Command with message id %d has timed out", (int)this->currentCommand.get()->message.id);
      if (this->currentCommand.get()->callback != nullptr)
      {
        auto error = std::make_shared<CommandError>(
            CommandError(CommandError::TIMEOUT,
                         "Message id " + std::to_string(this->currentCommand.get()->message.id) + " has timed out."));
        this->currentCommand.get()->callback(error, nullptr);
      }
      this->currentCommand = nullptr;
    }
  }
}

bool GripperCommunication::lastCommandReturnedSuccess(const unsigned char messageId)
{
  auto commandStateIterator = this->commandStates.find(messageId);
  if (commandStateIterator == this->commandStates.end())
  {
    return false;
  }
  else
  {
    return commandStateIterator->second == CommandStateCode::SUCCESS;
  }
}

void GripperCommunication::callbackListeners(const unsigned char messageId, Message& message)
{
  auto listenersIterator = this->callbacks.find(messageId);
  if (listenersIterator != this->callbacks.end())
  {
    auto listeners = listenersIterator->second;
    for (auto& keyValue : listeners)
    {
      auto m = std::make_shared<Message>(message);
      keyValue.second(nullptr, m);
    }
  }
}

void GripperCommunication::activateAutoUpdates(const unsigned char messageId, const int interval_ms)
{
  unsigned char payload[3];
  memset(payload, 0, 3);
  if (interval_ms > 0)
  {
    payload[0] = 0x01;
    payload[1] = (interval_ms & 0xff);
    payload[2] = ((interval_ms & 0xff00) >> 8);
  }

  Message message(messageId, 3, payload);

  this->sendCommandSynchronous(message);
}

void GripperCommunication::graspingStateCallback(Message& message)
{
  if (message.length > 0)
  {
    auto status = (status_t)make_short(message.data[0], message.data[1]);
    if (status == E_SUCCESS)
    {
      this->last_received_update = ros::Time::now();
      this->gripper_state.grasping_state = message.data[2];
      if (this->is_gripper_error_state_overwritten && this->gripper_state.grasping_state == wsg_50_common::Status::ERROR) {
        this->gripper_state.grasping_state = this->alternative_gripper_error_state;
      }
    }
  }
}

void GripperCommunication::speedCallback(Message& message)
{
  if (message.length > 0)
  {
    auto status = (status_t)make_short(message.data[0], message.data[1]);
    if (status == E_SUCCESS)
    {
      this->last_received_update = ros::Time::now();
      float speed_values = convert(&message.data[2]);
      this->gripper_state.current_speed = speed_values;
    }
  }
}

void GripperCommunication::forceCallback(Message& message)
{
  if (message.length > 0)
  {
    auto status = (status_t)make_short(message.data[0], message.data[1]);
    if (status == E_SUCCESS)
    {
      this->last_received_update = ros::Time::now();
      float force_values = convert(&message.data[2]);
      this->gripper_state.current_force = force_values;
    }
  }
}

void GripperCommunication::requestConfiguredAcceleration()
{
  this->requestValueUpdate((unsigned char)WellKnownMessageId::GET_ACCELERATION,
                           [&](auto error, auto message) { this->valueUpdateCallback(error, message); });
}

void GripperCommunication::requestConfiguredGraspingForce()
{
  this->requestValueUpdate((unsigned char)WellKnownMessageId::GET_GRASP_FORCE,
                           [&](auto error, auto message) { this->valueUpdateCallback(error, message); });
}

void GripperCommunication::shutdown()
{
  this->running = false;
  this->disconnectFromGripper(true);
  this->clearSubscriptions();
}

void GripperCommunication::clearSubscriptions()
{
  for (auto it = this->subscriptions.begin(); it != this->subscriptions.end(); ++it)
  {
    this->unregisterListener(it->messageId, it->listenerId);
  }
  this->subscriptions.clear();
}

void GripperCommunication::valueUpdateCallback(std::shared_ptr<CommandError> error, std::shared_ptr<Message> message)
{
  if ((error == nullptr) && (message != nullptr))
  {
    auto msg = message.get();
    auto status = (status_t)make_short(msg->data[0], msg->data[1]);
    if (status == E_SUCCESS)
    {
      float value = convert(&message->data[2]);
      if (msg->id == (unsigned char)WellKnownMessageId::GET_ACCELERATION)
      {
        this->gripper_state.configured_acceleration = value;
      }
      else if (msg->id == (unsigned char)WellKnownMessageId::GET_GRASP_FORCE)
      {
        this->gripper_state.configured_force = value;
      }
    }
  }
}

void GripperCommunication::widthCallback(Message& message)
{
  if (message.length > 0)
  {
    auto status = (status_t)make_short(message.data[0], message.data[1]);
    if (status == E_SUCCESS)
    {
      this->last_received_update = ros::Time::now();
      float opening_value = convert(&message.data[2]);
      this->gripper_state.width = opening_value;
    }
  }
}

void GripperCommunication::updateCommandState(unsigned char messageId, const CommandStateCode state)
{
  auto commandStateIterator = this->commandStates.find(messageId);
  if (commandStateIterator == this->commandStates.end())
  {
    this->commandStates.insert(std::make_pair(messageId, state));
  }
  else
  {
    commandStateIterator->second = state;
  }
}

void GripperCommunication::homing(GripperCallback callback, int timeout_in_ms)
{
  unsigned char payload[1];
  payload[0] = 0x00;
  Message m((unsigned char)WellKnownMessageId::HOMING, 1, payload);

  this->sendCommand(m, callback, timeout_in_ms);
}

void GripperCommunication::acknowledge_error(GripperCallback callback, int timeout_in_ms)
{
  unsigned char payload[3];
  payload[0] = 0x61;
  payload[1] = 0x63;
  payload[2] = 0x6B;

  Message m((unsigned char)WellKnownMessageId::ACK_ERROR, 3, payload);

  this->sendCommandSynchronous(m, callback, timeout_in_ms);
}

void GripperCommunication::soft_stop(GripperCallback callback, int timeout_in_ms)
{
  Message m((unsigned char)WellKnownMessageId::SOFT_STOP, 0, nullptr);

  this->sendCommandSynchronous(m, callback, timeout_in_ms);
}

void GripperCommunication::fast_stop()
{
  Message m((unsigned char)WellKnownMessageId::EMERGENCY_STOP, 0, nullptr);

  this->sendCommandSynchronous(m);
}

// @brief Sets the force
// @param force in N
void GripperCommunication::set_force(float force, GripperCallback callback, int timeout_in_ms)
{
  unsigned char payload[4];
  memcpy(&payload[0], &force, sizeof(float));

  Message m((unsigned char)WellKnownMessageId::SET_GRASP_FORCE, 4, payload);

  this->sendCommandSynchronous(m, callback, timeout_in_ms);
  this->requestConfiguredGraspingForce();
}

// @brief Sets the acceleration
// @param acceleration in mm/s²
void GripperCommunication::set_acceleration(float acceleration)
{
  unsigned char payload[4];
  memcpy(&payload[0], &acceleration, sizeof(float));

  Message m((unsigned char)WellKnownMessageId::SET_ACCELERATION, 4, payload);

  this->sendCommandSynchronous(m);
  this->requestConfiguredAcceleration();
}


void GripperCommunication::setOverrideForGripperErrorState(bool active, int32_t alternative_state) {
  this->is_gripper_error_state_overwritten = active;
  this->alternative_gripper_error_state = alternative_state;
}
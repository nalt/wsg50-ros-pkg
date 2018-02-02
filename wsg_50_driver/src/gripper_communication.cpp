#include "wsg_50/gripper_communication.h"
#include "wsg_50/functions.h"


GripperCommunication::GripperCommunication(std::string host, int port) {
	gripper_socket = new GripperSocket(host, port);

	this->currentCommand = nullptr;
	gripper_state.force = -1;
	gripper_state.speed = -1;
	gripper_state.width = -1;
	gripper_state.grasping_state = -1;
	gripper_state.system_state = -1;
	gripper_state.connection_state = ConnectionState::NOT_CONNECTED;
	running = true;
}

bool GripperCommunication::acceptsCommands() {
	return (this->currentCommand == nullptr) && (this->running == true)
			&& (this->getState().connection_state == ConnectionState::CONNECTED);
}

void GripperCommunication::activateAutomaticValueUpdates() {
	int interval_ms = 94;

	printf("Request updates for grip state\n");
	auto subcription = this->subscribe(
			(unsigned char) WellKnownMessageId::GRIPPING_STATE,
			[&](Message& message) {this->graspingStateCallback(message);});
	this->activateAutoUpdates(
			(unsigned char) WellKnownMessageId::GRIPPING_STATE, interval_ms);
	this->subscriptions.push_back(subcription);

	printf("Request updates for opening values\n");
	this->subscribe((unsigned char) WellKnownMessageId::OPENING_VALUES,
			[&](Message& message) {this->widthCallback(message);});
	this->activateAutoUpdates(
			(unsigned char) WellKnownMessageId::OPENING_VALUES,
			interval_ms + 2);
	this->subscriptions.push_back(subcription);

	printf("Request updates for force values\n");
	this->subscribe((unsigned char) WellKnownMessageId::FORCE_VALUES,
			[&](Message& m) {this->forceCallback(m);});
	this->activateAutoUpdates((unsigned char) WellKnownMessageId::FORCE_VALUES,
			interval_ms + 4);
	this->subscriptions.push_back(subcription);

	printf("Request updates for speed values\n");
	this->subscribe((unsigned char) WellKnownMessageId::SPEED_VALUES,
			[&](Message& m) {this->speedCallback(m);});
	this->activateAutoUpdates((unsigned char) WellKnownMessageId::SPEED_VALUES,
			interval_ms + 6);
	this->subscriptions.push_back(subcription);
}

int GripperCommunication::decodeStatus(Message& message) {
	if (message.length > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		return (int) status;
	} else {
		return -1;
	}
}

GripperState GripperCommunication::getState() {
	return this->gripper_state;
}

void GripperCommunication::disconnectFromGripper(bool announceDisconnect) {
	if (announceDisconnect) {
		Message m((unsigned char) WellKnownMessageId::ANNOUNCE_DISCONNECT, 0, nullptr);

		try {
			this->sendCommandSynchronous(m);
		} catch (...) {
			ROS_WARN("Could not announce disconnect to gripper. Will proceed in closing the connection. This might put the gripper into fast stop mode.");
		}
	}

	msg_close();
}

void GripperCommunication::grasp(float width, float speed,
		GripperCallback callback) {
	unsigned char payload_length = 8;
	unsigned char payload[payload_length];

	// Copy target width and speed
	memcpy(&payload[0], &width, sizeof(float));
	memcpy(&payload[4], &speed, sizeof(float));

	Message message((unsigned char) WellKnownMessageId::GRASP, payload_length, payload);

	this->sendCommand(message, callback);
}

void GripperCommunication::release(float width, float speed,
		GripperCallback callback) {
	unsigned char payload_length = 8;
	unsigned char payload[payload_length];

	// Copy target width and speed
	memcpy(&payload[0], &width, sizeof(float));
	memcpy(&payload[4], &speed, sizeof(float));

	Message message((unsigned char) WellKnownMessageId::RELEASE, payload_length, payload);

	this->sendCommand(message, callback);
}

void GripperCommunication::move(float width, float speed, bool stop_on_block,
		GripperCallback callback) {
	printf("Move w %f, s %f\n", width, speed);
	//auto message_ptr = this->createMoveMessage(width, speed, stop_on_block);

	unsigned char payload_length = 9;
	unsigned char payload[payload_length];

	// Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
	payload[0] = 0x00;
	if (stop_on_block)
		payload[0] |= 0x02;

	// Copy target width and speed
	memcpy(&payload[1], &width, sizeof(float));
	memcpy(&payload[5], &speed, sizeof(float));
	Message message((unsigned char) WellKnownMessageId::MOVE, payload_length, payload);

	this->sendCommand(message, callback);
}

CommandSubscription GripperCommunication::subscribe(unsigned char messageId,
		GripperCallback newCallback) {
	auto listenersIterator = this->callbacks.find(messageId);

	if (listenersIterator == this->callbacks.end()) {
		std::map<int, GripperCallback> emptyMap;
		auto pair = make_pair(messageId, emptyMap);
		this->callbacks.insert(pair);
		printf("Inserted into callbacks: %d, size: %d\n", pair.first,
				this->callbacks.size());
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

void GripperCommunication::sendCommand(Message& message,
		GripperCallback callback) {
	bool stop_command = message.id
			== (unsigned char) WellKnownMessageId::SOFT_STOP
			|| message.id == (unsigned char) WellKnownMessageId::EMERGENCY_STOP;

	if ((stop_command == false) && (this->currentCommand != nullptr)) {
		throw MessageQueueFull();
	}

	this->currentCommand = std::make_shared < CommandState
			> (CommandState(message, callback));
	this->gripper_socket->sendMessage(message);

	this->updateCommandState(message.id, CommandStateCode::PENDING);

	printf("-- Send message id: %d, len: %d\n", message.id, message.length);
}

void GripperCommunication::sendCommandSynchronous(Message& message,
		int timeout_in_ms) {
	printf("--> send: %d\n", message.id);
	this->gripper_socket->sendMessage(message);

	Message received;
	auto start_time = ros::Time::now().toSec();
	bool received_response = false;
	auto subscription = this->subscribe(message.id, [&](Message& m){received_response = true;});
	do {
		processMessages();

		std::chrono::milliseconds timespan(1);
		std::this_thread::sleep_for(timespan);

		if ((ros::Time::now().toSec() - start_time) * 1000 > timeout_in_ms) {
			this->unregisterListener(subscription.messageId, subscription.listenerId);
			throw MessageTimedOut();
		}
	} while (received_response == false);

	this->unregisterListener(subscription.messageId, subscription.listenerId);
}

void GripperCommunication::unregisterListener(unsigned char messageId,
		int listenerId) {
	auto listenersIterator = this->callbacks.find(messageId);

	if (listenersIterator != this->callbacks.end()) {
		auto& listeners = listenersIterator->second;
		auto it = listeners.find(listenerId);
		if (it != listeners.end()) {
			printf("-- Remove listener for message id: %d, listener id: %d\n",
					messageId, listenerId);
			listeners.erase(it);
		}
	}
}

void GripperCommunication::processMessages(int max_number_of_messages) {
	int processed_messages = 0;
	std::shared_ptr<Message> message = this->gripper_socket->getMessage();

	if (this->gripper_state.connection_state != this->gripper_socket->getConnectionState()) {
		this->gripper_state.connection_state = this->gripper_socket->getConnectionState();
		if (this->gripper_socket->getConnectionState() == ConnectionState::CONNECTED) {
			//this->activateAutomaticValueUpdates();
		}
	}

	while ((message != nullptr) && (processed_messages < max_number_of_messages)) { // message available
		//printf("-- Received id: %d, len: %d\n", message.id, message.len);

		GripperCallback callback = nullptr;
		if ((this->currentCommand != nullptr)
				&& (this->currentCommand.get()->message.id == message.get()->id)) {
			auto status = (status_t) make_short(message.get()->data[0], message.get()->data[1]);
			if (status != E_CMD_PENDING) {
				printf("-- Clear message id %d\n", message.get()->id);
				callback = this->currentCommand.get()->callback;
				this->currentCommand = nullptr;

				if (status == E_SUCCESS) {
					this->updateCommandState(message.get()->id, CommandStateCode::SUCCESS);
				} else {
					this->updateCommandState(message.get()->id, CommandStateCode::ERROR);
				}
			}
		}

		this->callbackListeners((unsigned char) WellKnownMessageId::WILDCARD,
				*message.get());
		this->callbackListeners(message.get()->id, *message.get());

		if (callback != nullptr) {
			callback(*message.get());
		}

		processed_messages += 1;
		message = this->gripper_socket->getMessage();
	}
}

bool GripperCommunication::lastCommandReturnedSuccess(
		const unsigned char messageId) {
	auto commandStateIterator = this->commandStates.find(messageId);
	if (commandStateIterator == this->commandStates.end()) {
		return false;
	} else {
		return commandStateIterator->second == CommandStateCode::SUCCESS;
	}
}

void GripperCommunication::callbackListeners(const unsigned char messageId,
		Message& message) {
	auto listenersIterator = this->callbacks.find(messageId);
	if (listenersIterator != this->callbacks.end()) {
		auto listeners = listenersIterator->second;
		for (auto& keyValue : listeners) {
			keyValue.second(message);
		}
	}
}

void GripperCommunication::activateAutoUpdates(const unsigned char messageId,
		const int interval_ms) {
	// Payload = 0, except for auto update
	unsigned char payload[3];
	memset(payload, 0, 3);
	if (interval_ms > 0) {
		payload[0] = 0x01;
		payload[1] = (interval_ms & 0xff);
		payload[2] = ((interval_ms & 0xff00) >> 8);
	}

	Message message(messageId, 3, payload);

	this->sendCommandSynchronous(message);
}

void GripperCommunication::requestValueUpdate(const unsigned char messageId) {
	//printf("Request id: %d\n", messageId);
	// Payload = 0, except for auto update
	unsigned char payload[3];
	memset(payload, 0, 3);

	Message message(messageId, 3, payload);

	this->gripper_socket->sendMessage(message);
}

void GripperCommunication::graspingStateCallback(Message& message) {
	if (message.length > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			this->gripper_state.grasping_state = message.data[2];
			//printf("-- Gripper State: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::speedCallback(Message& message) {
	if (message.length > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			float speed_values = convert(&message.data[2]);
			this->gripper_state.speed = speed_values;
			//printf("-- Gripper Opening: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::forceCallback(Message& message) {
	if (message.length > 0) {
		//printf("Force callback id: %d\n", message.id);
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			float force_values = convert(&message.data[2]);
			this->gripper_state.force = force_values;
			//printf("-- Gripper Force: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::shutdown() {
	this->running = false;
	this->disconnectFromGripper(true);
	for (auto it = this->subscriptions.begin(); it != this->subscriptions.end();
			++it) {
		this->unregisterListener(it->messageId, it->listenerId);
	}
}

void GripperCommunication::widthCallback(Message& message) {
	if (message.length > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			float opening_value = convert(&message.data[2]);
			this->gripper_state.width = opening_value;
			//printf("-- Gripper Opening: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::updateCommandState(unsigned char messageId,
		const CommandStateCode state) {
	auto commandStateIterator = this->commandStates.find(messageId);
	if (commandStateIterator == this->commandStates.end()) {
		this->commandStates.insert(std::make_pair(messageId, state));
	} else {
		commandStateIterator->second = state;
	}
}

void GripperCommunication::homing(GripperCallback callback) {
	unsigned char payload[1];
	payload[0] = 0x00;
	Message m((unsigned char) WellKnownMessageId::HOMING, 1, payload);

	this->sendCommand(m, callback);
}

void GripperCommunication::acknowledge_error() {
	unsigned char payload[3];
	payload[0] = 0x61;
	payload[1] = 0x63;
	payload[2] = 0x6B;

	Message m((unsigned char) WellKnownMessageId::ACK_ERROR, 3, payload);

	this->sendCommandSynchronous(m);
}

void GripperCommunication::soft_stop() {
	Message m((unsigned char) WellKnownMessageId::SOFT_STOP, 0, nullptr);

	this->sendCommandSynchronous(m);
}

void GripperCommunication::fast_stop() {
	Message m((unsigned char) WellKnownMessageId::EMERGENCY_STOP, 0, nullptr);

	this->sendCommandSynchronous(m);
}

void GripperCommunication::set_force(float force) {
	unsigned char payload[4];
	memcpy(&payload[0], &force, sizeof(float));

	Message m((unsigned char) WellKnownMessageId::SET_GRASP_FORCE, 4, payload);

	this->sendCommandSynchronous(m);
}

void GripperCommunication::set_acceleration(float acceleration) {
	unsigned char payload[4];
	memcpy(&payload[0], &acceleration, sizeof(float));

	Message m((unsigned char) WellKnownMessageId::SET_ACCELERATION, 4, payload);

	this->sendCommandSynchronous(m);
}


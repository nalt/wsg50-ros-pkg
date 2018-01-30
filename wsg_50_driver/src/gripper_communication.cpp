#include "wsg_50/gripper_communication.h"
#include "wsg_50/functions.h"
#include <chrono>
#include <thread>

bool GripperCommunication::acceptsCommands() {
	return this->currentCommand == nullptr;
}

void GripperCommunication::activateOpeningValueUpdates(const int interval_ms) {
	printf("Request updates for opening values\n");
	this->subscribe(
					(unsigned char) WellKnownMessageId::OPENING_VALUES,
					[&](msg_t& message){this->widthCallback(message);});
	this->activateAutoUpdates(
			(unsigned char) WellKnownMessageId::OPENING_VALUES, interval_ms);
}

void GripperCommunication::activateGripStateUpdates(const int interval_ms) {
	printf("Request updates for grip state\n");
	this->subscribe(
					(unsigned char) WellKnownMessageId::GRIPPING_STATE,
					[&](msg_t& message){this->graspingStateCallback(message);});
	this->activateAutoUpdates(
			(unsigned char) WellKnownMessageId::GRIPPING_STATE, interval_ms);
}


int GripperCommunication::decodeStatus(msg_t& message) {
	if (message.len > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		return (int) status;
	} else {
		return -1;
	}
}

GripperState GripperCommunication::getState() {
	return this->gripper_state;
}

void GripperCommunication::grasp(float width, float speed,
		GripperCallback callback) {
	unsigned char payload_length = 8;
	unsigned char payload[payload_length];

	// Copy target width and speed
	memcpy(&payload[0], &width, sizeof(float));
	memcpy(&payload[4], &speed, sizeof(float));

	msg_t message;
	message.id = (unsigned char) WellKnownMessageId::GRASP;
	message.len = payload_length;
	message.data = &payload[0];

	this->sendCommand(message, callback);
}

void GripperCommunication::release(float width, float speed,
		GripperCallback callback) {
	unsigned char payload_length = 8;
	unsigned char payload[payload_length];

	// Copy target width and speed
	memcpy(&payload[0], &width, sizeof(float));
	memcpy(&payload[4], &speed, sizeof(float));

	msg_t message;
	message.id = (unsigned char) WellKnownMessageId::RELEASE;
	message.len = payload_length;
	message.data = &payload[0];

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
	msg_t message;
	message.id = (unsigned char) WellKnownMessageId::MOVE;
	message.len = payload_length;
	message.data = &payload[0];

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

void GripperCommunication::sendCommand(msg_t& message,
		GripperCallback callback) {
	bool stop_command = message.id
			== (unsigned char) WellKnownMessageId::SOFT_STOP
			|| message.id == (unsigned char) WellKnownMessageId::EMERGENCY_STOP;

	if ((stop_command == false) && (this->currentCommand != nullptr)) {
		throw MessageQueueFull();
	}

	this->currentCommand = std::make_shared < CommandState
			> (CommandState(message, callback));
	int result = msg_send(&message);

	if (result == -1) {
		throw MessageSendFailed();
	}

	this->updateCommandState(message.id, CommandStateCode::PENDING);

	printf("-- Send message id: %d, len: %d\n", message.id, message.len);
}

void GripperCommunication::sendCommandSynchronous(msg_t& message) {
	int result = msg_send(&message);

	if (result == -1) {
		throw MessageSendFailed();
	}

	msg_t received;
	do {
		received = processMessages();
		printf("-- received id %d\n", received.id);
		std::chrono::milliseconds timespan(10);
		std::this_thread::sleep_for(timespan);
	} while (message.id != received.id);
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

msg_t GripperCommunication::processMessages() {
	msg_t message;

	int result = msg_receive(&message);

	if (result == -1) {
		throw MessageReceiveFailed();
	}

	//printf("-- Received id: %d, len: %d\n", message.id, message.len);

	GripperCallback callback = nullptr;
	if ((this->currentCommand != nullptr)
			&& (this->currentCommand.get()->message.id == message.id)) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status != E_CMD_PENDING) {
			printf("-- Clear message id %d\n", message.id);
			callback = this->currentCommand.get()->callback;
			this->currentCommand = nullptr;

			if (status == E_SUCCESS) {
				this->updateCommandState(message.id, CommandStateCode::SUCCESS);
			} else {
				this->updateCommandState(message.id, CommandStateCode::ERROR);
			}
		}
	}

	this->callbackListeners((unsigned char) WellKnownMessageId::WILDCARD,
			message);
	this->callbackListeners(message.id, message);

	if (callback != nullptr) {
		callback(message);
	}

	return message;
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
		msg_t& message) {
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

	msg_t message;
	message.id = messageId;
	message.len = 3;
	message.data = payload;

	this->sendCommandSynchronous(message);
}

void GripperCommunication::requestValueUpdate(const unsigned char messageId) {
	if (messageId == (unsigned char)WellKnownMessageId::FORCE_VALUES) {
		if (!requestForceToggle) {
			return;
		} else {
			requestForceToggle = false;
		}
	}
	if (messageId == (unsigned char)WellKnownMessageId::SPEED_VALUES) {
		if (!requestSpeedToggle) {
			return;
		} else {
			requestSpeedToggle = false;
		}
	}

	//printf("Request id: %d\n", messageId);
	// Payload = 0, except for auto update
	unsigned char payload[3];
	memset(payload, 0, 3);

	msg_t message;
	message.id = messageId;
	message.len = 3;
	message.data = payload;

	msg_send(&message);
}


void GripperCommunication::graspingStateCallback(msg_t& message) {
	if (message.len > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			this->gripper_state.grasping_state = message.data[2];
			//printf("-- Gripper State: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::speedCallback(msg_t& message) {
	requestSpeedToggle = true;
	if (message.len > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			float speed_values = convert(&message.data[2]);
			this->gripper_state.speed = speed_values;
			//printf("-- Gripper Opening: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::forceCallback(msg_t& message) {
	requestForceToggle = true;
	if (message.len > 0) {
		//printf("Force callback id: %d\n", message.id);
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			float force_values = convert(&message.data[2]);
			this->gripper_state.force = force_values;
			//printf("-- Gripper Force: id: %d, len: %d, state: %d\n", message.id, message.len, message.data[2]);
		}
	}
}

void GripperCommunication::widthCallback(msg_t& message) {
	if (message.len > 0) {
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
	//auto m = this->createHomingMessage().toMessage();
	unsigned char payload[1];
	payload[0] = 0x00;
	msg_t m;
	m.id = (unsigned char) WellKnownMessageId::HOMING;
	m.len = 1;
	m.data = payload;

	this->sendCommand(m, callback);
}

void GripperCommunication::acknowledge_error() {
	//auto m = this->createAcknowledgeMessage().toMessage();
	unsigned char payload[3];
	payload[0] = 0x61;
	payload[1] = 0x63;
	payload[2] = 0x6B;

	msg_t m;
	m.id = (unsigned char) WellKnownMessageId::ACK_ERROR;
	m.len = 3;
	m.data = payload;

	this->sendCommandSynchronous(m);
}

void GripperCommunication::soft_stop() {
	//auto m = this->createSoftStopMessage().toMessage();
	unsigned char payload[0];
	msg_t m;
	m.id = (unsigned char) WellKnownMessageId::SOFT_STOP;
	m.len = 0;
	m.data = payload;

	this->sendCommandSynchronous(m);
}

void GripperCommunication::fast_stop() {
	//auto m = this->createEmergencyStopMessage().toMessage();
	unsigned char payload[0];
	msg_t m;
	m.id = (unsigned char) WellKnownMessageId::EMERGENCY_STOP;
	m.len = 0;
	m.data = payload;

	this->sendCommandSynchronous(m);
}

void GripperCommunication::set_force(float force) {
	//auto m = this->createSetForceMessage(force).toMessage();
	unsigned char payload[4];
	memcpy( &payload[0], &force, sizeof( float ) );

	msg_t m;
	m.id = (unsigned char) WellKnownMessageId::SET_GRASP_FORCE;
	m.len = 4;
	m.data = payload;

	this->sendCommandSynchronous(m);
}

void GripperCommunication::set_acceleration(float acceleration) {
	//auto m = this->createSetAccelerationMessage(acceleration).toMessage();
	unsigned char payload[4];
	memcpy( &payload[0], &acceleration, sizeof( float ) );

	msg_t m;
	m.id = (unsigned char) WellKnownMessageId::SET_ACCELERATION;
	m.len = 4;
	m.data = payload;

	this->sendCommandSynchronous(m);
}



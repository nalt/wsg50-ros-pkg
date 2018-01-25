#include "wsg_50/gripper_communication.h"

// First Or Default for super convenient C++ Maps
// thanks https://stackoverflow.com/a/2333816
template<template<class, class, class ...> class C, typename K, typename V, typename... Args>
V GetWithDef(const C<K,V,Args...>& m, K const& key, const V & defval)
{
	typename C<K,V,Args...>::const_iterator it = m.find( key );
	if (it == m.end())
	return defval;
	return it->second;
}

bool GripperCommunication::acceptsCommands() {
	return this->currentCommand == nullptr;
}

void GripperCommunication::activateOpeningValueUpdates(const int interval_ms) {
	printf("Request updates for opening values\n");
	this->activateAutoUpdates(
			(unsigned char) WellKnownMessageId::OPENING_VALUES, interval_ms);
}

void GripperCommunication::activateGripStateUpdates(const int interval_ms) {
	printf("Request updates for grip state\n");
	this->activateAutoUpdates(
			(unsigned char) WellKnownMessageId::GRIPPING_STATE, interval_ms);
}

void GripperCommunication::move(float width, float speed, bool stop_on_block, GripperCallback callback) {
	unsigned char payload[9];

	// Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
	payload[0] = 0x00;
	if (stop_on_block)
		payload[0] |= 0x02;

	// Copy target width and speed
	memcpy(&payload[1], &width, sizeof(float));
	memcpy(&payload[5], &speed, sizeof(float));

	msg_t message;
	message.id = (unsigned char) WellKnownMessageId::MOVE;
	message.len = 9;
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

void GripperCommunication::sendCommand(msg_t& message, GripperCallback callback) {
	if (this->currentCommand != nullptr) {
		throw MessageQueueFull();
	}

	this->currentCommand = std::make_shared < CommandState > (CommandState(message, callback));
	int result = msg_send(&message);

	if (result == -1) {
		throw MessageSendFailed();
	}

	this->updateCommandState(message.id, CommandStateCode::PENDING);

	printf("-- Send message id: %d, len: %d\n", message.id, message.len);
}

void GripperCommunication::unregisterListener(unsigned char messageId,
		int listenerId) {
	auto listenersIterator = this->callbacks.find(messageId);

	if (listenersIterator != this->callbacks.end()) {
		auto& listeners = listenersIterator->second;
		auto it = listeners.find(listenerId);
		if (it != listeners.end()) {
			printf("-- Remove listener for message id: %d, listener id: %d\n", messageId, listenerId);
			listeners.erase(it);
		}
	}
}

void GripperCommunication::processMessages() {
	msg_t message;

	int result = msg_receive(&message);

	if (result == -1) {
		throw MessageReceiveFailed();
	}

	//printf("-- Received id: %d, len: %d\n", message.id, message.len);

	if ((this->currentCommand != nullptr)
			&& (this->currentCommand.get()->message.id == message.id)) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status != E_CMD_PENDING) {
			printf("-- Clear message id %d\n", message.id);
			GripperCallback callback = this->currentCommand.get()->callback;
			this->currentCommand = nullptr;

			if (status == E_SUCCESS) {
				this->updateCommandState(message.id, CommandStateCode::SUCCESS);
			} else {
				this->updateCommandState(message.id, CommandStateCode::ERROR);
			}

			if (callback != nullptr) {
				callback(message);
			}
		}
	}

	this->callbackListeners((unsigned char) WellKnownMessageId::WILDCARD,
			message);
	this->callbackListeners(message.id, message);
}

bool GripperCommunication::lastCommandReturnedSuccess(
		const unsigned char messageId) {
	auto commandStateIterator = this->commandStates.find(messageId);
	if (commandStateIterator == this->commandStates.end()) {
		printf("Last command no\n");
		return false;
	} else {
		printf("Last command yes, %d, %d\n", commandStateIterator->second,
				CommandStateCode::SUCCESS);
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

	this->sendCommand(message);
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

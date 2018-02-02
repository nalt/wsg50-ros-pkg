#ifndef GRIPPER_COMM_H_
#define GRIPPER_COMM_H_

#include <ros/ros.h>
#include <wsg_50/gripper_socket.h>
#include "msg.h"
#include "common.h"
#include <map>
#include <functional>
#include <memory>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>

typedef std::function<void(Message& response)> GripperCallback;

class CommandSubscription {
public:
	char messageId;
	int listenerId;
};

class GripperState {
public:
	float width;
	float speed;
	float force;
	int32_t grasping_state;
	int32_t system_state;
	ConnectionState connection_state;
};

class CommandState {
public:
	CommandState();
	CommandState(Message& message, GripperCallback callback = nullptr) {
		this->message = message;
		this->callback = callback;
	}

	Message message;
	GripperCallback callback;
	ros::Time sent;
};



enum class WellKnownMessageId
	: unsigned char {
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
	SET_GRASP_FORCE = 0x32,
	STATUS_VALUES = 0x40,
	GRIPPING_STATE = 0x41,
	OPENING_VALUES = 0x43,
	SPEED_VALUES = 0x44,
	FORCE_VALUES = 0x45,
	WILDCARD = 0xFA // Use to register a listener which receives a callback whenever a message is received, regardless of the message id.
};

enum class CommandStateCode
	: unsigned char {
		UNKNOWN = 0, PENDING = 1, SUCCESS = 2, ERROR = 3
};

class GripperCommunication
final {
	public:
		static constexpr float TIMEOUT_DELAY = 2.5;
		GripperCommunication(std::string host, int port, int auto_update_frequency = 50);
		~GripperCommunication() {}

		void sendCommand(Message& message, GripperCallback callback = nullptr);
		void sendCommandSynchronous(Message& message, int timeout_in_ms = 1000);
		CommandSubscription subscribe(unsigned char messageId,
				GripperCallback callback);
		void unregisterListener(unsigned char messageId, int listenerId);
		bool acceptsCommands();
		void processMessages(int max_number_of_messages = 100);

		// long running, asynchronous gripper commands
		void move(float width, float speed, bool stop_on_block,
				GripperCallback callback = nullptr);
		void grasp(float width, float speed,
				GripperCallback callback = nullptr);
		void release(float width, float speed, GripperCallback callback =
				nullptr);
		void homing(GripperCallback callback = nullptr);

		// short running, synchronous calls to gripper
		void soft_stop();
		void fast_stop();
		void acknowledge_error();
		void set_force(float force);
		void set_acceleration(float acceleration);

		void connectToGripper(std::string protocol, std::string ip, int port);
		void disconnectFromGripper(bool announceDisconnect);
		void shutdown();

		GripperState getState();

		void activateAutomaticValueUpdates();

		bool lastCommandReturnedSuccess(const unsigned char messageId);
		int decodeStatus(Message& message);

		// delete copy and move constructors and assign operators
		GripperCommunication(const GripperCommunication&) = delete; // Copy construct
		GripperCommunication(GripperCommunication&&) = delete; // Move construct
		GripperCommunication& operator=(const GripperCommunication&) = delete; // Copy assign
		GripperCommunication& operator=(GripperCommunication&&) = delete; // Move assign

		void forceCallback(Message& message);
		void speedCallback(Message& message);
	private:
		void activateAutoUpdates(const unsigned char messageId,
				const int interval_ms);
		void callbackListeners(const unsigned char messageId, Message& message);
		void updateCommandState(const unsigned char messageId,
				const CommandStateCode state);

		void graspingStateCallback(Message& message);
		void widthCallback(Message& message);

		GripperSocket* gripper_socket;
		std::map<unsigned char, std::map<int, GripperCallback> > callbacks;
		std::map<unsigned char, CommandStateCode> commandStates;
		std::shared_ptr<CommandState> currentCommand;
		std::vector<CommandSubscription> subscriptions;
 		GripperState gripper_state;
 		bool running;
 		int auto_update_frequency;
 		ros::Time last_received_update;
	};

	class MessageQueueFull: public std::runtime_error {
	public:
		MessageQueueFull() :
				std::runtime_error("MessageQueueFull") {
		}
	};

	class MessageSendFailed: public std::runtime_error {
	public:
		MessageSendFailed() :
				std::runtime_error("MessageSendFailed") {
		}
	};

	class MessageReceiveFailed: public std::runtime_error {
	public:
		MessageReceiveFailed() :
				std::runtime_error("MessageReceiveFailed") {
		}
	};

	class MessageTimedOut: public std::runtime_error {
	public:
		MessageTimedOut() :
				std::runtime_error("MessageTimedOut") {
		}
	};

	class ProtocolNotSupported: public std::runtime_error {
	public:
		ProtocolNotSupported() :
				std::runtime_error("ProtocolNotSupported") {
		}
	};

	class ConnectionError: public std::runtime_error {
	public:
		ConnectionError() :
				std::runtime_error("ConnectionError") {
		}
	};

#endif GRIPPER_COMM_H_

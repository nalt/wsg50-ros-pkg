#ifndef GRIPPER_COMM_H_
#define GRIPPER_COMM_H_

#include <ros/ros.h>
#include "msg.h"
#include "common.h"
#include <map>
#include <functional>
#include <memory>
#include <cstring>
#include <vector>

typedef std::function<void(msg_t& response)> GripperCallback;

class CommandSubscription {
public:
	char messageId;
	int listenerId;
};

enum class ConnectionState: unsigned char {
	NOT_CONNECTED = 0,
	CONNECTING = 1,
	CONNECTED = 2,
	DROPPED = 3
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
	CommandState(msg_t message, GripperCallback callback = nullptr) {
		this->message = message;
		this->callback = callback;
	}

	msg_t message;
	GripperCallback callback;
};



enum class WellKnownMessageId
	: unsigned char {
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

// thanks https://stackoverflow.com/a/34653512
class GripperCommunication
final {
	public:
		static GripperCommunication& Instance() {
			// Since it's a static variable, if the class has already been created,
			// it won't be created again.
			// And it **is** thread-safe in C++11.
			static GripperCommunication myInstance;

			// Return a reference to our instance.
			return myInstance;
		}
		~GripperCommunication() {
		}

		void sendCommand(msg_t& message, GripperCallback callback = nullptr);
		void sendCommandSynchronous(msg_t& message, int timeout_in_ms = 1000);
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
		void requestValueUpdate(unsigned char messageId);

		bool lastCommandReturnedSuccess(const unsigned char messageId);
		int decodeStatus(msg_t& message);

		// delete copy and move constructors and assign operators
		GripperCommunication(const GripperCommunication&) = delete; // Copy construct
		GripperCommunication(GripperCommunication&&) = delete; // Move construct
		GripperCommunication& operator=(const GripperCommunication&) = delete; // Copy assign
		GripperCommunication& operator=(GripperCommunication&&) = delete; // Move assign

		void forceCallback(msg_t& message);
		void speedCallback(msg_t& message);
	private:
		GripperCommunication() {
			this->currentCommand = nullptr;
			gripper_state.force = -1;
			gripper_state.speed = -1;
			gripper_state.width = -1;
			gripper_state.grasping_state = -1;
			gripper_state.system_state = -1;
			gripper_state.connection_state = ConnectionState::NOT_CONNECTED;
			running = true;
		}

		void activateAutoUpdates(const unsigned char messageId,
				const int interval_ms);
		void callbackListeners(const unsigned char messageId, msg_t& message);
		void updateCommandState(const unsigned char messageId,
				const CommandStateCode state);

		void graspingStateCallback(msg_t& message);
		void widthCallback(msg_t& message);


		std::map<unsigned char, std::map<int, GripperCallback> > callbacks;
		std::map<unsigned char, CommandStateCode> commandStates;
		std::shared_ptr<CommandState> currentCommand;
		std::vector<CommandSubscription> subscriptions;
 		GripperState gripper_state;
 		bool running;
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

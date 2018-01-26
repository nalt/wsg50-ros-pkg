#ifndef GRIPPER_COMM_H_
#define GRIPPER_COMM_H_

#include "msg.h"
#include "common.h"
#include <map>
#include <functional>
#include <memory>
#include <cstring>

typedef std::function<void(msg_t& response)> GripperCallback;

// thanks https://stackoverflow.com/a/24609331
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// thanks https://stackoverflow.com/a/27104183
template<class T>
std::unique_ptr<T> copy_unique(const std::unique_ptr<T>& source) {
	return source ? make_unique < T > (*source) : nullptr;
}

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

class SmartMessage {
public:
	SmartMessage() {
		id = 0;
		len = 0;
		data = std::unique_ptr<unsigned char>(new unsigned char[0]);
	}

	SmartMessage(const SmartMessage& m) {
		id = m.id;
		len = m.len;
		data = std::unique_ptr<unsigned char>(new unsigned char(m.len));
		memcpy( &data.get()[0], m.data.get(), sizeof( unsigned char ) * m.len);
		printf("copy cons\n");
	}

	unsigned char id;
	unsigned int len;
	std::unique_ptr<unsigned char> data;

	msg_t toMessage() {
		msg_t result;
		result.id = id;
		result.len = len;
		result.data = data.get();

		return result;
	}
};

enum class WellKnownMessageId
	: unsigned char {
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

		void sendSmartCommand(SmartMessage& message, GripperCallback callback =
				nullptr);
		void sendCommand(msg_t& message, GripperCallback callback = nullptr);
		void sendCommandSynchronous(msg_t& message);
		CommandSubscription subscribe(unsigned char messageId,
				GripperCallback callback);
		void unregisterListener(unsigned char messageId, int listenerId);
		bool acceptsCommands();
		msg_t processMessages();

		void move(float width, float speed, bool stop_on_block,
				GripperCallback callback = nullptr);
		void soft_stop(GripperCallback callback = nullptr);
		void emergency_stop(GripperCallback callback = nullptr);
		void grasp(float width, float speed,
				GripperCallback callback = nullptr);
		void release(float width, float speed, GripperCallback callback =
				nullptr);
		void homing(GripperCallback callback = nullptr);
		void acknowledge_error(GripperCallback callback = nullptr);
		void set_force(float force, GripperCallback callback = nullptr);
		void set_acceleration(float acceleration, GripperCallback callback = nullptr);

		SmartMessage createMoveMessage(float width, float speed,
				bool stop_on_block);
		SmartMessage createHomingMessage();
		SmartMessage createSoftStopMessage();
		SmartMessage createAcknowledgeMessage();
		SmartMessage createEmergencyStopMessage();
		SmartMessage createSetForceMessage(float force);
		SmartMessage createSetAccelerationMessage(float acceleration);

		GripperState getState();

		void activateOpeningValueUpdates(const int interval_ms);
		void activateGripStateUpdates(const int interval_ms);
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
			requestForceToggle = true;
			requestSpeedToggle = true;
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
		GripperState gripper_state;
		bool requestForceToggle;
		bool requestSpeedToggle;
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

#endif GRIPPER_COMM_H_

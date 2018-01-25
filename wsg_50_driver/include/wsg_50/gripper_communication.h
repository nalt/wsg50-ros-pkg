#ifndef GRIPPER_COMM_H_
#define GRIPPER_COMM_H_

#include "msg.h"
#include "common.h"
#include <map>
#include <functional>
#include <memory>
#include <cstring>

typedef std::function< void(msg_t& response) > GripperCallback;

class CommandSubscription {
    public:
        char messageId;
        int listenerId;
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

enum class WellKnownMessageId: unsigned char {
	MOVE = 0x21,
	STATUS_VALUES = 0x40,
	GRIPPING_STATE = 0x41,
	OPENING_VALUES = 0x43,
	WILDCARD = 250 // Use to register a listener which receives a callback whenever a message is received, regardless of the message id.
};

enum class CommandStateCode: unsigned char {
	UNKNOWN = 0,
	PENDING = 1,
    SUCCESS = 2,
    ERROR = 3
};

// thanks https://stackoverflow.com/a/34653512
class GripperCommunication final {
    public:
        static GripperCommunication& Instance() {
            // Since it's a static variable, if the class has already been created,
            // it won't be created again.
            // And it **is** thread-safe in C++11.
            static GripperCommunication myInstance;

            // Return a reference to our instance.
            return myInstance;
        }
        ~GripperCommunication() {}

        void sendCommand(msg_t& message, GripperCallback callback = nullptr);
        CommandSubscription subscribe(unsigned char messageId, GripperCallback callback);
        void unregisterListener(unsigned char messageId, int listenerId);
        bool acceptsCommands();
        void processMessages();

        void move(float width, float speed, bool stop_on_block, GripperCallback callback = nullptr);

        void activateOpeningValueUpdates(const int interval_ms);
        void activateGripStateUpdates(const int interval_ms);

        bool lastCommandReturnedSuccess(const unsigned char messageId);

        // delete copy and move constructors and assign operators
		GripperCommunication(const GripperCommunication&) = delete;             // Copy construct
		GripperCommunication(GripperCommunication&&) = delete;                  // Move construct
		GripperCommunication& operator=(const GripperCommunication&) = delete;  // Copy assign
		GripperCommunication& operator=(GripperCommunication&&) = delete;       // Move assign
    private:
        GripperCommunication() {
			this->currentCommand = nullptr;
		}

        void activateAutoUpdates(const unsigned char messageId, const int interval_ms);
        void callbackListeners(const unsigned char messageId, msg_t& message);
        void updateCommandState(const unsigned char messageId, const CommandStateCode state);

        std::map<unsigned char, std::map<int, GripperCallback> > callbacks;
        std::map<unsigned char, CommandStateCode> commandStates;
        std::shared_ptr<CommandState> currentCommand;
};


class MessageQueueFull : public std::runtime_error {
	public:
		MessageQueueFull() : std::runtime_error("MessageQueueFull") { }
};


class MessageSendFailed : public std::runtime_error {
	public:
		MessageSendFailed() : std::runtime_error("MessageSendFailed") { }
};

class MessageReceiveFailed : public std::runtime_error {
	public:
	MessageReceiveFailed() : std::runtime_error("MessageReceiveFailed") { }
};

#endif GRIPPER_COMM_H_

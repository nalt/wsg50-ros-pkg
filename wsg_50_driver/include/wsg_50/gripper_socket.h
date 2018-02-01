/*
 * gripper_socket.h
 *
 *  Created on: Feb 1, 2018
 *      Author: noertemann
 */
#ifndef GRIPPER_SOCKET_H_
#define GRIPPER_SOCKET_H_

#include <thread>
#include <mutex>
#include <memory>
#include <cstring>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

enum class ConnectionState: unsigned char {
	NOT_CONNECTED = 0,
	CONNECTED = 1,
};

class Message {
	public:
		Message() {
			this->id = 0;
			this->length = 0;
			this->data = nullptr;
		}

		Message(unsigned char id, unsigned int length, unsigned char* data) {
			this->id = id;
			this->length = length;
			this->data = new unsigned char[length];
			std::memcpy(this->data, data, sizeof(unsigned char) * length);
		}

		Message(Message* m) {
			this->id = m->id;
			this->length = m->length;
			if (m->length > 0) {
				this->data = new unsigned char[m->length];
				std::memcpy(this->data, m->data, sizeof(unsigned char) * m->length);
			} else {
				this->data = nullptr;
			}
		}

		~Message() {
			if (this->length > 0) {
				delete this->data;
				this->data = nullptr;
			}
		}

		unsigned char id;
		unsigned int length;
		unsigned char* data;
};

class GripperSocket {
	public:
		static const uint32_t TCP_RECEIVE_TIMEOUT_SEC = 60;
		static const uint32_t MSG_HEADER_SIZE = 3;
		static const uint32_t MSG_PREAMBLE_SIZE = 3;
		static const uint32_t MSG_CHECKSUM_SIZE = 2;
		static const unsigned char MSG_PREABMLE_BYTE = 0xaa;
		GripperSocket(std::string host, int port);
		void startReadLoop();
		void disconnect();
		ConnectionState getConnectionState();
		std::shared_ptr<Message> getMessage();
		void sendMessage(Message& message);

	private:
		void connectSocket();
		void disconnectSocket();
		void readLoop();
		bool tryParseMessage(std::shared_ptr<Message>& message);
		void readFromBuffer(int length, unsigned char* target);

		static const uint32_t RECEIVE_BUFFER_SIZE = 1024;
		static const uint32_t BUFFER_SIZE = 4096;
		unsigned char buffer[GripperSocket::BUFFER_SIZE];
		uint32_t buffer_pointer;
		uint32_t buffer_content_length;
		unsigned char receive_buffer[RECEIVE_BUFFER_SIZE];
		std::string host;
		int port;
		ConnectionState connection_state;
		ConnectionState previous_connection_state;
		std::recursive_mutex buffer_mutex;
		bool running;
		std::thread read_loop;
		int socket_fd;
};

class SocketCreationError: public std::runtime_error {
	public:
		SocketCreationError(std::string message) :
			std::runtime_error(message) {
		}
};

class AddressConversionError: public std::runtime_error {
	public:
		AddressConversionError(std::string message) :
			std::runtime_error(message) {
		}
};

class ConnectionOpenError: public std::runtime_error {
	public:
		ConnectionOpenError(std::string message) :
			std::runtime_error(message) {
		}
};

class SocketNotOpen: public std::runtime_error {
	public:
		SocketNotOpen(std::string message) :
			std::runtime_error(message) {
		}
};

class NotEnoughDataInBuffer: public std::runtime_error {
	public:
		NotEnoughDataInBuffer(std::string message) :
			std::runtime_error(message) {
		}
};

class ChecksumError: public std::runtime_error {
	public:
		ChecksumError(std::string message) :
			std::runtime_error(message) {
		}
};

class SendError: public std::runtime_error {
	public:
		SendError(std::string message) :
			std::runtime_error(message) {
		}
};

class SocketConfigrurationError: public std::runtime_error {
	public:
		SocketConfigrurationError(std::string message) :
			std::runtime_error(message) {
		}
};

#endif /* GRIPPER_SOCKET_H_ */

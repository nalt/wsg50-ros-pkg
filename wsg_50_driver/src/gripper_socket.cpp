/*
 * gripper_socket.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: noertemann
 */

#include "ros/ros.h"
#include "wsg_50/gripper_socket.h"
#include "wsg_50/checksum.h"

// Byte access
#define hi(x) (unsigned char)(((x) >> 8) & 0xff)  // Returns the upper byte of the passed short
#define lo(x) (unsigned char)((x)&0xff)           // Returns the lower byte of the passed short

GripperSocket::GripperSocket(std::string host, int port)
{
  this->running = false;
  this->host = host;
  this->port = port;
  this->connection_state = ConnectionState::NOT_CONNECTED;
  this->previous_connection_state = this->connection_state;
  this->socket_fd = -1;
  this->connection_state_change_callback = nullptr;

  this->startReadLoop();
}

void GripperSocket::connectSocket()
{
  sockaddr_in serv_addr;
  ROS_INFO("Try to connect to gripper at %s:%d", this->host.c_str(), this->port);

  int fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0)
  {
    throw SocketCreationError("Could not create socket.");
  }

  memset(&serv_addr, '0', sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(this->port);
  std::string hostname = this->host;
  if (inet_pton(AF_INET, hostname.c_str(), &serv_addr.sin_addr) <= 0)
  {
    throw AddressConversionError("Could not convert address into binary format: " + this->host);
  }

  unsigned int val = 1024;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (void*)&val, (socklen_t)sizeof(val));

  struct timeval timeout;
  timeout.tv_sec = TCP_RECEIVE_TIMEOUT_SEC;
  timeout.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (void*)&timeout, (socklen_t)sizeof(struct timeval));

  /* Set socket to non-blocking */
  int flags;
  if ((flags = fcntl(fd, F_GETFL, 0)) < 0)
  {
    throw SocketConfigrurationError("");
  }

  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    throw SocketConfigrurationError("");
  }

  int connect_result = 0;
  if ((connect_result = connect(fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0)
  {
    if (errno == EINPROGRESS)
    {
      struct timeval tv;
      fd_set fdset;
      tv.tv_sec = 5;
      tv.tv_usec = 0;
      FD_ZERO(&fdset);
      FD_SET(fd, &fdset);
      if (select(fd + 1, NULL, &fdset, NULL, &tv) > 0)
      {
        socklen_t lon = sizeof(int);
        int valopt = 0;
        getsockopt(fd, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon);
        if (valopt)
        {
          throw ConnectionOpenError("Could not open connection to " + this->host + ":" + std::to_string(this->port));
        }
      }
      else
      {
        throw ConnectionOpenError("Could not open connection to " + this->host + ":" + std::to_string(this->port));
      }
    }
    else
    {
      throw ConnectionOpenError("Could not open connection to " + this->host + ":" + std::to_string(this->port));
    }
  }

  this->connection_state = ConnectionState::CONNECTED;
  this->socket_fd = fd;
}

void GripperSocket::disconnect()
{
  if (this->running == true)
  {
    this->running = false;
    this->read_loop.join();
  }
}

ConnectionState GripperSocket::getConnectionState()
{
  return this->connection_state;
}

std::shared_ptr<Message> GripperSocket::getMessage()
{
  if (this->readBuffer.getLength() < MSG_HEADER_SIZE + MSG_PREAMBLE_SIZE)
  {
    return nullptr;
  }

  // search preamble
  uint32_t preamble_count = 0;
  while ((preamble_count != GripperSocket::MSG_PREAMBLE_SIZE) && (this->readBuffer.getLength() > 0))
  {
    unsigned char b;
    this->readBuffer.read(1, &b);

    if (b == GripperSocket::MSG_PREABMLE_BYTE)
    {
      preamble_count += 1;
    }
    else
    {
      preamble_count = 0;
    }
  }

  // read header
  if (this->readBuffer.getLength() < GripperSocket::MSG_HEADER_SIZE)
  {
    throw NotEnoughDataInBuffer("Could not read Header");
  }
  unsigned char header[GripperSocket::MSG_HEADER_SIZE];
  this->readBuffer.read(GripperSocket::MSG_HEADER_SIZE, header);
  unsigned char messageId = header[0];
  unsigned int messageLength = (unsigned short)header[1] | ((unsigned short)header[2] << 8);

  // read payload and checksum
  unsigned char messageData[messageLength + GripperSocket::MSG_CHECKSUM_SIZE];
  this->readBuffer.read(messageLength + GripperSocket::MSG_CHECKSUM_SIZE, messageData);

  unsigned short checksum = 0x50f5;  // Checksum over preamble (0xaa 0xaa 0xaa)
  checksum = checksum_update_crc16(header, GripperSocket::MSG_HEADER_SIZE, checksum);
  checksum = checksum_update_crc16(messageData, messageLength + GripperSocket::MSG_CHECKSUM_SIZE, checksum);

  if (checksum != 0)
  {
    throw ChecksumError("ChecksumError");
  }

  return std::make_shared<Message>(new Message(messageId, messageLength, messageData));
}

void GripperSocket::readLoop()
{
  while (this->running == true)
  {
    if (this->connection_state == ConnectionState::CONNECTED)
    {
      if (this->previous_connection_state != this->connection_state)
      {
        ROS_INFO("Changed connection state from NOT_CONNECTED to CONNECTED");
        if (this->connection_state_change_callback != nullptr)
        {
          this->connection_state_change_callback(this->connection_state);
        }
      }
      this->previous_connection_state = this->connection_state;

      if (this->socket_fd > 0)
      {
        int read_bytes = recv(this->socket_fd, this->receive_buffer, GripperSocket::BUFFER_SIZE, 0);
        if (read_bytes <= 0)
        {
          if ((read_bytes == 0) || ((errno != EWOULDBLOCK) && (errno != EAGAIN)))
          {
            this->disconnectSocket();
            this->connection_state = ConnectionState::NOT_CONNECTED;
          }
        }
        else
        {
          try
          {
            this->readBuffer.write(read_bytes, this->receive_buffer);
          }
          catch (NotEnoughSpaceInBuffer& ex)
          {
            ROS_ERROR("Discarding data from gripper, because the read buffer is full.");
          }
          catch (...)
          {
            ROS_ERROR("Unkown error while writing into read buffer");
          }
        }
      }

      if (this->socket_fd > 0)
      {
        if (this->writeBuffer.getLength() > 0)
        {
          unsigned int a = GripperSocket::BUFFER_SIZE;
          int length = std::min(a, this->writeBuffer.getLength());
          this->writeBuffer.read(length, this->send_buffer);
          int result = send(this->socket_fd, this->send_buffer, length, 0);
          if (result < length)
          {
            this->disconnectSocket();
            this->connection_state = ConnectionState::NOT_CONNECTED;
          }
        }
      }
    }
    else if (this->connection_state == ConnectionState::NOT_CONNECTED)
    {
      if (this->previous_connection_state != this->connection_state)
      {
        ROS_INFO("Changed connection state from CONNECTED to NOT_CONNECTED");
        if (this->connection_state_change_callback != nullptr)
        {
          this->connection_state_change_callback(this->connection_state);
        }
      }
      this->previous_connection_state = this->connection_state;

      try
      {
        this->connectSocket();
      }
      catch (...)
      {
        ROS_WARN("Error while connecting to %s:%d using TCP/IP.", this->host.c_str(), this->port);
        std::chrono::milliseconds timespan(200);
        std::this_thread::sleep_for(timespan);
      }
    }

    std::chrono::milliseconds timespan(10);
    std::this_thread::sleep_for(timespan);
  }
}

void GripperSocket::setConnectionStateChangedCallback(ConnectionStateCallback callback)
{
  this->connection_state_change_callback = callback;
}

void GripperSocket::reconnect()
{
  ROS_INFO("Reconnect requested.");
  this->disconnectSocket();
}

void GripperSocket::sendMessage(Message& message)
{
  if (this->socket_fd <= 0)
  {
    throw SocketNotOpen("");
  }

  unsigned char raw_message[3 + 3];  // Preamble plus Header
  unsigned short crc;
  // Preamble
  for (int i = 0; i < 3; i++)
  {
    raw_message[i] = 0xaa;
  }
  raw_message[3 + 0] = message.id;
  raw_message[3 + 1] = lo(message.length);
  raw_message[3 + 2] = hi(message.length);

  unsigned char messageData[message.length];
  for (int i = 0; i < message.length; i++)
  {
    messageData[i] = message.data.at(i);
  }
  crc = checksum_crc16(raw_message, 6);
  crc = checksum_update_crc16(messageData, message.length, crc);

  unsigned int raw_message_size = 6 + message.length + 2;
  unsigned char buf[raw_message_size];
  memcpy(buf, raw_message, 6);
  memcpy(buf + 6, messageData, message.length);

  memcpy(buf + 6 + message.length, (unsigned char*)&crc, 2);

  this->writeBuffer.write(raw_message_size, buf);
}

void GripperSocket::startReadLoop()
{
  this->running = true;
  this->read_loop = std::thread(&GripperSocket::readLoop, this);
}

void GripperSocket::disconnectSocket()
{
  if (this->socket_fd > 0)
  {
    ROS_INFO("Closing socket.");
    close(this->socket_fd);
    auto state_changed = this->connection_state != ConnectionState::NOT_CONNECTED;
    this->connection_state = ConnectionState::NOT_CONNECTED;
    this->socket_fd = -1;
    if ((state_changed == true) && (this->connection_state_change_callback != nullptr))
    {
      this->connection_state_change_callback(this->connection_state);
    }
  }
}

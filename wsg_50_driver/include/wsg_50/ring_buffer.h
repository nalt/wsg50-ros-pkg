/*
 * ring_buffer.h
 *
 *  Created on: Feb 1, 2018
 *      Author: noertemann
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <cstring>
#include <algorithm>
#include <memory>

class NotEnoughDataInBuffer : public std::runtime_error
{
public:
  NotEnoughDataInBuffer(std::string message = "") : std::runtime_error("[NotEnoughDataInBuffer] " + message)
  {
  }
};

class NotEnoughSpaceInBuffer : public std::runtime_error
{
public:
  NotEnoughSpaceInBuffer(std::string message = "") : std::runtime_error("[NotEnoughSpaceInBuffer] " + message)
  {
  }
};

class RingBuffer
{
public:
  RingBuffer()
  {
    this->buffer_pointer = 0;
    this->buffer_content_length = 0;
  }

  void read(unsigned int length, unsigned char* target)
  {
    std::lock_guard<std::mutex> guard(lock);
    if (this->buffer_content_length < length)
    {
      throw NotEnoughDataInBuffer();
    }

    int first = std::min(RingBuffer::BUFFER_SIZE - this->buffer_pointer, length);
    if (first > 0)
    {
      memcpy(target, this->buffer + this->buffer_pointer, first);
      this->buffer_content_length -= first;
      this->buffer_pointer = (this->buffer_pointer + first) % RingBuffer::BUFFER_SIZE;
    }

    int second = length - first;
    if (second > 0)
    {
      memcpy(target + first, this->buffer + this->buffer_pointer, second);
      this->buffer_content_length -= second;
      this->buffer_pointer = (this->buffer_pointer + second) % RingBuffer::BUFFER_SIZE;
    }
  }

  void peek(unsigned int length, unsigned char* target)
  {
    std::lock_guard<std::mutex> guard(lock);
    if (this->buffer_content_length < length)
    {
      throw NotEnoughDataInBuffer("Error while peeking");
    }

    int first = std::min(RingBuffer::BUFFER_SIZE - this->buffer_pointer, length);
    if (first > 0)
    {
      memcpy(target, this->buffer + this->buffer_pointer, first);
    }

    int tmp_buffer_pointer = (this->buffer_pointer + first) % RingBuffer::BUFFER_SIZE;

    int second = length - first;
    if (second > 0)
    {
      memcpy(target + first, this->buffer + tmp_buffer_pointer, second);
    }
  }

  void remove(unsigned int length)
  {
    std::lock_guard<std::mutex> guard(lock);
    this->buffer_content_length -= length;
    this->buffer_pointer = (this->buffer_pointer + length) % RingBuffer::BUFFER_SIZE;
  }

  void write(unsigned int length, unsigned char* source)
  {
    std::lock_guard<std::mutex> guard(lock);
    if (length > RingBuffer::BUFFER_SIZE - this->buffer_content_length)
    {
      throw NotEnoughSpaceInBuffer();
    }

    int first_free = (this->buffer_pointer + this->buffer_content_length) % RingBuffer::BUFFER_SIZE;

    int first = std::min(RingBuffer::BUFFER_SIZE - first_free, length);
    if (first > 0)
    {
      memcpy(this->buffer + first_free, source, first);
      this->buffer_content_length += first;
    }

    int second = length - first;
    if (second > 0)
    {
      memcpy(this->buffer, source + first, second);
      this->buffer_content_length += second;
    }
  }

  void clear()
  {
    this->buffer_pointer = 0;
    this->buffer_content_length = 0;
  }

  unsigned int getLength()
  {
    return this->buffer_content_length;
  }

private:
  static const uint32_t BUFFER_SIZE = 4096;
  unsigned char buffer[RingBuffer::BUFFER_SIZE];
  uint32_t buffer_pointer;
  uint32_t buffer_content_length;
  std::mutex lock;
};

#endif /* RING_BUFFER_H_ */

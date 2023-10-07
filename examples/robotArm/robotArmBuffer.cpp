#include "robotArmBuffer.h"

robotArmBuffer::robotArmBuffer(size_t buf_len, size_t str_len) {
  this->buffer_size = buf_len + 1;
  this->string_lenght = str_len;

  this->buffer = new char[this->string_lenght * this->buffer_size];
  this->head = this->tail = 0;
}

robotArmBuffer::~robotArmBuffer() { delete this->buffer; }

bool robotArmBuffer::put(char *value) {
  if (isFull()) {
    Serial.println(" FULL ");
    return false;
  }

  // Place string in buffer
  strcpy(&buffer[index(0, head)], value);

  // Increase the head
  head = (head + 1) % buffer_size;

  Serial.print("Added: ");
  Serial.println(head);

  return true;
}

bool robotArmBuffer::get(char *target) {

  if (isEmpty())
    return false;

  // Store the tailer string in target
  strcpy(target, (buffer + index(0, tail)));

  // Increase the tail
  tail = (tail + 1) % buffer_size;

  return true;
}

bool robotArmBuffer::isFull() {
  return ((head + 1) % buffer_size) == tail ? true : false;
}

bool robotArmBuffer::isEmpty() { return tail == head ? true : false; }

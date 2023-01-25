
#ifndef _ROBOTARMBUFFER_H_
#define _ROBOTARMBUFFER_H_

#include "robotArmConfig.h"

class robotArmBuffer {

public:
  robotArmBuffer(size_t buf_len = BUFFER_SIZE,
                 size_t str_len = MAX_PACKET_SIZE);
  ~robotArmBuffer();

  bool put(char *value);
  bool get(char *target);
  bool isFull();
  bool isEmpty();

private:
  char *buffer;
  size_t buffer_size, head, tail, string_lenght;

  size_t index(int x, int y) const { return x + string_lenght * y; }
};

#endif

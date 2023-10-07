
#ifndef _ROBOTARMCOMM_H_
#define _ROBOTARMCOMM_H_

#include "robotArmConfig.h"

#define BUFFER_EMPTY 0
#define BUFFER_READY 1
#define BUFFER_FULL 2

// Class for communication over UART, designed to work with GCode

class robotArmComm {

public:
  robotArmComm(void);

  void begin(void);

  // run() called in the main loop to read incoming Serial data and execute
  // commands
  bool listen(void);

  void send(char *command);
  void send(char *command, bool checksum);

  bool check(char *identifier);
  bool value(char *name, float *var);
  bool value(char *name, float *var, char **pos);
  bool value(char *name, bool *var);
  bool value(char *name, bool *var, char **pos);

  char *getPacket(void);

private:
  /*
   * @brief   Checks for Serial data in UART buffer and stores it
   */
  uint8_t read(void);

  // Array to temporary hold data
  char packet[MAX_PACKET_SIZE];

  uint16_t timeout = 2000; // Should be 200 at some point

  uint8_t status = 0;
};

#endif

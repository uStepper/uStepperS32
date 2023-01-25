#include "robotArmComm.h"

robotArmComm::robotArmComm(void) {}

void robotArmComm::begin(void) {

  UARTPORT.begin(BAUD_RATE);

  while (!UARTPORT) {
    ;
  }
}

// Check for an value (identifier) in the received packet
bool robotArmComm::check(char *identifier) {

  char command[10] = {'\0'};

  // The first part is always the command
  char *start = &this->packet[0];
  char *end = strpbrk(this->packet, " ");

  uint8_t len = end - start;
  strncpy(command, this->packet, len);

  if (strcmp(command, identifier) == 0) {

    this->send(COMMAND_VALID);
    return true;
  }

  return false;
}

bool robotArmComm::listen(void) {

  this->status = this->read();

  if (this->status == PACKET_READY) {

    return true;
  }

  return false;
}

char *robotArmComm::getPacket(void) {
  if (this->packet[0] == '\0') {
    return NULL;
  }
  return this->packet;
}

bool robotArmComm::value(char *name, float *var) {

  if (this->value(name, var, NULL))
    return true;
  else
    return false;
}

bool robotArmComm::value(char *name, bool *var) {

  if (this->value(name, var, NULL))
    return true;
  else
    return false;
}

bool robotArmComm::value(char *name, float *var, char **pos) {

  char *start;
  char *end;
  size_t len;

  char buf[20] = {'\0'};

  // Find start of parameter value
  if (start = strstr(this->packet, name)) {

    if (pos != NULL)
      *pos = start;

    start++; // Not interested in the param name

    // Find end of parameter value
    if (end = strpbrk(start, " \n")) {

      len = end - start;

      strncpy(buf, start, len);
      buf[len] = '\0';

      // Now convert the string in buf to a float
      *var = atof(buf);

      return true;
    }
  }

  return false;
}

bool robotArmComm::value(char *name, bool *var, char **pos) {

  char *start;
  char *end;
  size_t len;
  bool result;
  char buf[20] = {'\0'};

  // Find start of parameter value
  if (start = strstr(this->packet, name)) {

    if (pos != NULL)
      *pos = start;

    start++; // Not interested in the param name

    // Find end of parameter value
    if (end = strpbrk(start, " \n")) {

      len = end - start;

      strncpy(buf, start, len);
      buf[len] = '\0';

      // Now convert the string in buf to a float
      result = (bool)atof(buf);
      *var = result;

      return true;
    }
  }

  return false;
}

uint8_t robotArmComm::read(void) {

  static bool newPacket = false;
  static uint32_t lastChar = 0;
  static uint8_t packetLen = 0;

  char c;
  float checksum;
  char *packetEnd;
  if (newPacket == false && packetLen > 0) {
    packetLen = 0;

    // Clear packet buffer before any new data is added
    memset(this->packet, 0, sizeof(this->packet));
  }

  while (UARTPORT.available() > 0) {

    lastChar = micros();

    // Read the newest char from the UART buffer
    c = UARTPORT.read();
    // DEBUG_PRINTLN(c);
    // Save the char in the buffer
    this->packet[packetLen++] = c;
    newPacket = true;
  }

  // If the timeout has been reached, or a newline has been received the string
  // is complete
  if ((micros() - lastChar >= UART_TIMEOUT || c == '\n') && newPacket == true) {

    newPacket = false;

    // Check for checksum
    if (/*this->value("*", &checksum, &packetEnd)*/ 1) {

      // Checksum appended, check if it correct
      if (/*(uint8_t)checksum == 0xff*/ 1) {

        // Remove checksum from packet, as it is no longer needed
        *packetEnd = '\0';
        //DEBUG_PRINTLN("PACKET_READY");
        return PACKET_READY; // New data

      } else {
        //DEBUG_PRINTLN("PACKET_CRC_ERROR");
        return PACKET_CRC_ERROR; // Wrong checksum error
      }
    }
    //DEBUG_PRINTLN("PACKET_CRC_MISSING");
    return PACKET_CRC_MISSING; // No checksum found error
  }
  return PACKET_NONE; // No data
}

void robotArmComm::send(char *command) { this->send(command, true); }

void robotArmComm::send(char *command, bool checksum) {

  char buf[MAX_PACKET_SIZE] = {'\0'};

  strcpy(buf, command);

  if (checksum) {
    strcat(buf,
           " *255"); // Append checksum, should be calculated from entire string
  }

  UARTPORT.println(buf); // Always append a newline to indicate end of command
}

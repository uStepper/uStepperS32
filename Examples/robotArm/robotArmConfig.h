
#ifndef _ROBOTARMCONFIG_H_
#define _ROBOTARMCONFIG_H_

typedef enum {
  rdy = 1,
  home,
  resetHome,
  move,
  runContinously,
  setVelocity,
  setAcceleration,
  stop,
  setBrakeMode,
  setHomeSpeed,
  setStallSense
} state_t;

/* Define DEBUG to enable generelle debug output message in Serial */
#define DEBUG

#include <Arduino.h>
#include <Wire.h>

#define I2CPORT Wire
#define DEBUGPORT Serial
#define UARTPORT Serial2

#define BAUD_RATE 115200

#ifdef DEBUG
#define DEBUG_PRINT(x) DEBUGPORT.print(x)
#define DEBUG_PRINTLN(x) DEBUGPORT.println(x)
#define DEBUG_PRINTFLOAT(x,y) DEBUGPORT.print(x,y)
#define DEBUG_PRINTLNFLOAT(x,y) DEBUGPORT.println(x,y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define MATH_PI 3.14159265359f
#define MATH_PI_HALF 1.570796326795f

#define RAD2DEG 180.0 / MATH_PI
#define DEG2RAD MATH_PI / 180

#define GEARRATIO 5.1f

#define RAD2DEGGEARED RAD2DEG *GEARRATIO
#define DEG2RADGEARED DEG2RAD / GEARRATIO

// Numbers refering to each motor controlling each joint
#define BASE 0     // Master
#define SHOULDER 1 // Slave 1
#define ELBOW 2    // Slave 2

#define LOWERARMLEN 182.0f
#define UPPERARMLEN 188.0f
#define XOFFSET 47.0f // offset in the L1 vector direction
#define ZOFFSET 73.0f
#define AXOFFSET 40.0f  // Actuator offset in L1 vector direction
#define AZOFFSET -70.0f // Actuator offset in Z-direction

// Pre-calculated offsets in motor angle at home position
#define ROTOFFSET 0.0f
#define SHOULDEROFFSET (DEG2RAD * 142.0f)
#define ELBOWOFFSET (DEG2RAD * 45.0f)

// Constants for UART comm
#define MAX_PACKET_SIZE 50
#define BUFFER_SIZE 5
#define UART_TIMEOUT 2000

#define ADDRESS_PIN A0
#define SERVO_PIN 6
#define VALVE_PIN 3
#define PUMP_PIN 2

#define PACKET_NONE 0
#define PACKET_READY 1
#define PACKET_CRC_ERROR 2
#define PACKET_CRC_MISSING 3

#define COMMAND_VALID "OK"
#define COMMAND_READY "RDY"

// Union to convert byte array to float
typedef union {
  float f;
  uint8_t b[4];
} binaryFloat;

#endif
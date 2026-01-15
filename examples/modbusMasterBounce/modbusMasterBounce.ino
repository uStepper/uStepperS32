/********************************************************************************************
*       File:    modbusMasterBounce.ino                                                   *
*       Version: 2.3.0                                                                      *
*       Date:    October 11th, 2025                                                         *
*       Author:  Mogens Groth Nicolaisen                                                    *
*                                                                                           *
*  Description:  Modbus master example sketch for exercising a uStepper S32 stepper motor   *
*                via Modbus RTU. Sends relative angle commands and prints encoder feedback  *
*                to the serial monitor.                                                     *
*                                                                                           *
*                This sketch replaces direct control with Modbus register writes and reads, *
*                enabling integration with Modbus-based automation systems.                 *
*                                                                                           *
*  Library Dependencies:                                                                    *
*    - ModbusMaster by Doc Walker                                                           *
*      https://github.com/4-20ma/ModbusMaster                                               *
*      Install via Arduino Library Manager or manually from GitHub                         *
*                                                                                           *
*    - RS485 transceiver (e.g., MAX485) required for physical Modbus RTU communication      *
*                                                                                           *
*  For more information, check out the documentation:                                       *
*    https://github.com/uStepper/uStepperS32/blob/modbusfeature/docs/ModbusUtilitiesDocumentation.md *
*                                                                                           *
*********************************************************************************************
*   (C) 2025                                                                                 *
*                                                                                           *
*   uStepper ApS                                                                             *
*   www.ustepper.com                                                                         *
*   administration@ustepper.com                                                              *
*                                                                                           *
*   The code contained in this file is released under the following open source license:     *
*                                                                                           *
*       Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International              *
*                                                                                           *
*   The code in this file is provided without warranty of any kind – use at own risk!        *
*   Neither uStepper ApS nor the author can be held responsible for any damage               *
*   caused by the use of the code contained in this file!                                    *
********************************************************************************************/
#include <ModbusMaster.h>

// Modbus slave ID
#define SLAVE_ID 1

// Register addresses (based on ESPHome config)
#define REG_MOVE_ANGLE      14  // Relative angle move (FP32)
#define REG_ANGLE_MOVED      0  // Encoder angle moved (FP32)
#define REG_MOTOR_STATE      6  // Motor state (U_WORD)
#define REG_BRAKE_MODE      16  // Brake mode (U_WORD)
#define REG_RUN_CURRENT     24  // Run current % (U_WORD)
#define REG_MAX_ACCEL       18  // Max acceleration (FP32)
#define REG_MAX_VELOCITY    20  // Max velocity (FP32)
#define REG_MODE            17  // Motion mode (U_WORD)

ModbusMaster modbus;
float targetAngle = 360.0;
bool isMoving = false;

// Helper to write a float to two consecutive Modbus registers
void writeFloat(uint16_t startReg, float value) {
  union { float f; uint16_t u16[2]; } data;
  data.f = value;
  modbus.setTransmitBuffer(0, data.u16[0]); // Low word
  modbus.setTransmitBuffer(1, data.u16[1]); // High word
  modbus.writeMultipleRegisters(startReg, 2);
}

// Helper to read a float from two consecutive Modbus registers
float readFloat(uint16_t startReg) {
  if (modbus.readHoldingRegisters(startReg, 2) == modbus.ku8MBSuccess) {
    union { float f; uint16_t u16[2]; } data;
    data.u16[0] = modbus.getResponseBuffer(0);
    data.u16[1] = modbus.getResponseBuffer(1);
    return data.f;
  }
  return NAN;
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600); // RS485 UART

  modbus.begin(SLAVE_ID, Serial2);
  delay(1000); // Let everything settle

  // --- Motor Configuration ---
  modbus.writeSingleRegister(REG_BRAKE_MODE, 1);    // Cool brake
  modbus.writeSingleRegister(REG_RUN_CURRENT, 20);  // 30% current
  modbus.writeSingleRegister(REG_MODE, 3);          // Relative angle mode

  writeFloat(REG_MAX_ACCEL, 4000.0);    // Acceleration in fullsteps/s²
  writeFloat(REG_MAX_VELOCITY, 400.0);  // Velocity in fullsteps/s
}

void loop() {
  // Check if motor has stopped and send next command
  if (!isMoving) {
    if (modbus.readHoldingRegisters(REG_MOTOR_STATE, 1) == modbus.ku8MBSuccess) {
      writeFloat(REG_MOVE_ANGLE, targetAngle); // Initiate movement
      isMoving = true;
      targetAngle = -targetAngle; // Alternate direction
      delay(100); // Brief delay to ensure command is sent
    }
  } else {
    // Motor is moving - check status frequently
    if (modbus.readHoldingRegisters(REG_MOTOR_STATE, 1) == modbus.ku8MBSuccess) {
      uint16_t state = modbus.getResponseBuffer(0);
      if (state == 0) isMoving = false; // 0 = standstill
    }
  }

  // Read and display angle moved (frequent reads)
  float moved = readFloat(REG_ANGLE_MOVED);
  if (!isnan(moved)) {
    Serial.print("Angle Moved: ");
    Serial.print(moved);
    Serial.println(" °");
  }

  delay(100); // Small delay to allow Modbus responses, fast enough for smooth feedback
}
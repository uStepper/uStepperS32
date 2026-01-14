/********************************************************************************************
*       File:    modbusMasterBounce.ino                                                   *
*       Version: 2.3.1                                                                      *
*       Date:    January 14th, 2026                                                         *
*       Author:  Mogens Groth Nicolaisen                                                    *
*                                                                                           *
*  Description:  Modbus master example sketch for exercising a uStepper S32 stepper motor   *
*                via Modbus RTU. Sends relative angle commands and prints encoder feedback  *
*                to the serial monitor.                                                     *
*                                                                                           *
*                This sketch replaces direct control with Modbus register writes and reads, *
*                enabling integration with Modbus-based automation systems.                 *
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
#include <UstepperS32.h>
// Modbus slave ID
#define SLAVE_ID 1

// Register addresses
#define REG_MOVE_ANGLE      14  // Relative angle move (FP32)
#define REG_ANGLE_MOVED      0  // Encoder angle moved (FP32)
#define REG_MOTOR_STATE      6  // Motor state (U_WORD)
#define REG_BRAKE_MODE      16  // Brake mode (U_WORD)
#define REG_RUN_CURRENT     24  // Run current % (U_WORD)
#define REG_MAX_ACCEL       18  // Max acceleration (FP32)
#define REG_MAX_VELOCITY    20  // Max velocity (FP32)
#define REG_MODE            17  // Motion mode (U_WORD)

ModbusMasterUtils modbus;
float targetAngle = 360.0;
bool isMoving = false;

void setup() {
    Serial.begin(9600);
    
    // Initialize Modbus master on Serial2 at 9600 baud
    modbus.begin(SLAVE_ID, Serial2, 9600);
    delay(1000);

    // Configure motor parameters
    modbus.writeRegister(REG_BRAKE_MODE, 1);    // Soft brake
    modbus.writeRegister(REG_RUN_CURRENT, 20);  // 30% current
    modbus.writeRegister(REG_MODE, 3);          // Relative angle mode
    modbus.writeFloat(REG_MAX_ACCEL, 4000.0);   // Acceleration
    modbus.writeFloat(REG_MAX_VELOCITY, 400.0); // Velocity
}

void loop() {
    if (!isMoving) {
        delay(1000);
        
        // Send movement command
        if (modbus.writeFloat(REG_MOVE_ANGLE, targetAngle)) {
            isMoving = true;
            targetAngle = -targetAngle; // Alternate direction
        }
    }

    // Check if motor has stopped
    uint16_t state;
    if (modbus.readRegister(REG_MOTOR_STATE, &state)) {
        if (state == 0) {
            isMoving = false;
        }
    }

    // Read and display angle moved
    float moved = modbus.readFloat(REG_ANGLE_MOVED);
    if (!isnan(moved)) {
        Serial.print("Angle Moved: ");
        Serial.print(moved);
        Serial.println(" deg");
    }
}
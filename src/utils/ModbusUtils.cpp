#include "ModbusUtils.h"
#include "../UstepperS32.h" 
  
// ModbusRTU instance
ModbusRTU ModbusUtils::mb;

// Array to hold two 16-bit registers for float conversion
uint16_t ModbusUtils::regs[2];

// State tracking for robust command handling
uint8_t ModbusUtils::previousMode = 255;
uint8_t ModbusUtils::commandState = CMD_STATE_IDLE;
uint32_t ModbusUtils::lastCommandTime = 0;
uint16_t ModbusUtils::lastCommandSeq[4] = {0, 0, 0, 0}; // Track last command values per mode

// Register pairs corresponding to each function
// Each pair represents two 16-bit Modbus holding registers used to store a 32-bit float value.
// The first element in each pair is the high register, and the second is the low register.
const uint8_t ModbusUtils::regPairs[10][2] = {
    {9, 8},   // moveToAngle: Stores the target angle for the stepper motor
    {11, 10}, // setRPM: Stores the target RPM for the stepper motor
    {13, 12}, // moveSteps: Stores the number of steps to move the motor
    {15, 14}, // moveAngle: Stores the relative angle to move the motor
    {17, 16}, // mode: Stores the current operation mode (e.g., moveToAngle, setRPM, etc.)
    {19, 18}, // maxAcceleration: Stores the maximum acceleration for the motor
    {21, 20}, // maxVelocity: Stores the maximum velocity for the motor
    {23, 22}, // loopMode: Stores the closed-loop mode (0 = disabled, 1 = enabled)
    {25, 24}, // runCurrent: Stores the motor's running current
    {27, 26}  // brakeMode: Stores the brake mode (e.g., HARD, SOFT)
};

// Initialize Modbus communication
void ModbusUtils::modbusEnable(UstepperS32 *stepper, uint8_t id, uint32_t baud) {
    Serial2.begin(baud);       // Start Serial2 with the specified baud rate
    mb.begin(&Serial2);        // Initialize ModbusRTU with Serial2
    mb.slave(id);              // Set the Modbus slave ID

    // Add holding registers (28 total + command sequence register) to the Modbus object
    for (uint8_t i = 0; i < 29; i++) {
        mb.addHreg(i, 0);
    }
    
    // Initialize state
    commandState = CMD_STATE_IDLE;
    previousMode = 255;
}

// Handle Modbus communication and stepper motor control
void ModbusUtils::handleModbus(UstepperS32 *stepper) {
    // Process Modbus tasks - call multiple times to ensure we handle all pending data
    mb.task();
    
    // Update holding registers with the current encoder angle
    float angle = stepper->encoder.getAngleMoved();
    floatToRegisters(angle, regs);
    mb.Hreg(0, regs[0]); // Low part of the angle
    mb.Hreg(1, regs[1]); // High part of the angle

    // Update holding registers with the current driver RPM
    float speed = stepper->getDriverRPM();
    floatToRegisters(speed, regs);
    mb.Hreg(2, regs[0]); // Low part of the RPM
    mb.Hreg(3, regs[1]); // High part of the RPM

    // Update holding registers with the current encoder RPM
    float encoderSpeed = stepper->encoder.getRPM();
    floatToRegisters(encoderSpeed, regs);
    mb.Hreg(4, regs[0]); // Low part of the encoder RPM
    mb.Hreg(5, regs[1]); // High part of the encoder RPM

    // Update holding registers with motor state and stall status
    mb.Hreg(6, stepper->getMotorState()); // Motor state (e.g., running or stopped)
    mb.Hreg(7, stepper->isStalled());     // Stall status (1 if stalled, 0 otherwise)

    // Read the current mode from the Modbus holding register
    uint8_t mode = mb.Hreg(17);
    
    // Validate mode - only accept modes 0-3
    if (mode > 3) {
        mode = previousMode != 255 ? previousMode : 0;
    }

    // Handle mode change to moveToAngle (mode 0)
    if (mode == 0 && previousMode != mode) {
        stepper->stop(HARD); // Stop the motor
        float currentAngle = stepper->encoder.getAngleMoved();
        floatToRegisters(currentAngle, regs);
        mb.Hreg(regPairs[0][1], regs[0]); // Low part of the current angle
        mb.Hreg(regPairs[0][0], regs[1]); // High part of the current angle
    }

    // Check if command registers have been written (detect new commands)
    bool hasNewCommand = false;
    uint16_t currentCmdHigh = mb.Hreg(regPairs[mode][0]);
    uint16_t currentCmdLow = mb.Hreg(regPairs[mode][1]);
    
    // For modes 2 and 3 (moveSteps, moveAngle), detect new commands by checking
    // if registers have changed from the last command
    if (mode == 2 || mode == 3) {
        // Check if the command has changed from the last one
        if ((currentCmdHigh != lastCommandSeq[0] || currentCmdLow != lastCommandSeq[1]) &&
            commandState == CMD_STATE_IDLE) {
            // Store new command values for next comparison
            lastCommandSeq[0] = currentCmdHigh;
            lastCommandSeq[1] = currentCmdLow;
            hasNewCommand = true;
            commandState = CMD_STATE_EXECUTING;
            lastCommandTime = millis();
        }
    }

    // Retrieve the value from the registers for the current mode
    float receivedValue = registersToFloat(mode);

    // Execute the corresponding function based on the mode
    switch (mode) {
        case 0: // moveToAngle - continuous mode, always apply
            stepper->moveToAngle(receivedValue);
            mb.Hreg(22, 1); // Set loop mode to closed-loop
            break;
            
        case 1: // setRPM - continuous mode, always apply
            stepper->setRPM(receivedValue);
            break;
            
        case 2: // moveSteps - command mode, execute only on new commands
            if (hasNewCommand && receivedValue != 0.0f && isValidFloat(receivedValue)) {
                stepper->moveSteps(receivedValue);
            }
            
            // Non-blocking wait - check if motor has finished
            if (commandState == CMD_STATE_EXECUTING) {
                mb.task(); // Keep processing Modbus during movement
                
                // Check if motor has stopped or mode changed
                if (stepper->getMotorState() == 0 || mb.Hreg(17) != 2) {
                    // Command completed - reset registers and state
                    mb.Hreg(regPairs[2][0], 0);
                    mb.Hreg(regPairs[2][1], 0);
                    commandState = CMD_STATE_IDLE;
                }
                
                // Timeout protection - prevent infinite waiting (10 second timeout)
                if (millis() - lastCommandTime > 10000) {
                    mb.Hreg(regPairs[2][0], 0);
                    mb.Hreg(regPairs[2][1], 0);
                    commandState = CMD_STATE_IDLE;
                }
            }
            break;
            
        case 3: // moveAngle - command mode, execute only on new commands
            if (hasNewCommand && receivedValue != 0.0f && isValidFloat(receivedValue)) {
                stepper->moveAngle(receivedValue);
            }
            
            // Non-blocking wait - check if motor has finished
            if (commandState == CMD_STATE_EXECUTING) {
                mb.task(); // Keep processing Modbus during movement
                
                // Check if motor has stopped or mode changed
                if (stepper->getMotorState() == 0 || mb.Hreg(17) != 3) {
                    // Command completed - reset registers and state
                    mb.Hreg(regPairs[3][0], 0);
                    mb.Hreg(regPairs[3][1], 0);
                    commandState = CMD_STATE_IDLE;
                }
                
                // Timeout protection - prevent infinite waiting (10 second timeout)
                if (millis() - lastCommandTime > 10000) {
                    mb.Hreg(regPairs[3][0], 0);
                    mb.Hreg(regPairs[3][1], 0);
                    commandState = CMD_STATE_IDLE;
                }
            }
            break;
    }

    // Reset command state when mode changes
    if (mode != previousMode) {
        commandState = CMD_STATE_IDLE;
    }
    
    // Update the previous mode
    previousMode = mode;

    // Update stepper motor settings based on Modbus registers
    float maxAccel = registersToFloat(5); // Max acceleration (registers 19, 18)
    if (isValidFloat(maxAccel) && maxAccel > 0) {
        stepper->setMaxAcceleration(maxAccel * 200);
        stepper->setMaxDeceleration(maxAccel * 200);
    }

    if (mode != 1) { // Only update max velocity if not in RPM mode
        float maxSpeed = registersToFloat(6); // Max velocity (registers 21, 20)
        if (isValidFloat(maxSpeed) && maxSpeed > 0) {
            stepper->setMaxVelocity(maxSpeed * RPMTOSTEPSS);
        }
    }

    uint8_t brakeMode = mb.Hreg(16); // Brake mode (register 16)
    if (brakeMode <= 3) { // Validate brake mode
        stepper->setBrakeMode(brakeMode);
    }

    uint8_t loopMode = mb.Hreg(22); // Loop mode (register 22)
    if (loopMode == 0) {
        stepper->disableClosedLoop();
    } else {
        stepper->enableClosedLoop();
    }

    uint8_t runCurrent = mb.Hreg(24); // Running current (register 24)
    if (runCurrent <= 100) { // Validate current percentage
        stepper->setCurrent(runCurrent);
    }
}

// Convert a float to two 16-bit registers
void ModbusUtils::floatToRegisters(float value, uint16_t *regs) {
    union {
        float f;
        uint16_t u[2];
    } data;
    data.f = value;
    regs[0] = data.u[0]; // Low part of the float
    regs[1] = data.u[1]; // High part of the float
}

// Convert two 16-bit registers to a float
float ModbusUtils::registersToFloat(uint8_t index) {
    union {
        uint16_t reg[2];
        float value;
    } converter;

    converter.reg[1] = mb.Hreg(regPairs[index][0]); // High part
    converter.reg[0] = mb.Hreg(regPairs[index][1]); // Low part

    return converter.value;
}

// Check if a float value is valid (not NaN, not Inf, within reasonable range)
bool ModbusUtils::isValidFloat(float value) {
    // Check for NaN
    if (value != value) return false;
    
    // Check for infinity
    if (value == __builtin_inff() || value == -__builtin_inff()) return false;
    
    // Check for extremely large values that might indicate corruption
    if (value > 1e9f || value < -1e9f) return false;
    
    return true;
}
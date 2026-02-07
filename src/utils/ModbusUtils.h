#ifndef MODBUSUTILS_H
#define MODBUSUTILS_H

#include <ModbusRTU.h>

// Forward declaration of UstepperS32 to avoid circular dependency
class UstepperS32;

// Conversion factor: Converts RPM to steps per second
#define RPMTOSTEPSS 200.0 / 60.0

// Command state machine states
#define CMD_STATE_IDLE       0
#define CMD_STATE_EXECUTING  1

/**
 * @class ModbusUtils
 * @brief Utility class for handling Modbus communication and stepper motor control.
 */
class ModbusUtils {
public:
    /**
     * @brief Initializes Modbus communication.
     * @param stepper Reference to the UstepperS32 object.
     * @param id Modbus slave ID.
     * @param baud Baud rate for Modbus communication.
     */
    static void modbusEnable(UstepperS32 *stepper, uint8_t id, uint32_t baud);

    /**
     * @brief Handles Modbus communication and updates stepper motor control.
     * @param stepper Reference to the UstepperS32 object.
     */
    static void handleModbus(UstepperS32 *stepper);

private:
    static ModbusRTU mb; ///< ModbusRTU instance for handling Modbus communication.
    static uint16_t regs[2]; ///< Array to hold two 16-bit registers for float conversion.
    
    // State tracking for robust command handling
    static uint8_t previousMode; ///< Previous mode for change detection
    static uint8_t commandState; ///< Current command execution state
    static uint32_t lastCommandTime; ///< Timestamp of last command for timeout
    static uint16_t lastCommandSeq[4]; ///< Track command sequence per mode

    /**
     * @brief Register pairs for Modbus communication.
     * Each pair represents two 16-bit Modbus holding registers used to store a 32-bit float value.
     * The first element in each pair is the high register, and the second is the low register.
     */
    static const uint8_t regPairs[10][2];

    /**
     * @brief Converts a float value to two 16-bit registers.
     * @param value The float value to convert.
     * @param regs Pointer to the array where the converted registers will be stored.
     */
    static void floatToRegisters(float value, uint16_t *regs);

    /**
     * @brief Converts two 16-bit registers to a float value.
     * @param index Index of the register pair in regPairs.
     * @return The converted float value.
     */
    static float registersToFloat(uint8_t index);
    
    /**
     * @brief Validates if a float value is valid (not NaN, not Inf, within reasonable range).
     * @param value The float value to validate.
     * @return True if the value is valid, false otherwise.
     */
    static bool isValidFloat(float value);
};

#endif // MODBUSUTILS_H
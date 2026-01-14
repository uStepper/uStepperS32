#ifndef MODBUSMASTERUTILS_H
#define MODBUSMASTERUTILS_H

#include <ModbusMaster.h>

/**
 * @class ModbusMasterUtils
 * @brief Robust Modbus RTU master wrapper with automatic retry and error handling.
 * 
 * This class wraps the ModbusMaster library to provide:
 * - Automatic retries on communication failures
 * - Proper inter-frame timing
 * - Float read/write helpers
 * - Command verification
 * - Error tracking and recovery
 */
class ModbusMasterUtils {
public:
    /**
     * @brief Initialize Modbus master communication.
     * @param slaveId The Modbus slave ID to communicate with.
     * @param serial Reference to the HardwareSerial port (e.g., Serial2).
     * @param baud Baud rate (default: 9600).
     */
    void begin(uint8_t slaveId, HardwareSerial &serial, uint32_t baud = 9600);

    /**
     * @brief Write a float value to two consecutive Modbus registers.
     * @param startReg Starting register address.
     * @param value Float value to write.
     * @return true if successful, false otherwise.
     */
    bool writeFloat(uint16_t startReg, float value);

    /**
     * @brief Read a float value from two consecutive Modbus registers.
     * @param startReg Starting register address.
     * @return Float value, or NAN on failure.
     */
    float readFloat(uint16_t startReg);

    /**
     * @brief Write a single 16-bit register.
     * @param reg Register address.
     * @param value Value to write.
     * @return true if successful, false otherwise.
     */
    bool writeRegister(uint16_t reg, uint16_t value);

    /**
     * @brief Read a single 16-bit register.
     * @param reg Register address.
     * @param value Pointer to store the read value.
     * @return true if successful, false otherwise.
     */
    bool readRegister(uint16_t reg, uint16_t *value);

    /**
     * @brief Get the number of consecutive communication errors.
     * @return Error count.
     */
    uint8_t getErrorCount() { return _errorCount; }

    /**
     * @brief Reset error counter and clear serial buffer.
     */
    void resetErrors();

    /**
     * @brief Set maximum retry attempts (default: 3).
     * @param retries Number of retry attempts.
     */
    void setMaxRetries(uint8_t retries) { _maxRetries = retries; }

    /**
     * @brief Set delay between retries in ms (default: 50).
     * @param delayMs Delay in milliseconds.
     */
    void setRetryDelay(uint16_t delayMs) { _retryDelay = delayMs; }

private:
    ModbusMaster _modbus;
    HardwareSerial *_serial = nullptr;
    uint8_t _maxRetries = 3;
    uint16_t _retryDelay = 50;
    uint8_t _errorCount = 0;
    uint32_t _lastCommTime = 0;
    
    static const uint16_t INTER_FRAME_DELAY = 10; // ms between frames
    
    void ensureInterFrameDelay();
    void clearSerialBuffer();
    void recordSuccess();
    void recordError();
};

#endif // MODBUSMASTERUTILS_H

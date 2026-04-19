#ifndef __ENCODER_CALIBRATION_H
#define __ENCODER_CALIBRATION_H

#include <stdint.h>

/**
 * @brief Encoder linearization calibration using a 512-point lookup table.
 *
 * The TLE5012B magnetic encoder can have non-linearities from magnet placement.
 * This class provides:
 *   - A calibration routine that steps the motor through 512 positions and
 *     records the encoder reading at each (building a forward lookup table).
 *   - Flash storage so calibration persists across reboots.
 *   - A runtime reverse-lookup with linear interpolation to convert raw
 *     encoder readings into linearized angles.
 */

#define CALIBRATION_TABLE_SIZE 512
#define ENCODER_COUNTS_PER_REV 32768  // TLE5012B 15-bit
#define CALIBRATION_SAMPLES_PER_STEP 32
#define CALIBRATION_SETTLE_MS 100

// Flash storage: use the last 16KB sector of STM32F401 (Sector 7: 0x08060000)
// STM32F401CC has 256KB flash total, sector 7 = 0x08060000..0x0807FFFF (but
// STM32F401CC only has 256KB so sector 5 at 0x08020000 is the last 128KB sector).
// Actually STM32F401CCU6 has 256KB flash: sectors 0-5
//   Sector 0: 0x08000000 (16KB)
//   Sector 1: 0x08004000 (16KB)
//   Sector 2: 0x08008000 (16KB)
//   Sector 3: 0x0800C000 (16KB)
//   Sector 4: 0x08010000 (64KB)
//   Sector 5: 0x08020000 (128KB)
// We'll use the last 2KB of sector 5 (end of flash) to store the table.
// 512 entries * 2 bytes = 1024 bytes + header = ~1040 bytes
#define CALIBRATION_FLASH_BASE 0x0803F800  // Last 2KB of sector 5

#define CALIBRATION_MAGIC 0xCA1B  // Magic number to validate stored data

typedef struct __attribute__((packed)) {
    uint16_t magic;          // CALIBRATION_MAGIC if valid
    uint16_t tableSize;      // Should be CALIBRATION_TABLE_SIZE
    uint16_t table[CALIBRATION_TABLE_SIZE];  // Forward LUT: step_index -> encoder_angle
    uint16_t checksum;       // Simple sum checksum
} CalibrationData_t;

class EncoderCalibration
{
public:
    EncoderCalibration();

    /**
     * @brief Check if valid calibration data exists in flash.
     */
    bool isCalibrated(void);

    /**
     * @brief Load calibration data from flash into RAM.
     * @return true if valid data was loaded.
     */
    bool loadFromFlash(void);

    /**
     * @brief Save current calibration data to flash.
     * @return true if write succeeded.
     */
    bool saveToFlash(void);

    /**
     * @brief Erase calibration data from flash.
     */
    bool eraseCalibration(void);

    /**
     * @brief Forward lookup: given a step index (0..511), return expected encoder angle.
     */
    uint16_t getEncoderAngleForStep(uint16_t stepIndex);

    /**
     * @brief Reverse lookup with linear interpolation:
     *        Given a raw encoder reading (0..32767), return the linearized angle (0..32767).
     *        This is the main runtime function used in the control loop.
     */
    int32_t linearize(uint16_t rawAngle);

    /**
     * @brief Set a calibration table entry (used during calibration routine).
     */
    void setTableEntry(uint16_t index, uint16_t encoderAngle);

    /**
     * @brief Finalize calibration: compute checksum, mark as valid.
     */
    void finalizeCalibration(void);

    /**
     * @brief Get the calibration table pointer (for debugging/serial output).
     */
    const uint16_t* getTable(void) const { return &data.table[0]; }

    /**
     * @brief Check if calibration is loaded and ready for use.
     */
    bool isReady(void) const { return calibrationReady; }

private:
    CalibrationData_t data;
    bool calibrationReady;

    uint16_t computeChecksum(void);

    // Pre-built reverse lookup index for fast binary search
    // Maps encoder range to nearest table entry for O(1) reverse lookup
    // We divide 32768 encoder counts into 512 bins
    uint16_t reverseIndex[CALIBRATION_TABLE_SIZE];
    void buildReverseIndex(void);
};

#endif

#ifndef __ENCODER_CALIBRATION_H
#define __ENCODER_CALIBRATION_H

#include <stdint.h>

/**
 * @brief Encoder linearization using a 512-point correction table.
 *
 * The TLE5012B magnetic encoder can have non-linearities from magnet placement.
 * This class provides:
 *   - A continuous-speed calibration routine that runs the motor at constant
 *     velocity and samples (XACTUAL, encoder) pairs over multiple revolutions,
 *     directly computing a correction table by averaging.
 *   - Flash storage so calibration persists across reboots.
 *   - An O(1) runtime correction with linear interpolation.
 */

#define CALIBRATION_TABLE_SIZE 512
#define ENCODER_COUNTS_PER_REV 32768  // TLE5012B 15-bit
#define CALIBRATION_NUM_REVOLUTIONS 8  // Number of revolutions to average over

// Flash storage: last 2KB of sector 5 on STM32F401CCU6 (256KB flash, sectors 0-5)
//   Sector 5: 0x08020000 (128KB)
// 512 entries * 2 bytes = 1024 bytes + header = ~1040 bytes
#define CALIBRATION_FLASH_BASE 0x0803F800

#define CALIBRATION_MAGIC 0xCA1C  // Magic number (changed: new table format)

typedef struct __attribute__((packed)) {
    uint16_t magic;          // CALIBRATION_MAGIC if valid
    uint16_t tableSize;      // Should be CALIBRATION_TABLE_SIZE
    int16_t  correction[CALIBRATION_TABLE_SIZE];  // Signed correction per bin
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
     * @brief Set a correction table entry directly (used by calibration routine).
     * @param index Bin index (0..511)
     * @param correction Signed correction in encoder counts
     */
    void setCorrectionEntry(uint16_t index, int16_t correction);

    /**
     * @brief Apply linearization: given a raw encoder reading (0..32767),
     *        return the corrected angle (0..32767).
     *        O(1) with linear interpolation between bins.
     */
    int32_t linearize(uint16_t rawAngle);

    /**
     * @brief Finalize calibration: compute checksum, mark as valid.
     */
    void finalizeCalibration(void);

    /**
     * @brief Get the correction table pointer (for debugging/serial output).
     */
    const int16_t* getCorrectionTable(void) const { return data.correction; }

    /**
     * @brief Check if calibration is loaded and ready for use.
     */
    bool isReady(void) const { return calibrationReady; }

private:
    CalibrationData_t data;
    bool calibrationReady;

    uint16_t computeChecksum(void);
};

#endif

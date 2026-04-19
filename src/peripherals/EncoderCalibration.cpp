#include "EncoderCalibration.h"
#include <string.h>
#include <stdlib.h>
#include <stm32f4xx.h>  // For __disable_irq, __enable_irq, CMSIS core registers

// STM32F4 Flash register definitions (direct register access, avoids HAL dependency)
#define FLASH_BASE_ADDR     0x40023C00UL
#define FLASH_ACR           (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x00))
#define FLASH_KEYR          (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x04))
#define FLASH_SR            (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x0C))
#define FLASH_CR            (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x10))

#define FLASH_KEY1          0x45670123UL
#define FLASH_KEY2          0xCDEF89ABUL

#define FLASH_SR_BSY        (1UL << 16)
#define FLASH_SR_EOP        (1UL << 0)
#define FLASH_SR_PGSERR     (1UL << 7)
#define FLASH_SR_PGPERR     (1UL << 6)
#define FLASH_SR_PGAERR     (1UL << 5)
#define FLASH_SR_WRPERR     (1UL << 4)

#define FLASH_CR_PG         (1UL << 0)
#define FLASH_CR_SER        (1UL << 1)
#define FLASH_CR_STRT       (1UL << 16)
#define FLASH_CR_LOCK       (1UL << 31)
#define FLASH_CR_PSIZE_WORD (2UL << 8)   // 32-bit parallelism (2.7-3.6V)
#define FLASH_CR_SNB_5      (5UL << 3)   // Sector 5

static void flash_wait_busy(void)
{
    while (FLASH_SR & FLASH_SR_BSY) { }
}

static bool flash_unlock(void)
{
    if (FLASH_CR & FLASH_CR_LOCK)
    {
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
    }
    return !(FLASH_CR & FLASH_CR_LOCK);
}

static void flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
}

static bool flash_erase_sector5(void)
{
    flash_wait_busy();
    // Clear error flags
    FLASH_SR = FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR;

    FLASH_CR = FLASH_CR_SER | FLASH_CR_SNB_5 | FLASH_CR_PSIZE_WORD;
    FLASH_CR |= FLASH_CR_STRT;

    flash_wait_busy();

    bool ok = !(FLASH_SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR));
    FLASH_CR &= ~(FLASH_CR_SER);
    return ok;
}

static bool flash_program_word(uint32_t address, uint32_t data)
{
    flash_wait_busy();
    FLASH_SR = FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR;

    FLASH_CR = FLASH_CR_PG | FLASH_CR_PSIZE_WORD;
    *(volatile uint32_t *)address = data;

    flash_wait_busy();

    bool ok = !(FLASH_SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR));
    FLASH_CR &= ~FLASH_CR_PG;
    return ok;
}

EncoderCalibration::EncoderCalibration()
    : calibrationReady(false)
{
    memset(&data, 0, sizeof(data));
    memset(reverseIndex, 0, sizeof(reverseIndex));
}

bool EncoderCalibration::isCalibrated(void)
{
    const CalibrationData_t *flashData = (const CalibrationData_t *)CALIBRATION_FLASH_BASE;
    return (flashData->magic == CALIBRATION_MAGIC &&
            flashData->tableSize == CALIBRATION_TABLE_SIZE);
}

bool EncoderCalibration::loadFromFlash(void)
{
    const CalibrationData_t *flashData = (const CalibrationData_t *)CALIBRATION_FLASH_BASE;

    if (flashData->magic != CALIBRATION_MAGIC ||
        flashData->tableSize != CALIBRATION_TABLE_SIZE)
    {
        calibrationReady = false;
        return false;
    }

    // Copy from flash to RAM
    memcpy(&data, flashData, sizeof(CalibrationData_t));

    // Verify checksum
    uint16_t expected = data.checksum;
    if (computeChecksum() != expected)
    {
        calibrationReady = false;
        return false;
    }

    buildReverseIndex();
    calibrationReady = true;
    return true;
}

bool EncoderCalibration::saveToFlash(void)
{
    if (!flash_unlock())
        return false;

    // Erase sector 5 (128KB)
    __disable_irq();
    bool ok = flash_erase_sector5();
    __enable_irq();

    if (!ok)
    {
        flash_lock();
        return false;
    }

    // Write data word by word (32-bit aligned)
    // Copy to aligned buffer to avoid packed struct issues
    uint32_t alignedBuf[(sizeof(CalibrationData_t) + 3) / 4];
    memcpy(alignedBuf, &data, sizeof(CalibrationData_t));

    uint32_t dest = CALIBRATION_FLASH_BASE;
    uint32_t words = (sizeof(CalibrationData_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++)
    {
        if (!flash_program_word(dest, alignedBuf[i]))
        {
            flash_lock();
            return false;
        }
        dest += 4;
    }

    flash_lock();
    return true;
}

bool EncoderCalibration::eraseCalibration(void)
{
    if (!flash_unlock())
        return false;

    __disable_irq();
    bool ok = flash_erase_sector5();
    __enable_irq();

    flash_lock();
    calibrationReady = false;
    return ok;
}

uint16_t EncoderCalibration::getEncoderAngleForStep(uint16_t stepIndex)
{
    if (stepIndex >= CALIBRATION_TABLE_SIZE)
        stepIndex = CALIBRATION_TABLE_SIZE - 1;
    return data.table[stepIndex];
}

void EncoderCalibration::setTableEntry(uint16_t index, uint16_t encoderAngle)
{
    if (index < CALIBRATION_TABLE_SIZE)
    {
        data.table[index] = encoderAngle;
    }
}

void EncoderCalibration::finalizeCalibration(void)
{
    data.magic = CALIBRATION_MAGIC;
    data.tableSize = CALIBRATION_TABLE_SIZE;
    data.checksum = computeChecksum();
    buildReverseIndex();
    calibrationReady = true;
}

uint16_t EncoderCalibration::computeChecksum(void)
{
    uint16_t sum = 0;
    sum += data.magic;
    sum += data.tableSize;
    for (uint16_t i = 0; i < CALIBRATION_TABLE_SIZE; i++)
    {
        sum += data.table[i];
    }
    return sum;
}

void EncoderCalibration::buildReverseIndex(void)
{
    // For each bin of the raw angle space (512 bins of 64 counts each),
    // store the table index whose raw encoder value is closest to that bin center.
    uint16_t binSize = ENCODER_COUNTS_PER_REV / CALIBRATION_TABLE_SIZE; // 64

    for (uint16_t bin = 0; bin < CALIBRATION_TABLE_SIZE; bin++)
    {
        uint16_t targetAngle = bin * binSize + binSize / 2;
        uint16_t bestIdx = 0;
        int32_t bestDist = 0x7FFFFFFF;

        for (uint16_t j = 0; j < CALIBRATION_TABLE_SIZE; j++)
        {
            int32_t dist = (int32_t)data.table[j] - (int32_t)targetAngle;
            if (dist > (int32_t)(ENCODER_COUNTS_PER_REV / 2))
                dist -= ENCODER_COUNTS_PER_REV;
            else if (dist < -(int32_t)(ENCODER_COUNTS_PER_REV / 2))
                dist += ENCODER_COUNTS_PER_REV;

            int32_t absDist = dist < 0 ? -dist : dist;
            if (absDist < bestDist)
            {
                bestDist = absDist;
                bestIdx = j;
            }
        }
        reverseIndex[bin] = bestIdx;
    }
}

int32_t EncoderCalibration::linearize(uint16_t rawAngle)
{
    if (!calibrationReady)
        return rawAngle;

    // table[i] = absolute raw encoder angle when motor is at position i/512 rev.
    // The linearized angle for step i = i * 64 (evenly spaced, 0..32767).
    //
    // Given an absolute raw reading, find the two consecutive table entries
    // that bracket it, interpolate, and return the linearized angle.
    // The table is monotonically increasing with one wraparound.

    const uint16_t binSize = ENCODER_COUNTS_PER_REV / CALIBRATION_TABLE_SIZE; // 64

    // Use reverse index for O(1) approximate lookup
    uint16_t bin = rawAngle / binSize;
    if (bin >= CALIBRATION_TABLE_SIZE)
        bin = CALIBRATION_TABLE_SIZE - 1;
    uint16_t startIdx = reverseIndex[bin];

    // Search nearby consecutive pairs for the bracket containing rawAngle
    for (int16_t offset = -4; offset <= 4; offset++)
    {
        uint16_t i = (uint16_t)(((int16_t)startIdx + offset + CALIBRATION_TABLE_SIZE) % CALIBRATION_TABLE_SIZE);
        uint16_t j = (i + 1) % CALIBRATION_TABLE_SIZE;

        int32_t ai = (int32_t)data.table[i];
        int32_t aj = (int32_t)data.table[j];

        // Span from table[i] to table[j], always positive for monotonic table
        int32_t span = aj - ai;
        if (span < 0)
            span += ENCODER_COUNTS_PER_REV;

        if (span == 0)
            continue;

        // Distance from table[i] to rawAngle, wrapped positive
        int32_t d = (int32_t)rawAngle - ai;
        if (d < 0)
            d += ENCODER_COUNTS_PER_REV;

        // rawAngle is in [table[i], table[j]] if 0 <= d <= span
        if (d <= span)
        {
            // Interpolate: linearized = i*64 + (d/span)*64
            int32_t linAngle = (int32_t)i * binSize + (d * binSize + span / 2) / span;

            // Wrap to [0, 32768)
            linAngle %= ENCODER_COUNTS_PER_REV;
            if (linAngle < 0)
                linAngle += ENCODER_COUNTS_PER_REV;

            return linAngle;
        }
    }

    // Fallback: return best-guess from reverse index
    return ((int32_t)startIdx * binSize) % ENCODER_COUNTS_PER_REV;
}

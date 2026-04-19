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
    memset(correctionTable, 0, sizeof(correctionTable));
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

    buildCorrectionTable();
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
    buildCorrectionTable();
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

void EncoderCalibration::buildCorrectionTable(void)
{
    // The forward table maps: table[i] = raw encoder reading at ideal position i*64.
    // We need a reverse correction: given a raw angle, what correction to add.
    //
    // For each of 512 bins of raw encoder space (bin b covers raw angles b*64 to (b+1)*64-1),
    // find which forward table entry bracket contains this raw angle, interpolate to get
    // the ideal linearized angle, then store the correction = linearized - rawBinCenter.

    const int32_t binSize = ENCODER_COUNTS_PER_REV / CALIBRATION_TABLE_SIZE; // 64
    const int32_t halfRev = ENCODER_COUNTS_PER_REV / 2;

    for (uint16_t b = 0; b < CALIBRATION_TABLE_SIZE; b++)
    {
        int32_t rawCenter = (int32_t)b * binSize + binSize / 2;

        // Find the forward table bracket [i, i+1] that contains rawCenter
        bool found = false;
        for (uint16_t i = 0; i < CALIBRATION_TABLE_SIZE; i++)
        {
            uint16_t j = (i + 1) % CALIBRATION_TABLE_SIZE;

            int32_t ai = (int32_t)data.table[i];
            int32_t aj = (int32_t)data.table[j];

            // Compute span from table[i] to table[j], handling wrap
            int32_t span = aj - ai;
            if (span < 0) span += ENCODER_COUNTS_PER_REV;
            // Skip degenerate spans (protect against div-by-zero and bad entries)
            if (span <= 0 || span > ENCODER_COUNTS_PER_REV / 2) continue;

            // Distance from table[i] to rawCenter, wrapped positive
            int32_t d = rawCenter - ai;
            if (d < 0) d += ENCODER_COUNTS_PER_REV;

            if (d <= span)
            {
                // Interpolate: linearized = i*binSize + (d/span)*binSize
                // Using fixed-point: (d * binSize * 256 / span) for precision
                int32_t frac256 = (d * 256 + span / 2) / span;  // 0..256
                int32_t linearized = (int32_t)i * binSize + (frac256 * binSize + 128) / 256;
                linearized %= ENCODER_COUNTS_PER_REV;

                int32_t corr = linearized - rawCenter;
                // Wrap correction to [-halfRev, halfRev)
                if (corr > halfRev) corr -= ENCODER_COUNTS_PER_REV;
                if (corr < -halfRev) corr += ENCODER_COUNTS_PER_REV;

                correctionTable[b] = (int16_t)corr;
                found = true;
                break;
            }
        }
        if (!found)
        {
            correctionTable[b] = 0;
        }
    }

    // Smooth the correction table with a 5-tap moving average to reduce noise
    int16_t smoothed[CALIBRATION_TABLE_SIZE];
    for (uint16_t b = 0; b < CALIBRATION_TABLE_SIZE; b++)
    {
        int32_t sum = 0;
        for (int8_t k = -2; k <= 2; k++)
        {
            uint16_t idx = (b + k + CALIBRATION_TABLE_SIZE) % CALIBRATION_TABLE_SIZE;
            sum += correctionTable[idx];
        }
        smoothed[b] = (int16_t)(sum / 5);
    }
    memcpy(correctionTable, smoothed, sizeof(correctionTable));
}

int32_t EncoderCalibration::linearize(uint16_t rawAngle)
{
    if (!calibrationReady)
        return rawAngle;

    // O(1) correction lookup with linear interpolation between bins
    const uint16_t binSize = ENCODER_COUNTS_PER_REV / CALIBRATION_TABLE_SIZE; // 64

    uint16_t bin = rawAngle / binSize;           // 0..511
    uint16_t frac = rawAngle - bin * binSize;    // 0..63 (remainder within bin)

    if (bin >= CALIBRATION_TABLE_SIZE)
        bin = CALIBRATION_TABLE_SIZE - 1;

    uint16_t nextBin = (bin + 1) % CALIBRATION_TABLE_SIZE;

    // Linear interpolation of correction between adjacent bins
    int32_t c0 = correctionTable[bin];
    int32_t c1 = correctionTable[nextBin];

    // Handle wrap-around in correction values (shouldn't happen normally,
    // but be safe for bins near the 0/32768 boundary)
    int32_t cdiff = c1 - c0;
    if (cdiff > ENCODER_COUNTS_PER_REV / 4) cdiff -= ENCODER_COUNTS_PER_REV;
    if (cdiff < -ENCODER_COUNTS_PER_REV / 4) cdiff += ENCODER_COUNTS_PER_REV;

    int32_t correction = c0 + (cdiff * (int32_t)frac + binSize / 2) / binSize;

    int32_t result = (int32_t)rawAngle + correction;

    // Wrap to [0, 32768)
    result %= ENCODER_COUNTS_PER_REV;
    if (result < 0) result += ENCODER_COUNTS_PER_REV;

    return result;
}

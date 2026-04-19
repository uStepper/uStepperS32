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

    memcpy(&data, flashData, sizeof(CalibrationData_t));

    uint16_t expected = data.checksum;
    if (computeChecksum() != expected)
    {
        calibrationReady = false;
        return false;
    }

    calibrationReady = true;
    return true;
}

bool EncoderCalibration::saveToFlash(void)
{
    if (!flash_unlock())
        return false;

    __disable_irq();
    bool ok = flash_erase_sector5();
    __enable_irq();

    if (!ok)
    {
        flash_lock();
        return false;
    }

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

void EncoderCalibration::setCorrectionEntry(uint16_t index, int16_t correction)
{
    if (index < CALIBRATION_TABLE_SIZE)
    {
        data.correction[index] = correction;
    }
}

void EncoderCalibration::finalizeCalibration(void)
{
    data.magic = CALIBRATION_MAGIC;
    data.tableSize = CALIBRATION_TABLE_SIZE;
    data.checksum = computeChecksum();
    calibrationReady = true;
}

uint16_t EncoderCalibration::computeChecksum(void)
{
    uint16_t sum = 0;
    sum += data.magic;
    sum += data.tableSize;
    for (uint16_t i = 0; i < CALIBRATION_TABLE_SIZE; i++)
    {
        sum += (uint16_t)data.correction[i];
    }
    return sum;
}

int32_t EncoderCalibration::linearize(uint16_t rawAngle)
{
    if (!calibrationReady)
        return rawAngle;

    const uint16_t binSize = ENCODER_COUNTS_PER_REV / CALIBRATION_TABLE_SIZE; // 64

    uint16_t bin = rawAngle / binSize;           // 0..511
    uint16_t frac = rawAngle - bin * binSize;    // 0..63

    if (bin >= CALIBRATION_TABLE_SIZE)
        bin = CALIBRATION_TABLE_SIZE - 1;

    uint16_t nextBin = (bin + 1) % CALIBRATION_TABLE_SIZE;

    int32_t c0 = data.correction[bin];
    int32_t c1 = data.correction[nextBin];

    // Handle wrap-around at 0/32768 boundary
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

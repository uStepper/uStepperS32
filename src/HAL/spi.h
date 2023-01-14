#ifndef __SPI_H_
#define __SPI_H_
#include "gpio.h"
#include "stm32yyxx_ll_bus.h"
#include "stm32yyxx_ll_gpio.h"
#include "stm32yyxx_ll_spi.h"

typedef enum
{
	SPI_TRANSMISSION_SUCCESS,
	SPI_TRANSMISSION_TIMEOUT
} SpiTransmissionStatus_t;

#define SPITIMEOUT(reg, mask, setReset, timeout)                        \
	this->spiTransmissionStatus = SPI_TRANSMISSION_SUCCESS;             \
	this->timeoutCnt = 0;                                               \
	if (setReset)                                                       \
	{                                                                   \
		while (reg & mask)                                              \
		{                                                               \
			if (this->timeoutCnt > timeout)                             \
			{                                                           \
				this->spiTransmissionStatus = SPI_TRANSMISSION_TIMEOUT; \
				return 0;                                               \
			}                                                           \
			this->timeoutCnt++;                                         \
		}                                                               \
	}                                                                   \
	else                                                                \
	{                                                                   \
		while (!(reg & mask))                                           \
		{                                                               \
			if (this->timeoutCnt > timeout)                             \
			{                                                           \
				this->spiTransmissionStatus = SPI_TRANSMISSION_TIMEOUT; \
				return 0;                                               \
			}                                                           \
			this->timeoutCnt++;                                         \
		}                                                               \
	}

typedef enum
{
	activeLow,
	activeHigh
} csActivePolarity_t;

class Spi
{
  public:
	Spi(csActivePolarity_t csPolarity, GPIO mosi, GPIO miso, GPIO sck, GPIO cs, SPI_TypeDef *spiChannel);
	void init();
	void transmitBytes(uint8_t *data, uint8_t *response, uint8_t bytes);
	void transmitWords(uint16_t *data, uint16_t *response, uint8_t words);
	uint8_t transmit8BitData(uint8_t data, uint32_t timeout = 20);
	uint16_t transmit16BitData(uint16_t data, uint32_t timeout = 20);
	void csSet();
	void csReset();
	void releaseMosi();
	void engageMosi();
	SpiTransmissionStatus_t getTransmissionStatus();
	SPI_TypeDef *_spiChannel;

  private:
	GPIO _mosi;
	GPIO _miso;
	GPIO _cs;
	GPIO _sck;
	SpiTransmissionStatus_t spiTransmissionStatus = SPI_TRANSMISSION_SUCCESS;
	csActivePolarity_t chipSelectActivePolarity;
	int32_t timeoutCnt = 0;
};
#endif
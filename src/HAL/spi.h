#ifndef __SPI_H_
#define __SPI_H_
#include "stm32yyxx_ll_spi.h"
#include "stm32yyxx_ll_gpio.h"
#include "stm32yyxx_ll_bus.h"


struct spiSettings
{
	struct pins
	{
		uint32_t mosi;
		uint32_t miso;
		uint32_t cs;
		uint32_t sck;
	} _pins;
};

class Spi
{
  public:
	Spi();
	void init();
	uint8_t transmit8BitData(uint8_t data);
	uint16_t transmit16BitData(uint16_t data);
	void transmitBytes(uint8_t *data, uint8_t *response, uint8_t bytes);
	void csSet();
	void csReset();

  private:
	spiSettings settings;
};
#endif
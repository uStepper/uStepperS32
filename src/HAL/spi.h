#ifndef __SPI_H_
#define __SPI_H_
#include "stm32yyxx_ll_spi.h"
#include "stm32yyxx_ll_gpio.h"
#include "stm32yyxx_ll_bus.h"
#include "gpio.h" 

typedef struct SpiSettings
{
	struct pins
	{
		GPIO mosi;
		GPIO miso;
		GPIO cs;
		GPIO sck;
	} _pins;
	bool chipSelectActivePolarity;
} SpiSettings;

class Spi
{
  public:
	Spi(bool csPolarity, GPIO mosi, GPIO miso, GPIO sck, GPIO cs, SPI_TypeDef *spiChannel);
	void init();
	void transmitBytes(uint8_t *data, uint8_t *response, uint8_t bytes);
	void transmitWords(uint16_t *data, uint16_t *response, uint8_t words);
	uint8_t transmit8BitData(uint8_t data);
	uint16_t transmit16BitData(uint16_t data);
	void csSet();
	void csReset();

  private:
	GPIO _mosi;
	GPIO _miso;
	GPIO _cs;
	GPIO _sck;
	SPI_TypeDef *_spiChannel;
	bool chipSelectActivePolarity;
};
#endif
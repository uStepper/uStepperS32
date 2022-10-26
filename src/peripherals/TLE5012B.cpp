#include "TLE5012B.h"

TLE5012B::TLE5012B() : spiHandle(
									0, 
									GPIO(LL_GPIO_PIN_12, 12, GPIOC),
									 GPIO(LL_GPIO_PIN_11, 11, GPIOC),
									 GPIO(LL_GPIO_PIN_10, 10, GPIOC),
									 GPIO(LL_GPIO_PIN_15, 15, GPIOA),
									SPI3
							   )
{
}

/*
TLE5012B::TLE5012B() : spiHandle(
									0, 
									GPIO(LL_GPIO_PIN_7, 7, GPIOA),
									 GPIO(LL_GPIO_PIN_6, 6, GPIOA),
									 GPIO(LL_GPIO_PIN_5, 5, GPIOA),
									 GPIO(LL_GPIO_PIN_15, 15, GPIOA),
									SPI1
							   )
{
}
*/
/*
TLE5012B::TLE5012B() : spiHandle(
						   0,
						   GPIO(LL_GPIO_PIN_15, 15, GPIOB),
						   GPIO(LL_GPIO_PIN_14, 14, GPIOB),
						   GPIO(LL_GPIO_PIN_13, 13, GPIOB),
						   GPIO(LL_GPIO_PIN_12, 12, GPIOB),
						   SPI2)
{
}
*/
void TLE5012B::init()
{
	this->spiHandle.init();
}

void TLE5012B::sendCommand(uint16_t rw, uint16_t lock, uint16_t upd, uint16_t addr, uint16_t nd)
{
	uint16_t cmd = ( (rw << 15) | (lock << 11) | (upd << 10) | (addr << 4) | nd );

	spiHandle.transmit16BitData(cmd);
}

uint16_t TLE5012B::getAngle()
{
	spiHandle.csReset();
	sendCommand(0x1, 0x0, 0x0, 0x02, 0x0);
	this->spiHandle.releaseMosi();
	uint16_t angle = spiHandle.transmit16BitData(0x0000);
	spiHandle.csSet();
	this->spiHandle.engageMosi();
	this->angle = convertRawAngleToDeg(angle);
	return angle;
}

float TLE5012B::convertRawAngleToDeg(uint16_t raw)
{
	volatile float deg = 0.010986328125 * (float)(raw & 0x7FFF);
	return deg;
}

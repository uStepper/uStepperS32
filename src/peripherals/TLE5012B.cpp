#include "TLE5012B.h"

TLE5012B::TLE5012B()
{
}

void TLE5012B::init(Spi *handle)
{
	this->spiHandle = handle;
}

void TLE5012B::sendCommand(uint16_t rw, uint16_t lock, uint16_t upd, uint16_t addr, uint16_t nd)
{
	uint16_t cmd = ( (rw << 15) | (lock << 11) | (upd << 10) | (addr << 4) | nd );

	spiHandle->transmit16BitData(cmd);
	
}

uint16_t TLE5012B::getAngle()
{
	spiHandle->csReset();
	sendCommand(0x1, 0x0, 0x0, 0x02, 0x0);
	uint16_t angle = spiHandle->transmit16BitData(0x0000);
	spiHandle->csSet();
	this->angle = convertRawAngleToDeg(angle);
	return angle;
}

float TLE5012B::convertRawAngleToDeg(uint16_t raw)
{
	volatile float deg = 0.010986328125 * (float)(raw & 0x7FFF);
	return deg;
}

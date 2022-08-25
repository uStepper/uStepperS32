#ifndef __TLE5012B_H_
#define __TLE5012B_H_

#include "../HAL/spi.h"
//#include <Arduino.h>
class TLE5012B
{
  public:
	TLE5012B();
	void init(Spi *handle);
	uint16_t getAngle();
	float convertRawAngleToDeg(uint16_t raw);

  private:
	Spi *spiHandle;

	float angle;

	void sendCommand(uint16_t rw, uint16_t lock, uint16_t upd, uint16_t addr, uint16_t nd);
};

#endif
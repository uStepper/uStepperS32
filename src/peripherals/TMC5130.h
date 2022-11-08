#ifndef __TMC5130_H_
#define __TMC5130_H_

#include <Arduino.h>
#include "../HAL/spi.h"
#include "../HAL/gpio.h"
#include "TMC5130RegDef.h"

class TMC5130
{
  public:
	TMC5130();
	void init();
	int32_t readRegister(uint8_t address);
	int32_t writeRegister(uint8_t address, uint32_t datagram);
	void enableStealth();
	void setShaftDirection(bool direction);
	void setRampMode(uint8_t mode);
	void setAcceleration(uint32_t acceleration);
	void setDeceleration(uint32_t deceleration);
	void stop(void);
	void setVelocity(uint32_t velocity);
	void clearStall(void);
	void setDirection(bool direction);
	void setRPM(float rpm);

  private:
	Spi spiHandle;
	GPIO enablePin;
	GPIO sdPin;
	GPIO spiPin;
	/** Default acceleration profile for positioning mode */
	uint32_t VSTART = 0;
	uint32_t V1 = 0;
	uint32_t VMAX = 200000;
	uint32_t VSTOP = 10;
	uint16_t A1 = 600;
	uint16_t AMAX = 100;
	uint16_t DMAX = 600;
	uint16_t D1 = 600;
	
	uint8_t status;
	uint32_t lastReadValue;
	void reset(void);
	uint8_t current = 16;
	uint8_t holdCurrent = 0;
	uint8_t holdDelay = 0;
	/** STOP, VELOCITY, POSITION*/
	uint8_t mode = DRIVER_STOP;
	float rpmToVelocity = (float)(279620.267 * 200 * 256) / (12500000);
};

#endif
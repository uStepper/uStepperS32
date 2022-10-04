
#ifndef _USTEPPER_STM_H_
#define _USTEPPER_STM_H_

#include <Arduino.h>
#include "HAL/spi.h"
#include "peripherals/TMC5130.h"
#include "peripherals/TLE5012B.h"
#include "HAL/timer.h"


class UstepperSTM
{
	public:
	TLE5012B encoder;
	TMC5130 driver;
	/**
	 * @brief	Constructor of uStepper class
	 */
	UstepperSTM();
	void init();
	friend void _timerCallback();
  private:
	friend void timerIrqHandler(void);
	
};

#endif
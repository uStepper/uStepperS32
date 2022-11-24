
#ifndef _USTEPPER_STM_H_
#define _USTEPPER_STM_H_

#include <Arduino.h>
#include "HAL/spi.h"
#include "peripherals/TMC5130.h"
#include "peripherals/TLE5012B.h"
#include "HAL/timer.h"
#include "callbacks.h"


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
	
  private:
	friend void mainTimerCallback();
	friend void dropInStepInputEXTI();
	friend void dropInDirInputEXTI();
	friend void dropInEnableInputEXTI();
	
};

extern UstepperSTM *ptr;

#endif
#ifndef __GPIO_H_
#define __GPIO_H_

#include <inttypes.h>
#include "stm32yyxx_ll_gpio.h"

class GPIO
{
  public:
	init();

	toggle();

	set();

	reset();

	mode(uint32_t modeMask);

	mode(uint32_t modeM);

  private:
	uint32_t pinMask;
	uint32_t pin;
	GPIO_TypeDef *port;
};

#endif
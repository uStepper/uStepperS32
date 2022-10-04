#ifndef __GPIO_H_
#define __GPIO_H_

#include <inttypes.h>
#include "stm32yyxx_ll_gpio.h"
#include "stm32yyxx_ll_bus.h"

class GPIO
{
  public:
	GPIO(uint32_t pinMask, uint32_t pin, GPIO_TypeDef *port);
	GPIO();
	void toggle();
	void set();
	void reset();
	bool read();
	void configureSpi();
	void configureOutput();
	void configureInput();
	void configureAnalog();
	void enableClock();

  private:
	uint32_t _pinMask;
	uint32_t _pin;
	GPIO_TypeDef *_port;
	LL_GPIO_InitTypeDef _settings;
};

#endif
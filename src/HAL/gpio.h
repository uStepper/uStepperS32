#ifndef __GPIO_H_
#define __GPIO_H_

#include <inttypes.h>
#include "stm32yyxx_ll_gpio.h"
#include "stm32yyxx_ll_bus.h"
#include "stm32yyxx_ll_exti.h"
#include "stm32yyxx_ll_system.h"

class GPIO
{
  public:
	GPIO();
	GPIO(uint32_t pinMask, uint32_t pin, GPIO_TypeDef *port);
	void toggle();
	void set();
	void reset();
	bool read();
	void configureSpi(SPI_TypeDef *spiChannel);
	void configureOutput();
	void configureInput();
	void configureInterrupt(uint32_t pull = LL_GPIO_PULL_UP,
							uint32_t mode = LL_EXTI_MODE_IT,
							uint32_t trigger = LL_EXTI_TRIGGER_RISING,
							uint32_t prio = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	void configureAnalog();
	void enableClock();

  private:
	uint32_t _pinMask;
	uint32_t _pin;
	GPIO_TypeDef *_port;
	LL_GPIO_InitTypeDef _settings;
};

#define ENCODERMOSI GPIO(LL_GPIO_PIN_12, 12, GPIOC)
#define ENCODERMISO GPIO(LL_GPIO_PIN_11, 11, GPIOC)
#define ENCODERSCK GPIO(LL_GPIO_PIN_10, 10, GPIOC)
#define ENCODERCS GPIO(LL_GPIO_PIN_15, 15, GPIOA)
//#define DRIVERMOSI GPIO(LL_GPIO_PIN_15, 15, GPIOB)

#endif
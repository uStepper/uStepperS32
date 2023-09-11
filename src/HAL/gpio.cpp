#include "gpio.h"

GPIO::GPIO()
{
}

GPIO::GPIO(uint32_t pinMask, uint32_t pin, GPIO_TypeDef *port)
{
	this->_pinMask = pinMask;
	this->_pin = pin;
	this->_port = port;
}

void GPIO::toggle()
{
	this->_port->ODR ^= this->_pinMask;
}

void GPIO::set()
{
	this->_port->BSRR |= _pinMask;
}

void GPIO::reset()
{
	this->_port->BSRR |= _pinMask << 16;
}

bool GPIO::read()
{
	return (this->_port->IDR & this->_pinMask) ? 1 : 0;
}

void GPIO::configureSpi(SPI_TypeDef *spiChannel)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	this->enableClock();
	
	GPIO_InitStruct.Pin = _pinMask;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	if (spiChannel == SPI3)
	{
		GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	}
	else
	{
		GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	}
	
	LL_GPIO_Init(this->_port, &GPIO_InitStruct);
}

void GPIO::configureOutput()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	this->enableClock();
	
	GPIO_InitStruct.Pin = _pinMask;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(this->_port, &GPIO_InitStruct);
}

void GPIO::enableClock()
{
	if (this->_port == GPIOA)
	{
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	}
	else if (this->_port == GPIOB)
	{
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	}
	else if (this->_port == GPIOC)
	{
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	}
	else if (this->_port == GPIOD)
	{
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	}
	else if (this->_port == GPIOE)
	{
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
	}
	else if (this->_port == GPIOH)
	{
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
	}
}

void GPIO::configureInput()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	this->enableClock();

	GPIO_InitStruct.Pin = _pinMask;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(this->_port, &GPIO_InitStruct);
}

void GPIO::configureInterrupt(uint32_t pull, 
							  uint32_t mode, 
							  uint32_t trigger,
							  uint32_t prio)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	
	IRQn_Type interruptVector = this->_pin == 0 ? EXTI0_IRQn : this->_pin == 1 ? EXTI1_IRQn : this->_pin == 2 ? EXTI2_IRQn : this->_pin == 3 ? EXTI3_IRQn : this->_pin == 4 ? EXTI4_IRQn : (this->_pin >= 5 && this->_pin <= 9) ? EXTI9_5_IRQn : EXTI15_10_IRQn;
	uint32_t clockEnableMask = this->_port == GPIOA ? LL_AHB1_GRP1_PERIPH_GPIOA : this->_port == GPIOB ? LL_AHB1_GRP1_PERIPH_GPIOB : this->_port == GPIOC ? LL_AHB1_GRP1_PERIPH_GPIOC : this->_port == GPIOD ? LL_AHB1_GRP1_PERIPH_GPIOD : this->_port == GPIOE ? LL_AHB1_GRP1_PERIPH_GPIOE : LL_AHB1_GRP1_PERIPH_GPIOH;
	uint32_t sysCfgPortMask = this->_port == GPIOA ? LL_SYSCFG_EXTI_PORTA : this->_port == GPIOB ? LL_SYSCFG_EXTI_PORTB : this->_port == GPIOC ? LL_SYSCFG_EXTI_PORTC : this->_port == GPIOD ? LL_SYSCFG_EXTI_PORTD : this->_port == GPIOE ? LL_SYSCFG_EXTI_PORTE : LL_SYSCFG_EXTI_PORTH;
	uint32_t sysCfgLineMask = this->_pin == 0 ? LL_SYSCFG_EXTI_LINE0 : this->_pin == 1 ? LL_SYSCFG_EXTI_LINE1 : this->_pin == 2 ? LL_SYSCFG_EXTI_LINE2 : this->_pin == 3 ? LL_SYSCFG_EXTI_LINE3 : this->_pin == 4 ? LL_SYSCFG_EXTI_LINE4 : this->_pin == 5 ? LL_SYSCFG_EXTI_LINE5 : this->_pin == 6 ? LL_SYSCFG_EXTI_LINE6 : this->_pin == 7 ? LL_SYSCFG_EXTI_LINE7 : this->_pin == 8 ? LL_SYSCFG_EXTI_LINE8 : this->_pin == 9 ? LL_SYSCFG_EXTI_LINE9 : this->_pin == 10 ? LL_SYSCFG_EXTI_LINE10 : this->_pin == 11 ? LL_SYSCFG_EXTI_LINE11: this->_pin == 12 ? LL_SYSCFG_EXTI_LINE12 : this->_pin == 13 ? LL_SYSCFG_EXTI_LINE13 : this->_pin == 14 ? LL_SYSCFG_EXTI_LINE14 : LL_SYSCFG_EXTI_LINE15;
	uint32_t extiLineMask = this->_pin == 0 ? LL_EXTI_LINE_0 : this->_pin == 1 ? LL_EXTI_LINE_1 : this->_pin == 2 ? LL_EXTI_LINE_2 : this->_pin == 3 ? LL_EXTI_LINE_3 : this->_pin == 4 ? LL_EXTI_LINE_4 : this->_pin == 5 ? LL_EXTI_LINE_5 : this->_pin == 6 ? LL_EXTI_LINE_6 : this->_pin == 7 ? LL_EXTI_LINE_7 : this->_pin == 8 ? LL_EXTI_LINE_8 : this->_pin == 9 ? LL_EXTI_LINE_9 : this->_pin == 10 ? LL_EXTI_LINE_10 : this->_pin == 11 ? LL_EXTI_LINE_11 : this->_pin == 12 ? LL_EXTI_LINE_12 : this->_pin == 13 ? LL_EXTI_LINE_13 : this->_pin == 14 ? LL_EXTI_LINE_14 : LL_EXTI_LINE_15;
	
	LL_AHB1_GRP1_EnableClock(clockEnableMask);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	GPIO_InitStruct.Pin = this->_pinMask;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(this->_port, &GPIO_InitStruct);

	LL_SYSCFG_SetEXTISource(sysCfgPortMask, sysCfgLineMask);
	EXTI_InitStruct.Line_0_31 = extiLineMask;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	LL_EXTI_Init(&EXTI_InitStruct);

	//TODO: We need to disable IRQ in other "configure" functions for this pin, in case the pin is reconfigured runtime
	NVIC_SetPriority(interruptVector, prio);
	NVIC_EnableIRQ(interruptVector);
}

void GPIO::configureAnalog()
{
	
}
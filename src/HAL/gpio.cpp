#include "gpio.h"

GPIO::GPIO(uint32_t pinMask, uint32_t pin, GPIO_TypeDef *port)
{
	this->_pinMask = pinMask;
	this->_pin = pin;
	this->_port = port;
}

GPIO::GPIO()
{
}

void GPIO::toggle()
{
	
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
	
}

void GPIO::configureSpi()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	this->enableClock();
	
	GPIO_InitStruct.Pin = _pinMask;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
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
	
}

void GPIO::configureAnalog()
{
	
}
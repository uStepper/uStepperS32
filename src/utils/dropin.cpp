#include "../UstepperS32.h"

Dropin::Dropin() : stepPin(LL_GPIO_PIN_6, 6, GPIOA),
				   dirPin(LL_GPIO_PIN_2, 2, GPIOA),
				   enaPin(LL_GPIO_PIN_7, 7, GPIOA)
{
	
}

void Dropin::init() 
{
	this->dirPin.configureInterrupt(LL_GPIO_PULL_UP, LL_EXTI_MODE_IT, LL_EXTI_TRIGGER_RISING_FALLING, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	this->stepPin.configureInterrupt(LL_GPIO_PULL_UP, LL_EXTI_MODE_IT, LL_EXTI_TRIGGER_RISING, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	this->enaPin.configureInterrupt(LL_GPIO_PULL_UP, LL_EXTI_MODE_IT, LL_EXTI_TRIGGER_RISING_FALLING, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}
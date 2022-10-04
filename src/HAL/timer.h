#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32yyxx_ll_tim.h"
#include "stm32yyxx_ll_bus.h"

extern void (*timerCallback)(void);
void timerIrqHandler(void);
void TIM2_IRQHandler(void);
void timerInit(void);

#endif
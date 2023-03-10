#ifndef __TIMER_H__
#define __TIMER_H__
	#ifdef __cplusplus
	extern "C" {
	#endif

	#include "stm32yyxx_ll_tim.h"
	#include "stm32yyxx_ll_bus.h"
	#include "../callbacks.h"

	#define MAINTIMERINTERRUPTFREQUENCY 1541.85f
	#define MAINTIMERINTERRUPTPERIOD 1.0f / MAINTIMERINTERRUPTFREQUENCY
	
	void TIM4_IRQHandler(void);
	void mainTimerInit(void);
	void mainTimerPause();
	void mainTimerStart();

#ifdef __cplusplus
	}
	#endif
#endif
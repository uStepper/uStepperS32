#ifndef __TIMER_H__
#define __TIMER_H__
	#ifdef __cplusplus
	extern "C" {
	#endif

	#include "stm32yyxx_ll_tim.h"
	#include "stm32yyxx_ll_bus.h"
	#include "../callbacks.h"
	
	#define MAINTIMERINTERRUPTPERIOD 1.0f/1000.0f
	
	void TIM2_IRQHandler(void);
	void mainTimerInit(void);
	void mainTimerPause();
	void mainTimerStart();

#ifdef __cplusplus
	}
	#endif
#endif
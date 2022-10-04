#ifdef __cplusplus
extern "C" {
#endif

#include "timer.h"
void (*timerCallback)(void);
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void timerInit(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	
	NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM_InitStruct.Prescaler = 0;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 84000;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_EnableARRPreload(TIM2);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);

	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	TIM2->CR1 |= 0x01;
}

void timerIrqHandler(void)
{
	timerCallback();
}

/**
  * @brief This function handles TIM2 global interrupt.
  * ATTENTION: uncommented this IRQ handler in hardwarepackage, in "HardwareTimer.cpp"
  */
void TIM2_IRQHandler(void)
{
	timerIrqHandler();
	LL_TIM_ClearFlag_UPDATE(TIM2);
}

#ifdef __cplusplus
}
#endif
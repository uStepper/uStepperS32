
#include "timer.h"

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void mainTimerInit(void)
{
	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	
	NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	NVIC_EnableIRQ(TIM4_IRQn);
	
	TIM_InitStruct.Prescaler = 0;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 382164;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM4, &TIM_InitStruct);
	LL_TIM_EnableARRPreload(TIM4);
	LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM4);

	LL_TIM_ClearFlag_UPDATE(TIM4);
	LL_TIM_EnableIT_UPDATE(TIM4);
	mainTimerStart();
}

void mainTimerPause()
{
	TIM4->CR1 &= ~0x0001;
}

void mainTimerStart()
{
	TIM4->CNT = 0;
	TIM4->CR1 |= 0x0001;
}

/**
  * @brief This function handles TIM2 global interrupt.
  * ATTENTION: uncommented this IRQ handler in hardwarepackage, in "HardwareTimer.cpp"
  */
void TIM4_IRQHandler(void)
{
	callbacks._mainTimerCallback();
	LL_TIM_ClearFlag_UPDATE(TIM4);
}

void stepMotionControllerTimerInit(void)
{
	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_EnableIRQ(TIM3_IRQn);

	TIM_InitStruct.Prescaler = 0;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 1800; //100kHz
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM3, &TIM_InitStruct);
	LL_TIM_EnableARRPreload(TIM3);
	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM3);

	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
	stepMotionControllerTimerStart();
}

void stepMotionControllerTimerPause()
{
	TIM3->CR1 &= ~0x0001;
}

void stepMotionControllerTimerStart()
{
	TIM3->CNT = 0;
	TIM3->CR1 |= 0x0001;
}

/**
  * @brief This function handles TIM2 global interrupt.
  * ATTENTION: uncommented this IRQ handler in hardwarepackage, in "HardwareTimer.cpp"
  */
void TIM3_IRQHandler(void)
{
	callbacks._stepMotionControllerTimerCallback();
    LL_TIM_ClearFlag_UPDATE(TIM3);
}
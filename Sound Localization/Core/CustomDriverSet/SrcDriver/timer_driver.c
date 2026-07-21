#include "timer_driver.h"

/* Init TIM8 as ~192 kHz ADC sampling trigger (TRGO on update). */
void Custom_TIM8_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);

    TIM_InitStruct.Prescaler        = 0;
    TIM_InitStruct.CounterMode      = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload       = 884;    /* 170 MHz / 885 ≈ 192.09 kHz */
    TIM_InitStruct.ClockDivision    = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM8, &TIM_InitStruct);

    LL_TIM_DisableARRPreload(TIM8);
    LL_TIM_SetClockSource(TIM8, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM8, LL_TIM_TRGO_UPDATE);
    LL_TIM_SetTriggerOutput2(TIM8, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM8);
}

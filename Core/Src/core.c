#include "display.h"
#include "main.h"
#include "stm32f1xx_ll_tim.h"

void main_task()
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM1);
    display_init();
}
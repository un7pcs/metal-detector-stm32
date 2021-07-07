#include "display.h"
#include "main.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"

#define ADC_DATA_LENGTH 850

int16_t adc_buffer[ADC_DATA_LENGTH];

void main_task()
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableAllOutputs(TIM2);

    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableAllOutputs(TIM3);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_DATA_LENGTH);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_buffer);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    display_init();
}
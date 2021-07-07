#include "display.h"
#include "main.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"

#define ADC_DATA_LENGTH 850

int16_t adc_buffer[ADC_DATA_LENGTH];
volatile int k = 0;

void main_task()
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableAllOutputs(TIM3);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_DATA_LENGTH);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_buffer);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_ADC_Enable(ADC1);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

    display_init();
}

void dma_isr()
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
    k++;
    LL_TIM_DisableCounter(TIM3);
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC_DATA_LENGTH);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_buffer);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}
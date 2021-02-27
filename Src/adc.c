#include "adc.h"
static uint16_t adc_vals[N_ADC_CHANNELS]={0};
void ADC_Start(){
    HAL_ADC_Start_DMA(&hadc, adc_vals, N_ADC_CHANNELS);
}
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
    int_fast8_t i;
    for(i=N_ADC_CHANNELS-1; i--; i>=0){
        g_adcVals[i] *= (N_FILTER_ORDER-1);
        g_adcVals[i] /= N_FILTER_ORDER;
        #if N_FILTER_ORDER == 4 //do nothing, shift by 2 and divide by 4
        #else
        adc_vals[i] <<= 2; // do not loose precision
        adc_vals[i] /= N_FILTER_ORDER
        #endif
        g_adcVals[i] += adc_vals[i];
    }
}
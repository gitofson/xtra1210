#include "adc.h"
static uint16_t adc_vals[N_ADC_CHANNELS]={0};
void ADC_Start(){
    HAL_ADC_Start_DMA(&hadc, adc_vals, N_ADC_CHANNELS);
}
uint16_t ADC_GetRealTimeValue(uint8_t adcValueIdx){
    int16_t current;
    switch(adcValueIdx){
        case ADC_ARRAY_VOLTAGE_IDX:
            return (g_adcVals[ADC_ARRAY_VOLTAGE_IDX]>>ADC_NOT_LOOSE_PRECISION_SHIFT)*4400/1000;//4962
            break;
        case ADC_BATTERY_VOLTAGE_IDX:
            return (g_adcVals[ADC_BATTERY_VOLTAGE_IDX]>>ADC_NOT_LOOSE_PRECISION_SHIFT)*1888/1000;//1967
            break;
        case ADC_BATTERY_CURRENT_IDX:
            current = (((int16_t) ((g_adcVals[ADC_BATTERY_CURRENT_IDX]>>ADC_NOT_LOOSE_PRECISION_SHIFT) - 0x170)))*40/100;//0x160
            if (current < 0) {
                return 0;
            } else {
                return current;
            }
                //g_realTimeData[VAL_RTD_BATTERY_CURRENT].halfWord = (((int16_t) (g_adcVals[5] - 0x170)))*40/100;
            break;
    }
}
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
    int_fast8_t i;
    for(i=N_ADC_CHANNELS-1; i--; i>=0){
        g_adcVals[i] *= (N_FILTER_ORDER-1);
        g_adcVals[i] /= N_FILTER_ORDER;
        #if N_FILTER_ORDER == 4 //do nothing, shift by 2 and divide by 4, ADC_NOT_LOOSE_PRECISION_SHIFT must be 2
        #else
        adc_vals[i] <<= ADC_NOT_LOOSE_PRECISION_SHIFT; // do not loose precision
        adc_vals[i] /= N_FILTER_ORDER;
        #endif
        g_adcVals[i] += adc_vals[i];

    }
}
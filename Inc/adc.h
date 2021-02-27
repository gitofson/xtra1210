#ifndef __ADC_H
#define __ADC_H
#include "stm32f0xx_hal.h"
#define N_ADC_CHANNELS              (11)
#define N_FILTER_ORDER              (4) //important for filter divide by logical shift (for fast execution, value should be 2, 4 or 8)

#define ADC_ARRAY_VOLTAGE_IDX       (0)
#define ADC_BATTERY_VOLTAGE_IDX     (1)
#define ADC_BATTERY_CURRENT_IDX     (5)
extern ADC_HandleTypeDef hadc;
extern uint16_t g_adcVals[N_ADC_CHANNELS];
void ADC_Start();
#endif
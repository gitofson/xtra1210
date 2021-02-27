#ifndef __MPPT_H
#define __MPPT_H

#include "stm32f0xx_hal.h"
#define MPPT_PWM_MAX_VALUE                          (250-1)
#define MPPT_PWM_MIN_VALUE                          (MPPT_PWM_MAX_VALUE/2)
#define MPPT_START_MIN_BATTERY_VOLTAGE              (8)
#define MPPT_START_MIN_ARRAY_VOLTAGE                (15)

enum MPPT_Response{
    MPPT_OK = 0,
    MPPT_STOPPED,
};

enum MPPT_Response MPPT_Start();
void MPPT_SearchMax();
uint8_t MPPT_Adjust(int8_t);
uint16_t MPPT_GetPwm();

#endif
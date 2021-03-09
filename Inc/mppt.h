#ifndef __MPPT_H
#define __MPPT_H

#include "stm32f0xx_hal.h"
#define MPPT_PWM_MAX_VALUE                          (250-1)
#define MPPT_PWM_MIN_VALUE                          (MPPT_PWM_MAX_VALUE/2)
#define MPPT_START_MIN_BATTERY_VOLTAGE              (8)
#define MPPT_START_MIN_ARRAY_VOLTAGE                (15)
#define MPPT_MAX_CURENT_LIMIT_PERCENTAGE            (50)
#define MPPT_MAX_CHARGING_VOLTAGE_LIMIT_PERCENTAGE  (100)

enum MPPT_Response{
    MPPT_OK = 0,
    MPPT_STOPPED,
};

enum MPPT_Adjust_Response{
    MPPT_Adjust_OK = 0,
    MPPT_Adjust_Overvoltage,
    MPPT_Adjust_Overcurrent,
    MPPT_Adjust_None,
};

enum MPPT_Response MPPT_Start();
void MPPT_SearchMax();
enum MPPT_Adjust_Response MPPT_Adjust(int8_t);
uint16_t MPPT_GetPwm();
void MPPT_Increment(int16_t value);
#endif
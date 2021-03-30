#ifndef __MPPT_H
#define __MPPT_H

#include "stm32f0xx_hal.h"
#define MPPT_PWM_MAX_VALUE                          (250-1)
#define MPPT_PWM_MIN_VALUE                          (MPPT_PWM_MAX_VALUE/2)
#define MPPT_START_MIN_BATTERY_VOLTAGE              (800)
#define MPPT_START_MIN_ARRAY_VOLTAGE                (MPPT_START_MIN_BATTERY_VOLTAGE+200)
#define MPPT_MAX_CURENT_LIMIT_PERCENTAGE            (100)
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


typedef struct _mppt_handle_t {
    uint8_t isWorking;
    uint8_t isMpptSearchInitialRequest;
    uint8_t isMpptSearchInProgress;
    uint8_t isMpptManual;
    uint16_t pwm;
    uint32_t timer10ms;

} mppt_handle_t;

void MPPT_Init(mppt_handle_t*);
enum MPPT_Response MPPT_Start(mppt_handle_t*);
void MPPT_SearchMax(mppt_handle_t*);
enum MPPT_Adjust_Response MPPT_Adjust(mppt_handle_t*, int8_t);
void MPPT_Increment(mppt_handle_t*, int16_t value);
#endif
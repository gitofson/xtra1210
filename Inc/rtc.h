#ifndef __RTC_H
#define __RTC_H
#include "stm32f0xx_hal.h"

enum RTC_Event{
    RTC_EVENT_NOTHING=0,
    RTC_EVENT_NEW_DAY,
    RTC_EVENT_NEW_MONTH,
    RTC_EVENT_NEW_YEAR
};
extern enum RTC_Event g_rtcEvent;
HAL_StatusTypeDef rtcSynchroGetTime();
HAL_StatusTypeDef rtcSynchroSetTime();

#endif
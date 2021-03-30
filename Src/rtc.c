#include "rtc.h"
#include "values.h"

enum RTC_Event g_rtcEvent=RTC_EVENT_NOTHING;

HAL_StatusTypeDef rtcSynchroSetTime(){
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_StatusTypeDef out;
    sTime.Hours = g_settings[VAL_SET_RTC2].byte.lo;
    sTime.Minutes = g_settings[VAL_SET_RTC1].byte.hi;
    sTime.Seconds = g_settings[VAL_SET_RTC1].byte.lo;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_SUB1H;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (out = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN))
    {
        return out;
        //Error_Handler();
    }
    //sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    sDate.Month = g_settings[VAL_SET_RTC3].byte.lo;
    sDate.Date = g_settings[VAL_SET_RTC2].byte.hi;
    sDate.Year = g_settings[VAL_SET_RTC3].byte.hi;

    if (out=HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN))
    {
        return out;
        //Error_Handler();
    }
    return out;
}

HAL_StatusTypeDef rtcSynchroGetTime(){
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_StatusTypeDef out;
    if (out = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN))
    {
        return out;
        //Error_Handler();
    }
     g_settings[VAL_SET_RTC2].byte.lo = sTime.Hours;
     g_settings[VAL_SET_RTC1].byte.hi = sTime.Minutes;
     g_settings[VAL_SET_RTC1].byte.lo = sTime.Seconds;
    if (out=HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN))
    {
        return out;
        //Error_Handler();
    }
    //sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    g_settings[VAL_SET_RTC3].byte.lo = sDate.Month;
    g_settings[VAL_SET_RTC2].byte.hi = sDate.Date;
    g_settings[VAL_SET_RTC3].byte.hi = sDate.Year;


    return out;
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
    static uint8_t firstRun=0xff;
    static RTC_DateTypeDef lastDate;
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
  /*
    if (out = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD))
    {
        Error_Handler();
    }*/
    if (HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN))
    {
        Error_Handler();
    }
    if(firstRun) {
        firstRun = 0;
        g_rtcEvent=RTC_EVENT_NOTHING;
    } else {
        if(sDate.Year != lastDate.Year){
            g_rtcEvent = RTC_EVENT_NEW_YEAR;
        } else {
            if(sDate.Month != lastDate.Month){
                g_rtcEvent = RTC_EVENT_NEW_MONTH;
            } else {
                g_rtcEvent = RTC_EVENT_NEW_DAY;
            }

        }
        
    }
    lastDate=sDate;
}
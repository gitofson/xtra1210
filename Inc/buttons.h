#ifndef __BUTTONS_H
#define __BUTTONS_H
#include "stm32f0xx_hal.h"
typedef struct _butt {
    uint8_t eventButt14pressed;
    uint8_t eventButt15pressed;
    uint8_t isButt14pressed;
    uint8_t isButt15pressed;
} buttons_t;

extern buttons_t g_buttons;

#endif
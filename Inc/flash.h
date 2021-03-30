#ifndef __FLASH_H
#define __FLASH_H
//#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "values.h"

#define PAGE31_ADDRESS 0x8007C00

//={.buffer={0}} 
extern const uint16_t    f_statisticalParameters[32];
void archiveToFlash();
#endif
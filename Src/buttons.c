#include "buttons.h"
#include "cs1621.h"

buttons_t g_buttons;
void HAL_GPIO_EXTI_Callback( uint16_t gpioPin) {
    
    //stop the DCDC converter if Vbat>Vpv !!!
    if (gpioPin == GPIO_PIN_10) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        cs1621_showSymbol(CS1621_EXCLAMATION);
    }

    if (gpioPin == GPIO_PIN_14) {
        /* Do your stuff when PB14 is changed */
        g_buttons.eventButt14pressed=0xff;
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        
        /* Clear interrupt flag */
        //EXTI_ClearITPendingBit(EXTI_LINE_14);
    }
  if (gpioPin == GPIO_PIN_15) {
        /* Do your stuff when PB15 is changed */
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
        g_buttons.eventButt15pressed=0xff;
        /* Clear interrupt flag */
        //EXTI_ClearITPendingBit(EXTI_LINE_15);
    }
}
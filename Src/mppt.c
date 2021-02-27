#include "mppt.h"
#include "adc.h"
#include "values.h"
struct mppt_status_s{
  uint8_t     isWorking;
};
extern TIM_HandleTypeDef htim1;
static uint16_t pwm;
static struct mppt_status_s mppt_status={.isWorking=0};

void MPPT_Stop(){
  // stop DC/DC step down
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  //swtch off LEDG
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  mppt_status.isWorking=0;
}
enum MPPT_Response MPPT_Start(){
    
    //check Vbat(+) Vpv(-) comparator
    if(g_realTimeData[VAL_RTD_BATTERY_VOLTAGE].word < MPPT_START_MIN_BATTERY_VOLTAGE){
      MPPT_Stop();
      return MPPT_STOPPED;
    }
    if(g_realTimeData[VAL_RTD_ARRAY_VOLTAGE].word < MPPT_START_MIN_ARRAY_VOLTAGE){
      MPPT_Stop();
      return MPPT_STOPPED;
    }
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)){
      MPPT_Stop();
      return MPPT_STOPPED;
    } else {
      if(mppt_status.isWorking){
        return MPPT_OK;
      }
      //updateRealTimeValues();
      pwm = getLowPowerPWM();//(MPPT_PWM_MAX_VALUE * (g_realTimeData[VAL_RTD_BATTERY_VOLTAGE].word)) / (g_realTimeData[VAL_RTD_ARRAY_VOLTAGE].word);
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
      // start DC/DC step down
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        //swtch on LEDG
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      //set PWM on timer 1
      
      
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      mppt_status.isWorking=0xff;
      return MPPT_OK;
    }

}
void MPPT_SearchMax(){

}
uint8_t MPPT_Adjust(int8_t pwmStep){
  static uint32_t previous_power=0;
  uint32_t power;
  if(mppt_status.isWorking){
    pwm += pwmStep;
    if(pwm > MPPT_PWM_MAX_VALUE){
      pwm= MPPT_PWM_MAX_VALUE;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
    HAL_Delay(10);
    power=g_adcVals[ADC_BATTERY_VOLTAGE_IDX]*g_adcVals[ADC_BATTERY_VOLTAGE_IDX];
    if(power > previous_power){
      previous_power=power;
      return 0;
    } else {
      pwm -= pwmStep;
      if(pwm < MPPT_PWM_MIN_VALUE){
        pwm = MPPT_PWM_MIN_VALUE;
      }
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
    }
  }
  return 1;



}
uint16_t MPPT_GetPwm(){
  return pwm;
}

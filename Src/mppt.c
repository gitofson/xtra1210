#include "mppt.h"
#include "adc.h"
#include "values.h"
struct mppt_status_s{
  uint8_t     isWorking;
};
extern TIM_HandleTypeDef htim1;
static uint16_t pwm;
static uint16_t minPwmValue=MPPT_PWM_MIN_VALUE;
static struct mppt_status_s mppt_status={.isWorking=0};

int8_t maxCurrentCheck(uint8_t percentageLimit);

uint16_t getLowPowerPWM();

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
    if(g_realTimeData[VAL_RTD_BATTERY_VOLTAGE].halfWord < MPPT_START_MIN_BATTERY_VOLTAGE){
      MPPT_Stop();
      return MPPT_STOPPED;
    }
    if(g_realTimeData[VAL_RTD_ARRAY_VOLTAGE].halfWord < MPPT_START_MIN_ARRAY_VOLTAGE){
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
      minPwmValue = pwm = getLowPowerPWM();//(MPPT_PWM_MAX_VALUE * (g_realTimeData[VAL_RTD_BATTERY_VOLTAGE].halfWord)) / (g_realTimeData[VAL_RTD_ARRAY_VOLTAGE].halfWord);
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
  uint32_t power, maxPower=0;
  uint16_t optimalPwm;

  if(mppt_status.isWorking){
//swtch on backlight
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);    
    optimalPwm=minPwmValue=pwm=getLowPowerPWM();
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
    while(++pwm <= MPPT_PWM_MAX_VALUE){
      HAL_Delay(40);
      power=g_adcVals[ADC_BATTERY_VOLTAGE_IDX]*g_adcVals[ADC_BATTERY_CURRENT_IDX];
      if(power>maxPower){
        maxPower=power;
        optimalPwm=pwm;
      }/*
      if(maxCurrentCheck(MPPT_MAX_CURENT_LIMIT_PERCENTAGE)){
        optimalPwm = pwm;
        break;
      }*/
    }
    pwm=optimalPwm;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
//swtch off backlight
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);    
  }
}
uint8_t MPPT_Adjust(int8_t pwmStep){
  static int32_t previous_power=0;
  int32_t power;
  if(mppt_status.isWorking){
    pwm += pwmStep;
    if(pwm > MPPT_PWM_MAX_VALUE){
      pwm= MPPT_PWM_MAX_VALUE;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
    HAL_Delay(50);
    power=g_adcVals[ADC_BATTERY_VOLTAGE_IDX]*g_adcVals[ADC_BATTERY_CURRENT_IDX];
    //power=ADC_GetRealTimeValue(ADC_BATTERY_VOLTAGE_IDX)*ADC_GetRealTimeValue(ADC_BATTERY_CURRENT_IDX);
    if(power > previous_power && (maxCurrentCheck(MPPT_MAX_CURENT_LIMIT_PERCENTAGE) == 0)){
      previous_power=power;
      return 0;
    } else {
      pwm -= pwmStep;
      if(pwm < minPwmValue){
        pwm = minPwmValue;
      }
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
    }
  }
  return 1;
}
uint16_t MPPT_GetPwm(){
  return pwm;
}

void MPPT_Increment(int16_t value){
  uint16_t tmp;
  tmp=pwm+value;
  if(tmp >= minPwmValue && tmp <= MPPT_PWM_MAX_VALUE){
    pwm = tmp;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
  }
}

//percentage limit 100 <=> full max current, 50 <=> half max current
int8_t maxCurrentCheck(uint8_t percentageLimit){
  uint16_t maxCurrent=g_ratedData[VAL_RTD_BATTERY_CURRENT].halfWord;
  if(100 * ADC_GetRealTimeValue(ADC_BATTERY_CURRENT_IDX) > percentageLimit * g_ratedData[VAL_RTD_BATTERY_CURRENT].halfWord)
  {
    return -1;
  } else {
    return 0;
  }
}

uint16_t getLowPowerPWM(){
    uint16_t ret;
    ret = (MPPT_PWM_MAX_VALUE*ADC_GetRealTimeValue(ADC_BATTERY_VOLTAGE_IDX)) / ADC_GetRealTimeValue(ADC_ARRAY_VOLTAGE_IDX);
    if(ret>MPPT_PWM_MIN_VALUE){
        return ret;
    } else {
        return MPPT_PWM_MIN_VALUE;
    }
}
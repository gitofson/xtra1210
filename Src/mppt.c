#include "mppt.h"
#include "adc.h"
#include "values.h"

extern TIM_HandleTypeDef htim1;
static uint16_t minPwmValue=MPPT_PWM_MIN_VALUE;
//static struct mppt_status_s mppt_status={.isWorking=0};

int8_t maxCurrentCheck(uint8_t percentageLimit);
int8_t maxChargingVoltageCheck(uint8_t percentageLimit);

uint16_t getLowPowerPWM();

void MPPT_Stop(mppt_handle_t *hmppt){
  // stop DC/DC step down
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  //swtch off LEDG
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  hmppt->isWorking=0;
}
void MPPT_Init(mppt_handle_t *mppt){
  mppt->isMpptManual=0;
  mppt->isMpptSearchInitialRequest=0;
  mppt->isMpptSearchInProgress=0;
  mppt->isWorking=0;
  mppt->pwm=MPPT_PWM_MIN_VALUE;
  mppt->timer10ms=0;
}
enum MPPT_Response MPPT_Start(mppt_handle_t *hmppt){
    
    //check Vbat(+) Vpv(-) comparator
    if(g_realTimeData.par.batteryVoltage < MPPT_START_MIN_BATTERY_VOLTAGE){
      MPPT_Stop(hmppt);
      return MPPT_STOPPED;
    }
    if(g_realTimeData.par.pvArrayVoltage < MPPT_START_MIN_ARRAY_VOLTAGE){
      MPPT_Stop(hmppt);
      return MPPT_STOPPED;
    }
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)){
      MPPT_Stop(hmppt);
      return MPPT_STOPPED;
    } else {
      if(hmppt->isWorking){
        return MPPT_OK;
      }
      //updateRealTimeValues();
      minPwmValue = hmppt->pwm = getLowPowerPWM();//(MPPT_PWM_MAX_VALUE * (g_realTimeData[VAL_RTD_BATTERY_VOLTAGE].halfWord)) / (g_realTimeData[VAL_RTD_ARRAY_VOLTAGE].halfWord);
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, hmppt->pwm);
      // start DC/DC step down
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        //swtch on LEDG
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      //set PWM on timer 1
      
      
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      hmppt->isWorking=0xff;
      return MPPT_OK;
    }

}
void MPPT_SearchMax(mppt_handle_t *hmppt){
  static uint32_t maxPower;
  static uint16_t optimalPwm;
  static uint32_t nextPoint10msTimer;
  uint32_t delta;
  //uint32_t tmp;
  uint32_t power;

  if(hmppt->isWorking){
    if(hmppt->isMpptSearchInitialRequest){
      hmppt->isMpptSearchInitialRequest = 0;
      maxPower = 0;
      hmppt->isMpptSearchInProgress=0xff;
      optimalPwm=minPwmValue = hmppt->pwm = getLowPowerPWM();
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, hmppt->pwm);
      nextPoint10msTimer=hmppt->timer10ms+4;
      //swtch on backlight
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); 
      return;
    }
    if(hmppt->isMpptSearchInProgress){
      delta = nextPoint10msTimer - hmppt->timer10ms;
      // assert overflow case, see https://luckyresistor.me/2019/07/10/real-time-counter-and-integer-overflow/
      if((delta & 0x80000000u) != 0 || delta == 0){
   

        if(++hmppt->pwm <= MPPT_PWM_MAX_VALUE){
          //HAL_Delay(40);
          power=g_adcVals[ADC_BATTERY_VOLTAGE_IDX]*g_adcVals[ADC_BATTERY_CURRENT_IDX];
          if(power>maxPower){
            maxPower = power;
            optimalPwm = hmppt->pwm;
          }
          if(maxCurrentCheck(MPPT_MAX_CURENT_LIMIT_PERCENTAGE)){
            optimalPwm = hmppt->pwm-1;
            goto L_MPPT_SearchMax001;
          }
          if(maxChargingVoltageCheck(MPPT_MAX_CHARGING_VOLTAGE_LIMIT_PERCENTAGE)){
            optimalPwm = hmppt->pwm-1;
            goto L_MPPT_SearchMax001;
          }
          if(optimalPwm < minPwmValue){
            optimalPwm = minPwmValue;
          }
          nextPoint10msTimer=hmppt->timer10ms+4;
        } else {
    L_MPPT_SearchMax001:   
          hmppt->isMpptSearchInProgress = 0;
          hmppt->pwm = optimalPwm;
          //swtch off backlight
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, hmppt->pwm);
        
        
      }
    }
  }
}
enum MPPT_Adjust_Response MPPT_Adjust(mppt_handle_t *hmppt, int8_t pwmStep){
  static int32_t previous_power=0;
  int32_t power;
  enum MPPT_Adjust_Response response=MPPT_Adjust_None;
  if(hmppt->isWorking){
    if(hmppt->pwm == minPwmValue){
      previous_power = 0;
    }
    hmppt->pwm += pwmStep;
    if(hmppt->pwm > MPPT_PWM_MAX_VALUE){
      hmppt->pwm= MPPT_PWM_MAX_VALUE;
    }
    if(hmppt->pwm <= minPwmValue){
      hmppt->pwm = minPwmValue;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, hmppt->pwm);
    HAL_Delay(50);
    power=ADC_GetRealTimeValue(ADC_BATTERY_VOLTAGE_IDX)*ADC_GetRealTimeValue(ADC_BATTERY_CURRENT_IDX);
    if(maxCurrentCheck(MPPT_MAX_CURENT_LIMIT_PERCENTAGE)){
      response=MPPT_Adjust_Overcurrent;
    }
    if(maxChargingVoltageCheck(MPPT_MAX_CHARGING_VOLTAGE_LIMIT_PERCENTAGE)){
      response=MPPT_Adjust_Overvoltage;
    }
    if(power >= previous_power && response == MPPT_Adjust_None){
      previous_power=power;
      return MPPT_Adjust_OK;
    } else {
      if((pwmStep < 0 && response==MPPT_Adjust_None) || (pwmStep > 0)){
        hmppt->pwm -= pwmStep;
        if(hmppt->pwm < minPwmValue){
          //previous_power = 0;
          hmppt->pwm = minPwmValue;
        }
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, hmppt->pwm);
      } else {
        
      }
    }
  }
  return response;
}

void MPPT_Increment(mppt_handle_t *hmppt, int16_t value){
  uint16_t tmp;
  tmp=hmppt->pwm+value;
  if(tmp >= minPwmValue && tmp <= MPPT_PWM_MAX_VALUE){
    hmppt->pwm = tmp;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, hmppt->pwm);
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

//percentage limit 100 <=> full max current, 50 <=> half max current
int8_t maxChargingVoltageCheck(uint8_t percentageLimit){
  uint16_t maxChargingVoltage=g_settings[VAL_SET_CHARGING_LIMIT_VOLTAGE].halfWord;
  if(100 * ADC_GetRealTimeValue(ADC_BATTERY_VOLTAGE_IDX) > percentageLimit * g_settings[VAL_SET_CHARGING_LIMIT_VOLTAGE].halfWord)
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
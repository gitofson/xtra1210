#include "main.h"
#include "values.h"
#include "adc.h"
#include "rtc.h"

t_request g_request;
//extern uint16_t g_adcVals[N_ADC_CHANNELS];
halfWord_t    g_ratedData[]={
    {.halfWord=10000},
    {.halfWord=1000},
    {.halfWord=25000 & 0xffff},  //PV array power LO halfword
    {.halfWord=(25000>>16)},     //PV array power HI halfword
    {.halfWord=2400},
    {.halfWord=1000},
    {.halfWord=25000 & 0xffff},    //battery power LO halfword
    {.halfWord=24000>>16},       //battery power HI halfword
    {.halfWord=0},
    {.halfWord=0},
    {.halfWord=0},
    {.halfWord=0},
    {.halfWord=0},
    {.halfWord=0},
    {.halfWord=0},
    {.halfWord=0}
};
realTimeData_t      g_realTimeData={.buffer={0}};
realTimeStatus_t    g_realTimeStatus={.buffer={0}};
statisticalParameters_t    g_statisticalParameters;
halfWord_t      g_settings[32]={
    {.halfWord=0x00},//        uint16_t    batteryType =  buffer[ 0x00 ]; //0001H- Sealed , 0002H- GEL, 0003H- Flooded, 0000H- User defined
    {.halfWord=86},//uint16_t    batteryCapacity =  buffer[ 0x01 ]; //[Ah]
    {.halfWord=0},//float   tempCompensationCoeff   = ((float) buffer[ 0x02 ]) / 100.0;
    {.halfWord=3300},//float   highVoltageDisconnect   = ((float) buffer[ 0x03 ]) / 100.0;
    {.halfWord=2840},//2840 float   chargingLimitVoltage    = ((float) buffer[ 0x04 ]) / 100.0;
    {.halfWord=2840},//float   overVoltageReconnect    = ((float) buffer[ 0x05 ]) / 100.0;
    {.halfWord=2840},//float   equalizationVoltage     = ((float) buffer[ 0x06 ]) / 100.0;
    {.halfWord=2840},//float   boostVoltage            = ((float) buffer[ 0x07 ]) / 100.0;
    {.halfWord=2840},//float   floatVoltage            = ((float) buffer[ 0x08 ]) / 100.0;
    {.halfWord=2830},//float   boostReconnectVoltage   = ((float) buffer[ 0x09 ]) / 100.0;

    //  Our LS1024B controller doesn't seem to support any register data above 0x0A
    {.halfWord=2640},//float   lowVoltageReconnect     = ((float) buffer[ 0x0A ]) / 100.0;
    {.halfWord=2440},//float   underVoltageRecover     = ((float) buffer[ 0x0B ]) / 100.0;
    {.halfWord=2400},//float   underVoltageWarning     = ((float) buffer[ 0x0C ]) / 100.0;
    {.halfWord=2470},//float   lowVoltageDisconnect    = ((float) buffer[ 0x0D ]) / 100.0;
    {.halfWord=2120},//float   dischargingLimitVoltage = ((float) buffer[ 0x0E ]) / 100.0;
    {.halfWord=0},{.halfWord=0},{.halfWord=0},{.halfWord=0},
    {.byte.lo=0, .byte.hi=30}, // uint16_t    realTimeClock1      = buffer[ 0x13 ];
    {.byte.lo=19, .byte.hi=30}, //uint16_t    realTimeClock2      = buffer[ 0x14 ];
    {.byte.lo=3, .byte.hi=21}, //uint16_t    realTimeClock3      = buffer[ 0x15 ];

};

uint8_t altSpecialHeader[]={0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x0f, 0xff, 0xff};
halfWord_t crc16(uint8_t  *data, uint8_t length)
{ 
    uint8_t i, j;
    halfWord_t out;
    out.halfWord = 0xFFFF;
    for (i = 0; i < length; i++)
    {
        out.halfWord ^=data[i];
        for (j = 8; j !=0; j--)
        {
            if ((out.halfWord & 0x0001) !=0)
            {
                out.halfWord >>= 1;
                out.halfWord ^= 0xA001;
            }
            else
            out.halfWord >>= 1;
        }		
    }
    return out;
}

/* Read the data from a modbus slave and put that data into an array */
/*
static int read_registers(int slave, int function,
			  int start_addr, int count, int *data_dest)
{
	int query_size;
	int status;
	int query_ret;
	unsigned char query[MIN_QUERY_SIZE];

	query_size = build_request_packet(slave, function, 
					  start_addr, count, query);

	query_ret = modbus_query(mb_param, query, query_size);
	if (query_ret > 0)
		status = read_reg_response(mb_param, data_dest, query);
	else
		status = query_ret;
	
	return status;
}*/
void valuesInit(){
    memcpy(g_statisticalParameters.buffer, f_statisticalParameters, sizeof(statisticalParameters_t));
}
void worldToByteCp(uint8_t * buff, halfWord_t * w,  uint8_t nWords){
    for (int i=0; i<nWords; i++){
        buff[2*i]=w[i].byte.hi;
        buff[2*i+1]=w[i].byte.lo;
    }
}
//do not use thos callback. Lets Tx of sender be done first.
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    g_readyToSend = 0xff;
}*/
void processMessage(UART_HandleTypeDef *huart)
{
    //uint8_t bufferToBeReceivedIdx = 0;
    //uint8_t n_restBytes = RECEIVED_DATA_MIN_LENGTH;
    uint8_t idx = 3;
    uint8_t length;
    halfWord_t crc;
    //__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    if(g_request.req.slave_id == DEFAULT_SLAVE_ID){
        crc = crc16(g_request.buff, 6);
        if(crc.halfWord == g_request.req.crc.halfWord) {
            length =  2*g_request.req.rest.data.length.byte.hi;
            switch(g_request.req.modbus_function_code){
                case MODBUS_FC_READ_INPUT_REGISTERS_ALT:
                case MODBUS_FC_READ_INPUT_REGISTERS:
                    switch(g_request.req.rest.data.address.byte.lo ){
                        case 0x30:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            if(length > sizeof(g_ratedData)){
                                length = sizeof(g_ratedData);
                            }
                            g_tx_buff[2] = length;
                            worldToByteCp(g_tx_buff+3, g_ratedData + g_request.req.rest.data.address.byte.hi, length/2);
                            break;
                        case 0x31:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            /*
                            if(length > sizeof(g_realTimeData)){
                                length = sizeof(g_realTimeData);
                            }*/
                            g_tx_buff[2] = length;
                            if(g_request.req.modbus_function_code == MODBUS_FC_READ_INPUT_REGISTERS_ALT){
                                memcpy(g_tx_buff+idx, altSpecialHeader, sizeof(altSpecialHeader) );
                                idx+=sizeof(altSpecialHeader);
                                length=sizeof(g_realTimeData);
                            }
                            worldToByteCp(g_tx_buff+idx, g_realTimeData.buffer + g_request.req.rest.data.address.byte.hi, length/2);
                            break;
                         case 0x32:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            /*
                            if(length > sizeof(g_realTimeStatus)){
                                length = sizeof(g_realTimeStatus);
                            }*/
                            g_tx_buff[2] = length;
                            if(g_request.req.modbus_function_code == MODBUS_FC_READ_INPUT_REGISTERS_ALT){
                                g_tx_buff[idx++]=0x03;
                            }
                            worldToByteCp(g_tx_buff+idx, g_realTimeStatus.buffer + g_request.req.rest.data.address.byte.hi, length/2);
                            break;
                        case 0x33:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            /*
                            if(length > sizeof(g_statisticalParameters)){
                                length = sizeof(g_statisticalParameters);
                            }*/
                            g_tx_buff[2] = length;
                            if(g_request.req.modbus_function_code == MODBUS_FC_READ_INPUT_REGISTERS_ALT){
                                memcpy(g_tx_buff+idx, altSpecialHeader, sizeof(altSpecialHeader) );
                                idx+=sizeof(altSpecialHeader);
                                length = sizeof(g_statisticalParameters);
                            }
                            worldToByteCp(g_tx_buff+idx, g_statisticalParameters.buffer + g_request.req.rest.data.address.byte.hi, length/2);
                            break;                        
                    }
                    break;
                case MODBUS_FC_READ_HOLDING_REGISTERS:
                    if(g_request.req.rest.data.address.byte.lo == 0x90){
                        if(g_request.req.rest.data.address.byte.hi + g_request.req.rest.data.length.halfWord > VAL_SET_RTC1){
                            rtcSynchroGetTime();
                        }
                        memcpy(g_tx_buff, g_request.buff, 2);
                        g_tx_buff[2] = length;
                        worldToByteCp(g_tx_buff+3, g_settings + g_request.req.rest.data.address.byte.hi, length/2);
                        break;
                    }
                case MODBUS_FC_READ_DEVICE_INFO:
                    memcpy(g_tx_buff, g_request.buff, 5);
                    g_tx_buff[5]=0;
                    g_tx_buff[6]=g_request.req.rest.info.code_object;
                    g_tx_buff[7]=1;
                    g_tx_buff[8]=g_request.req.rest.info.code_object;
                    switch(g_request.req.rest.info.code_object){
                        case 0:
                            g_tx_buff[9]=sizeof(INFO_VENDOR_NAME)-1;
                            memcpy(g_tx_buff+10, INFO_VENDOR_NAME, sizeof(INFO_VENDOR_NAME));
                            length=10-3+sizeof(INFO_VENDOR_NAME)-1;
                            break;
                        case 1:
                            g_tx_buff[9]=sizeof(INFO_PRODUCT_CODE)-1;
                            memcpy(g_tx_buff+10, INFO_PRODUCT_CODE, sizeof(INFO_PRODUCT_CODE));
                            length=10-3+sizeof(INFO_PRODUCT_CODE)-1;
                            break;
                        case 2:
                            g_tx_buff[9]=sizeof(INFO_VERSION)-1;
                            memcpy(g_tx_buff+10, INFO_VERSION, sizeof(INFO_VERSION));
                            length=10-3+sizeof(INFO_VERSION)-1;
                            break;
                        default:
                            length=10-3;
                            break;
                    }
            }
            crc = crc16(g_tx_buff, length+idx);
            g_tx_buff[length+idx] = crc.byte.lo;
            g_tx_buff[length+idx+1] = crc.byte.hi;
            g_tx_buff_length=length+idx+2;
            g_readyToSend=0xff;
        }
    }
}

void updateRealTimeValues(mppt_handle_t *hmppt){
    static uint64_t energy=0; //[0.1mW * s]
    uint32_t tmp;
    g_realTimeData.par.pvArrayVoltage = ADC_GetRealTimeValue(ADC_ARRAY_VOLTAGE_IDX);
    g_realTimeData.par.batteryVoltage = ADC_GetRealTimeValue(ADC_BATTERY_VOLTAGE_IDX);
    g_realTimeData.par.batteryCurrent = ADC_GetRealTimeValue(ADC_BATTERY_CURRENT_IDX);
    g_realTimeData.par.pvArrayCurrent = g_realTimeData.par.batteryCurrent*hmppt->pwm/MPPT_PWM_MAX_VALUE;
    g_realTimeData.par.pvArrayPower = g_realTimeData.par.pvArrayVoltage* g_realTimeData.par.pvArrayCurrent/100;
    tmp = g_realTimeData.par.batteryVoltage*g_realTimeData.par.batteryCurrent;//[100uW]
    g_realTimeData.par.batteryPower = tmp/100;//[10mW]
    g_realTimeData.par.batterySOC = hmppt->pwm;
    
    energy += tmp;
    
    g_statisticalParameters.par.generatedEnergyToday = energy/360000000 ; //[10Wh]
    if(g_rtcEvent){
        g_statisticalParameters.par.totalGeneratedEnergy += g_statisticalParameters.par.generatedEnergyToday;
        g_statisticalParameters.par.generatedEnergyYear  += g_statisticalParameters.par.generatedEnergyToday;
        g_statisticalParameters.par.generatedEnergyMonth += g_statisticalParameters.par.generatedEnergyToday;
        switch(g_rtcEvent){
            case RTC_EVENT_NEW_YEAR:
                g_statisticalParameters.par.generatedEnergyYear=0;
            case RTC_EVENT_NEW_MONTH:
                g_statisticalParameters.par.generatedEnergyMonth=0;
            case RTC_EVENT_NEW_DAY:
                g_statisticalParameters.par.generatedEnergyToday=0;
                energy=0;
            default:
                
            break;
        }
        g_rtcEvent=0;
        archiveToFlash();
    }
}

//HAL_USART_RxCpltCallback(huart){
HAL_UARTEx_RxEventCallback() {
   if(g_request.req.slave_id == DEFAULT_SLAVE_ID){
      g_isDataReceived = 0xff;
    }
    if(huart1.hdmarx->Instance->CNDTR){
        g_isUartRestartRequired=0xff;
    }
    //HAL_UARTEx_ReceiveToIdle_DMA(huart, g_request.buff, RECEIVED_DATA_MAX_LENGTH);
}


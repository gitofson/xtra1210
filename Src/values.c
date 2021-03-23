#include "main.h"
#include "values.h"
#include "adc.h"
#include "mppt.h"
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
uint16_t    g_realTimeStatus[2]={0};
statisticalParameters_t    g_statisticalParameters={.buffer={0}};
halfWord_t      g_settings[32]={
    {.halfWord=0x00},//        uint16_t    batteryType =  buffer[ 0x00 ]; //0001H- Sealed , 0002H- GEL, 0003H- Flooded, 0000H- User defined
    {.halfWord=80},//uint16_t    batteryCapacity =  buffer[ 0x01 ]; //[Ah]
    {.halfWord=0},//float   tempCompensationCoeff   = ((float) buffer[ 0x02 ]) / 100.0;
    {.halfWord=3300},//float   highVoltageDisconnect   = ((float) buffer[ 0x03 ]) / 100.0;
    {.halfWord=2750},//2840 float   chargingLimitVoltage    = ((float) buffer[ 0x04 ]) / 100.0;
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
    {.byte.lo=0, .byte.hi=0}, // uint16_t    realTimeClock1      = buffer[ 0x13 ];
    {.byte.lo=0, .byte.hi=23}, //uint16_t    realTimeClock2      = buffer[ 0x14 ];
    {.byte.lo=2, .byte.hi=21}, //uint16_t    realTimeClock3      = buffer[ 0x15 ];

};

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
    uint8_t length;
    halfWord_t crc;
    //__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    if(g_request.req.slave_id == DEFAULT_SLAVE_ID){
        crc = crc16(g_request.buff, 6);
        if(crc.halfWord == g_request.req.crc.halfWord) {
            length =  2*g_request.req.rest.data.length.byte.hi;
            switch(g_request.req.modbus_function_code){
                case MODBUS_FC_READ_INPUT_REGISTERS:
                case MODBUS_FC_READ_INPUT_REGISTERS_ALT:
                    switch(g_request.req.rest.data.address.byte.lo ){
                        case 0x30:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            g_tx_buff[2] = length;
                            worldToByteCp(g_tx_buff+3, g_ratedData + g_request.req.rest.data.address.byte.hi, length/2);
                            break;
                        case 0x31:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            g_tx_buff[2] = length;
                            worldToByteCp(g_tx_buff+3, g_realTimeData.buffer + g_request.req.rest.data.address.byte.hi, length/2);
                            break;
                        case 0x33:
                            memcpy(g_tx_buff, g_request.buff, 2);
                            g_tx_buff[2] = length;
                            worldToByteCp(g_tx_buff+3, g_statisticalParameters.buffer + g_request.req.rest.data.address.byte.hi, length/2);
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
            crc = crc16(g_tx_buff, length+3);
            g_tx_buff[length+3] = crc.byte.lo;
            g_tx_buff[length+4] = crc.byte.hi;
            g_tx_buff_length=length+5;
            g_readyToSend=0xff;
            //while(!g_uart_free){

            //}
            //while(!g_uart_free);
        }
    }
    //HAL_UART_Receive_DMA(huart, g_request.buff+bufferToBeReceivedIdx, n_restBytes);
    //HAL_UART_Receive_DMA(huart, g_request.buff, RECEIVED_DATA_MAX_LENGTH);
}

void updateRealTimeValues(){
    static uint64_t energy=0; //[0.1mW * s]
    uint32_t tmp;
    g_realTimeData.par.pvArrayVoltage = ADC_GetRealTimeValue(ADC_ARRAY_VOLTAGE_IDX);
    g_realTimeData.par.batteryVoltage = ADC_GetRealTimeValue(ADC_BATTERY_VOLTAGE_IDX);
    g_realTimeData.par.batteryCurrent = ADC_GetRealTimeValue(ADC_BATTERY_CURRENT_IDX);
    tmp=g_realTimeData.par.batteryVoltage*g_realTimeData.par.batteryCurrent;//[100uW]
    g_realTimeData.par.batteryPower = tmp/100;//[10mW]
    
    energy += tmp;
    
    g_statisticalParameters.par.generatedEnergyToday = energy/360000000 ; //[10Wh]
    switch(g_rtcEvent){
        case RTC_EVENT_NEW_YEAR:
            g_statisticalParameters.par.generatedEnergyYear=0;
        case RTC_EVENT_NEW_MONTH:
            g_statisticalParameters.par.generatedEnergyMonth=0;
        case RTC_EVENT_NEW_DAY:
            g_statisticalParameters.par.generatedEnergyToday=0;
        default:
            energy=0;
        break;
    }
    g_rtcEvent=0;
    //g_statisticalParameters[VAL_STAT_GENERATED_ENERGY_TODAY_LO] = tmp &=0xffff;
    //g_statisticalParameters[VAL_STAT_GENERATED_ENERGY_TODAY_HI] = tmp >> 16;
}

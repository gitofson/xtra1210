#include "main.h"
#include "values.h"

t_word    g_ratedData[]={
    {.word=10000}, {.word=1000}, {.word=100000|0xffff}, {.word=(uint16_t)(100000>>16)},
    {.word=2400}, {.word=1000}, {.word=24000|0xffff}, {.word=24000>>16},  
    0,0,0,0,
    0,0,0,0
};
t_word      g_realTimeData[32]={0};
uint16_t    g_realTimeStatus[2]={0};
uint16_t    g_statisticalParameters[32]={0};
t_word      g_settings[32]={
    {.word=0x00},//        uint16_t    batteryType =  buffer[ 0x00 ]; //0001H- Sealed , 0002H- GEL, 0003H- Flooded, 0000H- User defined
    {.word=80},//uint16_t    batteryCapacity =  buffer[ 0x01 ]; //[Ah]
    {.word=0},//float   tempCompensationCoeff   = ((float) buffer[ 0x02 ]) / 100.0;
    {.word=3300},//float   highVoltageDisconnect   = ((float) buffer[ 0x03 ]) / 100.0;
    {.word=2840},//float   chargingLimitVoltage    = ((float) buffer[ 0x04 ]) / 100.0;
    {.word=2840},//float   overVoltageReconnect    = ((float) buffer[ 0x05 ]) / 100.0;
    {.word=2840},//float   equalizationVoltage     = ((float) buffer[ 0x06 ]) / 100.0;
    {.word=2840},//float   boostVoltage            = ((float) buffer[ 0x07 ]) / 100.0;
    {.word=2840},//float   floatVoltage            = ((float) buffer[ 0x08 ]) / 100.0;
    {.word=2830},//float   boostReconnectVoltage   = ((float) buffer[ 0x09 ]) / 100.0;

    //  Our LS1024B controller doesn't seem to support any register data above 0x0A
    {.word=2640},//float   lowVoltageReconnect     = ((float) buffer[ 0x0A ]) / 100.0;
    {.word=2440},//float   underVoltageRecover     = ((float) buffer[ 0x0B ]) / 100.0;
    {.word=2400},//float   underVoltageWarning     = ((float) buffer[ 0x0C ]) / 100.0;
    {.word=2470},//float   lowVoltageDisconnect    = ((float) buffer[ 0x0D ]) / 100.0;
    {.word=2120},//float   dischargingLimitVoltage = ((float) buffer[ 0x0E ]) / 100.0;
    {.word=0},{.word=0},{.word=0},{.word=0},
    {.byte.lo=0, .byte.hi=0}, // uint16_t    realTimeClock1      = buffer[ 0x13 ];
    {.byte.lo=0, .byte.hi=23}, //uint16_t    realTimeClock2      = buffer[ 0x14 ];
    {.byte.lo=2, .byte.hi=21}, //uint16_t    realTimeClock3      = buffer[ 0x15 ];

};

t_word crc16(uint8_t  *data, uint8_t length)
{ 
    uint8_t i, j;
    t_word out;
    out.word = 0xFFFF;
    for (i = 0; i < length; i++)
    {
        out.word ^=data[i];
        for (j = 8; j !=0; j--)
        {
            if ((out.word & 0x0001) !=0)
            {
                out.word >>= 1;
                out.word ^= 0xA001;
            }
            else
            out.word >>= 1;
        }		
    }
    return out;
}

HAL_StatusTypeDef rtcSynchroSetTime(){
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_StatusTypeDef out;
    sTime.Hours = g_settings[VAL_SET_RTC2].byte.lo;
    sTime.Minutes = g_settings[VAL_SET_RTC1].byte.hi;
    sTime.Seconds = g_settings[VAL_SET_RTC1].byte.lo;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_SUB1H;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (out = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD))
    {
        return out;
        //Error_Handler();
    }
    //sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    sDate.Month = g_settings[VAL_SET_RTC3].byte.lo;
    sDate.Date = g_settings[VAL_SET_RTC2].byte.hi;
    sDate.Year = g_settings[VAL_SET_RTC3].byte.hi;

    if (out=HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD))
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
    if (out = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD))
    {
        return out;
        //Error_Handler();
    }
     g_settings[VAL_SET_RTC2].byte.lo = sTime.Hours;
     g_settings[VAL_SET_RTC1].byte.hi = sTime.Minutes;
     g_settings[VAL_SET_RTC1].byte.lo = sTime.Seconds;
    if (out=HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD))
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
void worldToByteCp(uint8_t * buff, t_word * w,  uint8_t nWords){
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
    uint8_t bufferToBeReceivedIdx = 0;
    uint8_t n_restBytes = RECEIVED_DATA_MIN_LENGTH;
    uint8_t length;
    t_word crc;
    //__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    if(g_request.req.slave_id == DEFAULT_SLAVE_ID){
        crc = crc16(g_request.buff, 6);
        if(crc.word == g_request.req.crc.word) {
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
                            worldToByteCp(g_tx_buff+3, g_realTimeData + g_request.req.rest.data.address.byte.hi, length/2);
                            break;
                    }
                    break;
                case MODBUS_FC_READ_HOLDING_REGISTERS:
                    if(g_request.req.rest.data.address.byte.lo == 0x90){
                        if(g_request.req.rest.data.address.byte.hi + g_request.req.rest.data.length.word > VAL_SET_RTC1){
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
    HAL_UART_Receive_DMA(huart, g_request.buff+bufferToBeReceivedIdx, n_restBytes);
}
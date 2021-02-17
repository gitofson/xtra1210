#include "main.h"
#include "values.h"

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t length;
    t_word crc;
    crc = crc16(g_request.buff, 6);
    if(crc.word == g_request.req.crc.word) {
        length =  2*g_request.req.rest.data.length.byte.hi;
        switch(g_request.req.rest.data.address.byte.lo ){
            case 0x31:
                memcpy(g_tx_buff, g_request.buff, 2);
                g_tx_buff[2] = length;
                worldToByteCp(g_tx_buff+3, g_realTimeData ,length/2);
                break;
        }
        crc = crc16(g_tx_buff, length+3);
        g_tx_buff[length+3] = crc.byte.lo;
        g_tx_buff[length+4] = crc.byte.hi;
    }
    //while(!g_uart_free){
    g_tx_buff_length=length+5;
    g_readyToSend=0xff;
    //}
    //while(!g_uart_free);
    HAL_UART_Receive_DMA(&huart1, g_request.buff, sizeof(g_request.buff));
}
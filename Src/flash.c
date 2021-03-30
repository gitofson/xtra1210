#include "flash.h"
const uint16_t    f_statisticalParameters[32] __attribute__((section("ARM.__at_0x08007C00"))) = 
    {
        0,0,0,0,0,0,0,0,
        0,0,0,0,314,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0};//page31
//volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
void archiveToFlash(){
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint16_t i;
    uint32_t SECTORError = 0;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);	
    //FLASHStatus = 
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = PAGE31_ADDRESS; // start address
    EraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
    for(i=0; i<sizeof(g_statisticalParameters); i++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE31_ADDRESS+i,g_statisticalParameters.buffer[i]);
    }
    HAL_FLASH_Lock();
}
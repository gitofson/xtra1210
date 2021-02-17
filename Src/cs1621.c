#include "main.h"
#include "cs1621.h"
static void SPI1_Reinit(uint32_t spiDatasize);
uint8_t get_digit(uint8_t val);
void send13bitVal(uint16_t buff);
void send8bitVal(uint16_t addr, uint8_t data);

  static uint16_t cs1621_config[] = {
    0x1004, //turn on oscillator
    0x100C,//LCD on
    0x10A0 //BIAS 1/2 for com3
  };
  static uint16_t cs1621_data[] = {
    0x140F, // N, halfsun, moon, PV panel
    0x141F, //PV, emptyBat, right >, left >, 
    0x142F, //N, bar3, bar2, bar1 (bottom <=> 8)
    0x143F,
    0x144F, //kWh%
    0x145F, //C, F
    0x146F, //_ _ C op
    0x147F, // _ _ 1-.
    0x148F, // _  Cop
    0x149F, // _ 1- -- only 3 segs
    0x14AF, // Cop
    0x14BF, // -1-
    0x14CF, //TYPE TYPE LOAD
    0x14DF //PV SOC    
  };

void cs1621_init(){
    uint8_t i;
    for (i=0; i < 3/* sizeof(cs1621_config)/sizeof(uint16_t)*/; i++){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_SPI_Transmit( &hspi1, cs1621_config+i, 1,  100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    }
}
void cs1621_demo(uint8_t cnt){
    uint8_t i;
    uint16_t a;
    for (i=0; i<sizeof(cs1621_data)/sizeof(uint16_t); i++){
    if(i == cnt){
        a=cs1621_data[i] & 0xFFF0;
      } else {
        a=cs1621_data[i];
      }
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_SPI_Transmit( &hspi1, &a, 1/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
      //HAL_Delay(1);
    }
}
void cs1621_showValue(int16_t val){
    SPI1_Reinit(SPI_DATASIZE_9BIT);
    uint16_t addr=0x146;
    uint8_t v[3];
    v[0]=get_digit(abs(val % 10));
    v[1]=get_digit(abs((val / 10) % 10));
    v[2]=get_digit(abs(val / 100));
    if(val < 0) {
        v[2] |= 0x01; // sign
    }
    v[0] |= 0x01; // decimal point
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Transmit( &hspi1, &addr, 1/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
    SPI1_Reinit(SPI_DATASIZE_8BIT);
    HAL_SPI_Transmit( &hspi1, v, 3/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void cs1621_showSource(enum cs1621_source source){
    uint8_t buff=0;

    //uint16_t buff=0x14D0; //!, PV, SOC, NA
    //uint16_t buff=0x14C0; //BATT., TYPE, V, C, F
    
    switch(source){
        case CS1621_PV:
            buff = 0x2;
            break;
        case CS1621_TYPE:
            buff = 0x20;
            break;            
        case CS1621_BATTERY:
            buff = 0x10;
            break;
        case CS1621_LOAD:
            buff = 0x40;
            break;
        case CS1621_SOC:
            buff = 0x4;
            break;
        default:
        break;      
    }
    send8bitVal(0x14c, buff);
}

void cs1621_showUnits(enum cs1621_unit unit){
    uint8_t buff=0;
    //uint16_t buff=0x1440; //holy bulb <> 0x80, A, kWh, % 
    //uint16_t buff=0x1450; //V, C, F
    
    switch(unit){
        case CS1621_VOLTAGE:
            buff |= 0x01;
            break;
        case CS1621_CURRENT:
            buff |= 0x10;
            break;
        case CS1621_ENERGY:
            buff |= 0x20;
            break;
        case CS1621_PERCENT:
            buff |= 0x40;
            break;
        case CS1621_CELSIUS:
            buff |= 0x02;
            break;
        case CS1621_FAHRENHEIT:
            buff |= 0x04;
            break;
        default:
        break;      
    }
    send8bitVal(0x144, buff);
}

void cs1621_test(uint8_t val){
    uint8_t i=0;
    uint16_t buff[2];
    switch(val){
        case 0:
            buff[i++] = 0x14AF;
            buff[i++] = 0x14BA;
            break;
        case 1:
            buff[i++] = 0x14A6;
            buff[i++] = 0x14B0;
            break;
        case 2:
            buff[i++] = 0x14AD; //Bn //en //ci
            buff[i++] = 0x14B6;
            break;
        case 3:
            buff[i++] = 0x14AF; //Bn //en //ci
            buff[i++] = 0x14B4;
            break;
        case 4:
            buff[i++] = 0x14A6; //Bn //en //ci
            buff[i++] = 0x14BC;
            break;
        case 5:
            buff[i++] = 0x14AB;
            buff[i++] = 0x14BC;
            break;
        case 6:
            buff[i++] = 0x14AB;
            buff[i++] = 0x14BE;
            break;
        case 7:
            buff[i++] = 0x14AE;
            buff[i++] = 0x14B0;
            break;
        case 8:
            buff[i++] = 0x14AF;
            buff[i++] = 0x14BE;
            break;
        case 9:
            buff[i++] = 0x14AF;
            buff[i++] = 0x14BC;
            break;                        
        default:
            buff[i++] = 0x14A0;
            buff[i++] = 0x14B1;
            break;
    }
    SPI1_Reinit(SPI_DATASIZE_13BIT);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_SPI_Transmit( &hspi1, buff, 1/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_SPI_Transmit( &hspi1, buff+1, 1/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

uint8_t get_digit(uint8_t val){
    uint8_t i=0;
    uint16_t buff[2];
    switch(val){
        case 0:
            return 0xFA;
            break;
        case 1:
            return 0x60;
            break;
        case 2:
            return 0xD6; //Bn //en //ci
            break;
        case 3:
            return 0xF4; //Bn //en //ci
            break;
        case 4:
            return  0x6C; //Bn //en //ci
        case 5:
            return 0xBC;
            break;
        case 6:
            return  0xBE;
            break;
        case 7:
            return 0xE0;
            break;
        case 8:
            return  0xFE;
            break;
        case 9:
            return 0xFC;
            break;                        
        default:
            return  0x00;
            break;
    }
}
void send13bitVal(uint16_t buff){
    SPI1_Reinit(SPI_DATASIZE_13BIT);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Transmit( &hspi1,&buff, 1,  100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void send8bitVal(uint16_t addr, uint8_t data){
    SPI1_Reinit(SPI_DATASIZE_9BIT);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Transmit( &hspi1, &addr, 1/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
    SPI1_Reinit(SPI_DATASIZE_8BIT);
    HAL_SPI_Transmit( &hspi1, &data, 1/*sizeof(uint16_t)*sizeof(spi_data)*/,  100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void SPI1_Reinit(uint32_t spiDatasize)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = spiDatasize;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
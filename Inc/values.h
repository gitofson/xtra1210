#define DEFAULT_SLAVE_ID 2
#define RECEIVED_DATA_MAX_LENGTH 8
#define TRASNSMIT_DATA_MAX_LENGTH 128

#define INFO_VENDOR_NAME "EPsolar Tech co., LtdTriRon1210"
#define INFO_PRODUCT_CODE "pic17f1705dcac"
#define INFO_VERSION "00.00"

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SLAVE_ID           0x11
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17
#define MODBUS_FC_READ_DEVICE_INFO          0x2b

#define VAL_RTD_ARRAY_VOLTAGE               0x00
#define VAL_RTD_ARRAY_CURRENT               0x01
#define VAL_RTD_ARRAY_POWER_LO              0x02
#define VAL_RTD_ARRAY_POWER_HI              0x03
#define VAL_RTD_BATTERY_VOLTAGE             0x04
#define VAL_RTD_BATTERY_CURRENT             0x05
#define VAL_RTD_BATTERY_POWER_LO            0x06
#define VAL_RTD_BATTERY_POWER_HI            0x07


typedef union _word {
	struct {
		unsigned char lo;
		unsigned char hi;
	}	byte;
	unsigned short	word;
} t_word;


typedef union _request {
    struct _req {
        uint8_t slave_id;
        uint8_t modbus_function_code;
        union _rest {
            struct {
                uint8_t mei;
                uint8_t code_id;
                uint8_t code_object;
            } info;
            struct _data {
                t_word address;
                t_word length;
            } data;
        } rest;
        t_word crc;
    } req;
	uint8_t buff[sizeof(struct _req)];
} t_request;
t_request g_request;

extern uint8_t g_readyToSend;
extern uint8_t g_tx_buff[TRASNSMIT_DATA_MAX_LENGTH];
extern uint8_t g_tx_buff_length;
extern uint8_t g_uart_free;
extern UART_HandleTypeDef huart1;

//getRatedData (modbus_t *ctx)
    uint16_t g_ratedData[16];
    /*
    int         registerAddress = 0x3000;
    int         numBytes = 0x09;                  // 0x0A and up gives 'illegal data address' error
    uint16_t    buffer[ 32 ];
    */
    
/*
    float   pvArrayRatedVoltage     = ((float) buffer[ 0x00 ]) / 100.0;
    float   pvArrayRatedCurrent     = ((float) buffer[ 0x01 ]) / 100.0;

    long temp  = buffer[ 0x03 ] << 16;
    temp |= buffer[ 0x02 ];
    float pvArrayRatedPower =  (float) temp / 100.0;

    float   batteryRatedVoltage     = ((float) buffer[ 0x04 ]) / 100.0;
    float   batteryRatedCurrent     = ((float) buffer[ 0x05 ]) / 100.0;
 
    temp  = buffer[ 0x07 ] << 16;
    temp |= buffer[ 0x06 ];
    float batteryRatedPower =  (float) temp / 100.0;

    uint16_t    chargingMode = buffer[ 0x08 ];                  // 0x01 == PWMM
Charging equipment rated input voltage 3000 PV array rated voltage V 100
Charging equipment rated input current 3001 PV array rated current A 100
Charging equipment rated input power L 3002 PV array rated power (low 16 bits) W 100
Charging equipment rated input power H 3003 PV array rated power (high 16 bits) W 100
Charging equipment rated output voltage 3004 Battery's voltage V 100
Charging equipment rated output current 3005 Rated charging current to battery A 100
Charging equipment rated output power 3006 Rated charging power to battery W 100
Charging equipment rated output power 3007 W 100
Charging mode 3008 0001H-PWM 
Rated output current of load 300E A 100

    */
//getRealTimeData:
    t_word g_realTimeData[32];
    /*
    int         registerAddress = 0x3100;
    uint8_t     numBytes = 0x13;                  // 0x14 and up gives 'illegal data address' error
    uint16_t    buffer[ 32 ];
    */
    /*
       // ---------------------------------------------
    //  Photo Voltaic Values - Volts, Amps and Watts
    float pvArrayVoltage =  ((float) buffer[ 0x00 ]) / 100.0;
    float pvArrayCurrent =  ((float) buffer[ 0x01 ]) / 100.0;
    long    temp = buffer[ 0x03 ] << 16;
    temp |= buffer[ 0x02 ];
    float pvArrayPower   =  (float) temp / 100.0;

    //  Battery Values - Volts, Amps and Watts
    float batteryVoltage =  ((float) buffer[ 0x04 ]) / 100.0;
    float batteryCurrent =  ((float) buffer[ 0x05 ]) / 100.0;
    temp = buffer[ 0x07 ] << 16;
    temp |= buffer[ 0x06 ];
    float batteryPower   =  (float) temp / 100.0;
    
    //  Load Values - Volts, Amps and Watts
    float loadVoltage =  ((float) buffer[ 0x0C ]) / 100.0;
    float loadCurrent =  ((float) buffer[ 0x0D ]) / 100.0;
    temp    = buffer[ 0x0F ] << 16;
    temp |= buffer[ 0x0E ];
    float loadPower   =  (float) temp / 100.0;
    
    
    float   batteryTemp =  ((float) buffer[ 0x10 ]) / 100.0;
    float   caseTemp =  ((float) buffer[ 0x11 ]) / 100.0;
    float   componentsTemp =  ((float) buffer[ 0x12 ]) / 100.0;
 
    //  Our LS1024B controller doesn't seem to support any register data above 0x12
    //float   batterySOC =  ((float) buffer[ 0x1A ]) / 100.0;
    //float   remoteBatteryTemp =  ((float) buffer[ 0x1B ]) / 100.0;
    //float   systemRatedVoltage =  ((float) buffer[ 0x1D ]) / 100.0; //Current system rated votlage. 1200, 2400 represent 12V, 24V
    */

//getRealTimeStatus (modbus_t *ctx)
    uint16_t    g_realTimeStatus[2];
    /*
    int         registerAddress = 0x3200;
    int         numBytes = 0x2;
    uint16_t    buffer[ 32 ];
    uint16_t    batteryStatus =  buffer[ 0x00 ];
    */
    /*
     *  D3-D0: 01H Overvolt , 00H Normal , 02H Under Volt, 03H Low Volt Disconnect, 04H Fault
     *  D7-D4: 00H Normal, 01H Over Temp.(Higher than the warning settings), 02H Low Temp.( Lower than the warning settings),
     *  D8: Battery inerternal resistance abnormal 1, normal 0
     *  D15: 1-Wrong identification for rated voltage
     */
    
    
    /*uint16_t    chargingStatus =  buffer[ 0x01 ];
    
     *  D15-D14: Input volt status. 00 normal, 01 no power connected, 02H Higher volt input, 03H Input volt error.
     *  D13: Charging MOSFET is short.
     *  D12: Charging or Anti-reverse MOSFET is short.
     *  D11: Anti-reverse MOSFET is short.
     *  D10: Input is over current.
     *  D9: The load is Over current.
     *  D8: The load is short.
     *  D7: Load MOSFET is short.
     *  D4: PV Input is short.
     *  D3-2: Charging status. 00 No charging, 01 Float, 02 Boost, 03 Equlization.
     *  D1: 0 Normal, 1 Fault
     */

//getStatisicalParameters (modbus_t *ctx)
    uint16_t    g_statisticalParameters[32];
/*
    int         registerAddress = 0x3300;
    int         numBytes = 0x1E;                  
    uint16_t    buffer[ 32 ];
    */
/*
    float   maximumInputVoltageToday = ((float) buffer[ 0x00 ]) / 100.0;
    float   minimumInputVoltageToday = ((float) buffer[ 0x01 ]) / 100.0;
    float   maximumBatteryVoltageToday = ((float) buffer[ 0x02 ]) / 100.0;
    float   minimumBatteryVoltageToday = ((float) buffer[ 0x03 ]) / 100.0;

    long temp  = buffer[ 0x05 ] << 16;
    temp |= buffer[ 0x04 ];
    float consumedEnergyToday =   (float) temp / 100.0;

    temp  = buffer[ 0x07 ] << 16;
    temp |= buffer[ 0x06 ];
    float consumedEnergyMonth =   (float) temp / 100.0;

    temp  = buffer[ 0x09 ] << 16;
    temp |= buffer[ 0x08 ];
    float consumedEnergyYear =   (float) temp / 100.0;
    
    temp  = buffer[ 0x0B ] << 16;
    temp |= buffer[ 0x0A ];
    float totalConsumedEnergy =   (float) temp / 100.0;
    
    temp  = buffer[ 0x0D ] << 16;
    temp |= buffer[ 0x0C ];
    float generatedEnergyToday =   (float) temp / 100.0;
    
    temp  = buffer[ 0x0F ] << 16;
    temp |= buffer[ 0x0E ];
    float generatedEnergyMonth =   (float) temp / 100.0;

    temp  = buffer[ 0x11 ] << 16;
    temp |= buffer[ 0x10 ];
    float generatedEnergyYear =   (float) temp / 100.0;

    temp  = buffer[ 0x13 ] << 16;
    temp |= buffer[ 0x12 ];
    float totalGeneratedEnergy =   (float) temp / 100.0;
    
    temp  = buffer[ 0x15 ] << 16;
    temp |= buffer[ 0x14 ];
    float CO2Reduction =   (float) temp / 100.0;
    
    temp  = buffer[ 0x1C ] << 16;
    temp |= buffer[ 0x1B ];
    float batteryCurrent =   (float) temp / 100.0;
    
    float batteryTemp =   ((float) buffer[ 0x01D ]) / 100.0;
    float ambientTemp =   ((float) buffer[ 0x01E ]) / 100.0;*/
    

    //getSettings (modbus_t *ctx)
    g_settings[32];
/*
    int         registerAddress = 0x9000;
    int         numBytes = 0x0F;                    // 0x10 and up gives 'illegal data address' error
    uint16_t    buffer[ 32 ];*/
    /*
        uint16_t    batteryType =  buffer[ 0x00 ]; //0001H- Sealed , 0002H- GEL, 0003H- Flooded, 0000H- User defined
    uint16_t    batteryCapacity =  buffer[ 0x01 ]; //[Ah]
    
    float   tempCompensationCoeff   = ((float) buffer[ 0x02 ]) / 100.0;
    float   highVoltageDisconnect   = ((float) buffer[ 0x03 ]) / 100.0;
    float   chargingLimitVoltage    = ((float) buffer[ 0x04 ]) / 100.0;
    float   overVoltageReconnect    = ((float) buffer[ 0x05 ]) / 100.0;
    float   equalizationVoltage     = ((float) buffer[ 0x06 ]) / 100.0;
    float   boostVoltage            = ((float) buffer[ 0x07 ]) / 100.0;
    float   floatVoltage            = ((float) buffer[ 0x08 ]) / 100.0;
    float   boostReconnectVoltage   = ((float) buffer[ 0x09 ]) / 100.0;

    //  Our LS1024B controller doesn't seem to support any register data above 0x0A
    float   lowVoltageReconnect     = ((float) buffer[ 0x0A ]) / 100.0;
    float   underVoltageRecover     = ((float) buffer[ 0x0B ]) / 100.0;
    float   underVoltageWarning     = ((float) buffer[ 0x0C ]) / 100.0;
    float   lowVoltageDisconnect    = ((float) buffer[ 0x0D ]) / 100.0;
    float   dischargingLimitVoltage = ((float) buffer[ 0x0E ]) / 100.0;
    uint16_t    realTimeClock1      = buffer[ 0x13 ];
    uint16_t    realTimeClock2      = buffer[ 0x14 ];
    uint16_t    realTimeClock3      = buffer[ 0x15 ];
    */

   

   
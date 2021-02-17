extern SPI_HandleTypeDef hspi1;
enum  cs1621_unit{
    CS1621_VOLTAGE,
    CS1621_CURRENT,
    CS1621_ENERGY,
    CS1621_PERCENT,
    CS1621_CELSIUS,
    CS1621_FAHRENHEIT,
};
enum cs1621_source{
    CS1621_PV,
    CS1621_BATTERY,
    CS1621_LOAD,
    CS1621_TYPE,
    CS1621_SOC
};
void cs1621_init();
void cs1621_demo(uint8_t);
void cs1621_showValue(int16_t);
void cs1621_test(uint8_t);
void cs1621_showUnits(enum cs1621_unit);
void cs1621_showSource(enum cs1621_source);

# xtra1210
STM32F030C8T6 Firmware Created for Epever XTRA1210N MPPT solar charge controller.

This is firmware for MCU STM32F030 (heart of EPEVER solar charge controllers).
---
What is slightly implemented:
- cs1621 chip driver for showing segments on XDS1 display
- MODBUS RTU protocol for showing realtime data
- ADC 10 channels in DMA mode
- PWM complementary signal for pseudo-synchronous DC/DC step down converter working on 40kHz

---
STM32F030C8T6 pinout description:
GPIO in:
- PB14 XDS1 button 1
- PB15 XDS1 button 2
- PB10 output Vpv/Vbat comparator, set if Vbat > Vpv

GPIO out:
- PA8  IR2106 like Hin (main DC/DC switch IRF100200b2, 100V, 68A, 8.mOhm, 77nC). Passed throuhgt two 74HC132 NANDs
- PB13 IR2106 like Hlo (synchronous switch IRF100200b2 connected parallel with fast diode). Passed throught two 74HC132 NANDs
- PB1 this enables output of previous two ones using 74HC132 logic
- PB11 red LED on XDS1 display. Active High
- PC13 green LED on XDS1 display. Active Low

ADC in:
- PA0 Vpv, I guess  Vpv = x[0]*4962/1000 [100mV];
- PA1 Vbat, I guess  Vbat = x[1]*1967/1000 [100mV];
- PA4 V?
- PA2 thermal sensor
- PA3 load current obtained using LMV932
- PA5 battery current obtained using LMV932 (can be negative), I guess Ibat = (x[5]-0x170)*40/100
- PA7 177mV reference (3v3-60k-PA7-3k9-GND)
- PB0 another voltage reference

UART:
- PA10 Rx
- PA9 Tx
- PB9 Rx/Tx switch (low means receive) 

SPI (XDS1 driving):
- PB3  nWR (clock)
- PA15 nCR (NSS) must be set by software in this mode
- PB5  data

headers on board:
- PA12 - pin 2 PS1 heade
- PB8 - pin1 P5 header
----
HOWTO build this source:
- download [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- load xtra1210n.ioc from cloned path by git
- select Makefile in ProjectManager->Project->toolchain inside STM32CubeMX
- push GENERATE CODE button
- all necessary files should be shown in your project folder obtained by git 
- build using arm-none-eabi-gcc and make



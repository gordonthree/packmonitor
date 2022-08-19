#include <Arduino.h>

#ifdef  MCU_AVR128DA28
// LEDs used on breadboard only
#define LED1 PIN_PA4 // 
#define LED2 PIN_PA5 // 
#define LED3 PIN_PA6 // 
#define LED4 PIN_PA7 // 

// left hand side of the chip
#define ADC0 PIN_PD0 // PD0
#define ADC1 PIN_PD1 // PD1
#define ADC2 PIN_PD2 // PD2
#define ADC3 PIN_PD3 // PD3
#define ADC4 PIN_PD4 // PD4
#define ADC5 PIN_PD5 // PD4

// Stand alone slave-only pins
#define SCL  PIN_PA3 // Slave TWI0
#define SDA  PIN_PA2 // " "

// Dual-mode slave pins
#define SCL0 PIN_PA3 // Slave TWI0
#define SDA0 PIN_PA2 // " "

// Dual-mode master pins
#define SCL1 PIN_PC3 // Master TWI0 
#define SDA1 PIN_PC2 // " "

#elif   MCU_AVR128DA32
// LEDs used on breadboard only
#define LED1 PIN_PA4
#define LED2 PIN_PA5
#define LED3 PIN_PA6
#define LED4 PIN_PA7

// bottom right of processor
#define ADC0 PIN_PD1 // PD0
#define ADC1 PIN_PD2 // PD1
#define ADC2 PIN_PD3 // PD2
#define ADC3 PIN_PD4 // PD3
#define ADC4 PIN_PD5 // PD4
#define ADC5 PIN_PD6 // PD4

// human readable pin names
#define VBUS      PIN_PD0 // INPUT bus voltage divider
#define TS0       PIN_PD4 // INPUT temp sensor 0
#define TS1       PIN_PD5 // INPUT temp sensor 1
#define TS2       PIN_PD6 // INPUT temp sensor 2
#define VPACK     PIN_PF5 // INPUT pack voltage divider

#define GATEDIS   PIN_PF2 // OUTPUT Discharge enable mosfet
#define GATECHG   PIN_PF4 // OUTPUT Charge enable mosfet

#define BUSREADY  PIN_PD1 // INPUT bus ready flag from hotswap interface, needs pullup
#define BUSENABLE PIN_PD2 // OUTPUT enable pin hot-swap interface, active high


// Stand alone slave-only pins
#define SCL  PIN_PA3 // Slave TWI0
#define SDA  PIN_PA2 // " "

// Dual-mode slave pins
#define SCL0 PIN_PA3 // SDA_H host side bus clock PULLUP REQD
#define SDA0 PIN_PA2 // SDA_H host side bus data PULLUP REQD

// Dual-mode master pins
#define SCL1 PIN_PC3 // SCL_C client side bus clock PULLUP REQD
#define SDA1 PIN_PC2 // SDA_C client side bus data PULLUP REQD
#endif
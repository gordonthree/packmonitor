#include <Arduino.h>

#ifdef  MCU_AVR128DA28
// LEDs used on breadboard only
#define LED1 PIN_PA4 // 
#define LED2 PIN_PA5 // 
#define LED3 PIN_PA6 // 
#define LED4 PIN_PA7 // 

// left hand side of the chip
#define ADC0 PIN_PD1 // PD1
#define ADC1 PIN_PD2 // PD2
#define ADC2 PIN_PD3 // PD3
#define ADC3 PIN_PD4 // PD4

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
#define ADC0 PIN_PD3
#define ADC1 PIN_PD4
#define ADC2 PIN_PD5
#define ADC3 PIN_PD6

// Stand alone slave-only pins
#define SCL  PIN_PA3 // Slave TWI0
#define SDA  PIN_PA2 // " "

// Dual-mode slave pins
#define SCL0 PIN_PA3 // Slave TWI0
#define SDA0 PIN_PA2 // " "

// Dual-mode master pins
#define SCL1 PIN_PF3 // Master TWI1 
#define SDA1 PIN_PF2 // " "
#endif
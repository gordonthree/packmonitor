#include <Arduino.h>

#ifdef MCU_ATMEGA328P
#define LED1 PD2
#define LED2 PD3
#define LED3 PD4
#define LED4 PD5

//SDA 18 (A4) SCL 19 (A5)
#define SCL PC5 // A5
#define SDA PC4 // A4

#define ADC0 A1
#define ADC1 A2
#define ADC2 A3
#elif MCU_ATMEGA4808
#define LED1 PF2
#define LED2 PF3
#define LED3 PF4
#define LED4 PF5

#define ADC0 A0
#define ADC1 A2
#define ADC2 A2
// Nano Every: SDA 4 SCL 5 
#define SCL PA3 
#define SDA PA2 
#elif MCU_AVR128DA28
#define LED1 7 // PA0
#define LED2 8 // PA1
#define LED3 9 // PA2
#define LED4 10 // PA3

#define ADC0 13 // PD1
#define ADC1 14 // PD2
#define ADC2 15 // PD3
#define ADC3 16 // PD4
// Nano Every: SDA 4 SCL 5 
#define SCL 3 // PA3
#define SDA 2 // PA2
#elif MCU_AVR128DA32
#define LED1 4 // PA0
#define LED2 5 // PA1
#define LED3 6 // PA2
#define LED4 7 // PA3

#define ADC0 15 // PD1
#define ADC1 16 // PD2
#define ADC2 17 // PD3
#define ADC3 18 // PD3
// Nano Every: SDA 4 SCL 5 
//#define SCL 3 // PA3
//#define SDA 2 // PA2
#elif MCU_NANOEVERY
#define LED1 5 
#define LED2 4
#define LED3 3
#define LED4 2

#define ADC0 15
#define ADC1 16
#define ADC2 17
#define ADC3 18

// Nano Every: SDA 4 SCL 5 
#define SDA 18 
#define SCL 19 
#endif
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
#elif MCU_ATMEGA4809
#define LED1 LED_BUILTIN
#define LED2 4
#define LED3 3
#define LED4 2

#define ADC0 14
#define ADC1 15
#define ADC2 16

// Nano Every: SDA 4 SCL 5 
#define SCL 19 
#define SDA 18 
#endif
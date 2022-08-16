#include <Arduino.h>

const uint8_t txBufferSize = 50;
const uint8_t rxBufferSize = 50;
const uint8_t adcBufferSize = 4;        // adc1 current, adc2 pack voltage, adc3 bus voltage, adc4 coulumb counter

struct I2C_RX_DATA {
  uint8_t cmdAddr;                // single byte command register
  uint8_t dataLen;                // number of bytes in buffer
  uint8_t cmdData[rxBufferSize];  // room for N bytes of data
};

struct I2C_TX_DATA {
  uint8_t dataLen;
  uint8_t cmdData[txBufferSize];   // room for N bytes of data
};

struct ADC_DATA {
  uint8_t adcPin   = 0;                 // Arduino pin number?
  int32_t adcRaw   = 0;                 // raw value
  int32_t rawMin   = 0;                 // raw value
  int32_t rawMax   = 0;                 // raw value
};

struct DEBUG_MSGS {
  uint8_t messageNo;
  char    messageTxt[50];
};

typedef struct 
{
  float  tempReading = 0.0;
  float  lowReading  = 200.0;
  float  highReading = -100.0;
  uint8_t sensorAddr  = 0; 
} tempsensor_t;

union ulongArray
{
  uint32_t longNumber=0;
  uint8_t  byteArray[4];
};

union floatArray
{
  float   floatNumber=0.0;
  uint8_t byteArray[4];
};

union longArray
{
  int32_t longNumber=0;
  uint8_t byteArray[4];
};

union doubleArray {
  float doubleVal;
  uint8_t byteArray[4];
};

float raw2amps(int32_t rawVal, float vcc, float mvA)
{
    // float mvPa    = 0.136;  // 0.136v or 136mV per amp
    float Amps    = 0.0;
    float Volts   = 0.0;

    Volts = (float)(rawVal * (vcc / 1024.0)) - (vcc / 2);
    Amps =  (float)Volts / mvA;

    return Amps;
}

float raw2volts(int32_t rawVal, float vcc, float scale)
{
    float Volts   = 0.0;
  
    Volts = (float)(rawVal * (vcc / 1024.0)) / scale;
    //Amps =  (float)Volts / acsmvA;
    //adcDataBuffer[0].Amps  = Amps;
    return Volts;
}

 
 
/**
* \brief    Converts the ADC result into a temperature value.
*
*           The temperature values are read from the table.
*
* \param    adc_value  The converted ADC result
* \return              The temperature in 0.1 °C
*
*/
float raw2temp(unsigned int adc_value){
 
  /* Read values directly from the table. */
  return (float) NTC_table[ adc_value ] / 100.0;
};




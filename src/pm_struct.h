#include <Arduino.h>

const uint8_t txBufferSize = 50;
const uint8_t rxBufferSize = 50;
const uint8_t adcBufferSize = 4;        // adc1 current, adc2 pack voltage, adc3 bus voltage, adc4 coulumb counter

struct I2C_RX_DATA {
  uint8_t cmdAddr               = 0;    // single byte command register
  char cmdData[rxBufferSize] = {};   // room for N bytes of data
  size_t  dataLen               = 0;    // number of bytes in buffer
};

struct I2C_TX_DATA {
  uint8_t cmdData[txBufferSize] = {};   // room for N bytes of data
  size_t  dataLen = 50;
};

struct ADC_DATA {
  uint8_t adcPin   = 0;                 // Arduino pin number?
  int32_t adcRaw   = 0;                 // raw value
  int32_t adcMin   = 0;                 // raw value
  int32_t adcMax   = 0;                 // raw value
  double  Amps     = 0.0;               // formatted value
  double  Volts    = 0.0;               // formatted value
};

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



volatile I2C_RX_DATA rxData;
volatile I2C_TX_DATA txData;
volatile ADC_DATA    adcDataBuffer[adcBufferSize];  // Enough room to store three adc readings

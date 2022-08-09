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
  int32_t adcMin   = 0;                 // raw value
  int32_t adcMax   = 0;                 // raw value
  float   Amps     = 0.0;               // formatted value
  float   Volts    = 0.0;               // formatted value
};

struct DEBUG_MSGS {
  uint8_t messageNo;
  char    messageTxt[50];
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

union longArray
{
    int32_t longNumber=0;
    uint8_t byteArray[4];
};

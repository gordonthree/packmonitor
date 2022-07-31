#include <Arduino.h>

const uint8_t txBufferSize = 50;
const uint8_t rxBufferSize = 50;
struct I2C_RX_DATA {
  uint8_t cmdAddr = 0;                  // single byte command register
  uint8_t cmdData[rxBufferSize] = {};   // room for N bytes of data
  char padding[10] = {};                // padding not sure it's needed
  size_t dataLen = 0;
};

struct I2C_TX_DATA {
  uint8_t cmdData[txBufferSize] = {};   // room for N bytes of data
  char padding[10] = {};                // padding not sure it's needed
  size_t dataLen = 0;
};

volatile I2C_RX_DATA rxData;
volatile I2C_TX_DATA txData;

#include <Arduino.h>
#include <Wire.h>

#include "pm_fram.h"

// function to eventually save data to on board FRAM
void writeFRAMuint(uint8_t myAddr, uint32_t myData) { 

}

// function to read byte from FRAM
uint8_t readFRAMbyte(uint8_t myAddr) { 

}

// function to read uint from FRAM
uint32_t readFRAMuint(uint8_t myAddr) { 
  uint32_t framData = 0;
  if (myAddr==0x39) { // pack voltage
    framData = adcDataBuffer[2].adcRaw;
  } else if (myAddr==0x3E) { // bus voltage
    framData = adcDataBuffer[1].adcRaw;
  } else if (myAddr==0x33) { // active current
    framData = adcDataBuffer[0].adcRaw;
  }
  return framData;
}

// function to read uint from FRAM (eventually)
float readFRAMfloat(uint8_t myAddr) { 
  float framData = 0.0;
  if (myAddr==0x39) { // pack voltage
    framData = adcDataBuffer[2].Volts;
  } else if (myAddr==0x3E) { // bus voltage
    framData = adcDataBuffer[1].Volts;
  } else if (myAddr==0x33) { // active current
    framData = adcDataBuffer[0].Amps;
  }
  return framData;
}


// function to read ulong from FRAM
uint32_t readFRAMulong(uint8_t myAddr) { 

}

// function to read int from FRAM
int16_t readFRAMint(uint8_t myAddr) { 

}

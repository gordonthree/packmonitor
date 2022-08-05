#ifndef pm_fram_h
#define pm_fram_h
#endif

#include <Arduino.h>
#include <Wire.h>

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


void writeFRAMuint(uint8_t myAddr, uint32_t myData);
uint8_t readFRAMbyte(uint8_t myAddr);
uint32_t readFRAMuint(uint8_t myAddr);
float readFRAMfloat(uint8_t myAddr);
uint32_t readFRAMulong(uint8_t myAddr);
int16_t readFRAMint(uint8_t myAddr);


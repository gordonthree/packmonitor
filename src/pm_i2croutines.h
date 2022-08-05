#ifndef pm_i2croutines_h
#define pm_i2croutines_h
#endif

#include <Arduino.h>
#include <Wire.h>

void i2cWriteUL(uint8_t slaveAddress, uint8_t cmdAddress, uint32_t cmdData) ;
float i2cReadF(uint8_t slaveAddress, uint8_t cmdAddress) ;
uint32_t i2cReadUL(uint8_t slaveAddress, uint8_t cmdAddress);
long i2cReadI(int slaveAddress, int cmdAddress);

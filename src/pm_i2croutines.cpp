#include <Arduino.h>
#include <Wire.h>

void i2cWriteUL(uint8_t slaveAddress, uint8_t cmdAddress, uint32_t cmdData) {
  // Uint32Buff txbuffer;
  char txbuffer[15];
  sprintf(txbuffer, "%lu", cmdData);
  Wire.beginTransmission(slaveAddress);               // begin transaction with slave address
  Wire.write(0x60);                                   // send register address byte
  Wire.write(txbuffer);                                // send bytes
  Wire.endTransmission(true);                         // end transaction with a stop
  // sprintf(buff, "Wrote %s to slave 0x%X at address 0x%X", txbuffer, slaveAddress, cmdAddress);
  // telnet.println(buff);
}

float i2cReadF(uint8_t slaveAddress, uint8_t cmdAddress) {
  // uint8_t byteCnt = 0;
  uint8_t readBytes = 5;
  char rxBuffer[20];
  float theResult = 0.0;
  char stopChar = '\0';                             // unix null char
  Wire.beginTransmission(slaveAddress);             // start transaction
  Wire.write(cmdAddress);                                 // tell slave we want to read this register
  Wire.endTransmission(false);                      // send instruction, retain control of bus
  Wire.requestFrom(slaveAddress, readBytes, (bool) true);     // request 6 bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, rxBuffer, readBytes);   // read five bytes or until the first null

  theResult = strtod(rxBuffer, NULL);

  return theResult;
}

uint32_t i2cReadUL(uint8_t slaveAddress, uint8_t cmdAddress) {
  char      rxBuffer[20];
  char      stopChar  = '\0';                             // unix null char
  uint8_t   readBytes = 11;
  uint32_t  theResult = 0.0;
  Wire.beginTransmission(slaveAddress);             // start transaction
  Wire.write(cmdAddress);                                 // tell slave we want to read this register
  Wire.endTransmission(false);                      // send instruction, retain control of bus
  Wire.requestFrom(slaveAddress, readBytes, (bool) true);     // request 6 bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, rxBuffer, readBytes);   // read five bytes or until the first null
  // telnet.println(rxBuffer);
  theResult = strtoul(rxBuffer, NULL, 10);
  
  return theResult;
}

long i2cReadI(int slaveAddress, int cmdAddress) {
    // uint8_t byteCnt = 0;
  uint8_t readBytes = 5;
  char rxBuffer[20];
  long theResult = 0.0;
  char stopChar = '\0';                                       // unix null char
  Wire.beginTransmission(slaveAddress);                       // start transaction
  Wire.write(cmdAddress);                                     // tell slave we want to read this register
  Wire.endTransmission(false);                                // send instruction, retain control of bus
  Wire.requestFrom(slaveAddress, readBytes, (bool) true);     // request bytes from slave device and then release bus
  Wire.readBytesUntil(stopChar, rxBuffer, readBytes);         // read bytes or until the first null

  theResult = strtol(rxBuffer, NULL, 10);

  return theResult;
}

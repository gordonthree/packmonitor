#include <I2C_eeprom.h>
#include "pm_fram.h"


// provide fram eeprom address and device size
FRAMSTORAGE::FRAMSTORAGE() 
{
  // hello, world
}

// read the eeprom into the buffer
// void FRAMSTORAGE::begin(I2C_eeprom &_ee_fram)
void FRAMSTORAGE::begin()
{
  // ptr = &_ee_fram;
  // ptr->begin();                                             // init eeprom library
  // _ee_fram.begin();
}

// void FRAMSTORAGE::load()
// {
//   for (uint16_t x = 0; x < ee_buffer_size; x++)
//   {
//     _ee_fram.readBlock((x * ee_record_size) + ee_start_offset, fram_buffer[x].byteArray, ee_record_size);
//   }
// }

// // write the buffer into the eeprom
// void FRAMSTORAGE::save() 
// {
//   for (uint16_t x = 0; x < ee_buffer_size; x++)
//   {
//     _ee_fram.writeBlock((x * ee_record_size) + ee_start_offset, fram_buffer[x].byteArray, ee_record_size);
//   }
// }

void FRAMSTORAGE::addByteArray(uint8_t dataAddr, uint8_t * dataArray)           /* load i2c data into buffer */
{ // memcpy(dst, src, len)
  memcpy(fram_buffer[dataAddr].byteArray, byteArray, 24);
}

uint8_t * FRAMSTORAGE::getArrayData(uint8_t dataAddr)                   // return the entire byte array
{
  uint8_t * byteArray = fram_buffer[dataAddr].byteArray;
  return byteArray;
}

void FRAMSTORAGE::addDouble(uint8_t dataAddr, uint32_t ts, double doubleVal) /* update array with a double from userland */
{ // memcpy(dst, src, len)
  dbuffer.doubleVal = doubleVal;

  fram_buffer[dataAddr].data.ts    = ts;
  memcpy(fram_buffer[dataAddr].data.array, dbuffer.byteArray, 4);
}

void FRAMSTORAGE::addUInt(uint8_t dataAddr, uint32_t ts, uint32_t uintVal) /* update array with a unsigned int from userland */
{ // memcpy(dst, src, len)
  ubuffer.longNumber = uintVal;

  fram_buffer[dataAddr].data.ts    = ts;
  memcpy(fram_buffer[dataAddr].data.array, ubuffer.byteArray, 4);
}

void FRAMSTORAGE::addSInt(uint8_t dataAddr, uint32_t ts, int32_t intVal)   /* update array with a signed integer from userland */
{ // memcpy(dst, src, len)
  lbuffer.longNumber                = intVal;
  fram_buffer[dataAddr].data.ts    = ts;
  memcpy(fram_buffer[dataAddr].data.array, lbuffer.byteArray, 4);
}

void FRAMSTORAGE::addByte(uint8_t dataAddr, uint32_t ts, uint8_t byteVal)   /* update array with a single byte from userland */
{
  fram_buffer[dataAddr].data.ts       = ts;
  fram_buffer[dataAddr].data.array[0] = byteVal;
}

void FRAMSTORAGE::addRaw(uint8_t dataAddr, uint32_t ts, int32_t rawVal)   /* update array with a raw adc data (16 bits) */
  fram_buffer[dataAddr].data.ts       = ts;     // update record timestamp
  fram_buffer[dataAddr].data.raw      = rawVal; // update raw value
}

uint32_t FRAMSTORAGE::getTimeStamp(uint8_t dataAddr)                    // get the timestamp for this record
{
  return fram_buffer[dataAddr].data.ts;
}

int32_t FRAMSTORAGE::getRaw(uint8_t dataAddr)                        // don't know what this would be useful for
{
  return fram_buffer[dataAddr].data.raw;
}

uint32_t FRAMSTORAGE::getDataUInt(uint8_t dataAddr)                 // convert byte array to unsigned int
{ // memcpy(dst, src, len)
  memcpy(ubuffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  return ubuffer.longNumber;
}

int32_t FRAMSTORAGE::getDataSInt(uint8_t dataAddr)                  // convert byte array to signed int
{ // memcpy(dst, src, len)
  memcpy(lbuffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  return lbuffer.longNumber;
}

double FRAMSTORAGE::getDataDouble (uint8_t dataAddr)                 // convert byte array to double precision
{ // memcpy(dst, src, len)
  memcpy(dbuffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  return dbuffer.doubleVal;
}

uint8_t * FRAMSTORAGE::getByteArray(uint8_t dataAddr)                   // return the entire byte array
{
  uint8_t * byteArray = fram_buffer[dataAddr].data.array;
  return byteArray;
}

uint8_t FRAMSTORAGE::getDataByte(uint8_t dataAddr)                 // read one byte from byte array
{
  return fram_buffer[dataAddr].data.array[0];;
}

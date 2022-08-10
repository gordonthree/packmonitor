#include "pm_fram.h"

// provide fram eeprom address and device size
FRAMSTORAGE::FRAMSTORAGE() 
{
  // hello, world
}

// read the eeprom into the buffer
void FRAMSTORAGE::begin(TwoWire &_i2cwire)
{
  ee_fram = I2C_eeprom(deviceAddress, deviceSize, &_i2cwire);   // setup eeprom address and size
  ee_fram.begin();                                             // init eeprom library
}

void FRAMSTORAGE::load()

{
  for (uint16_t x = 0; x < ee_buffer_size; x++)
  {
    ee_fram.readBlock(x + ee_start_byte, fram_buffer[x].byteArray, ee_record_size);
  }
}

// write the buffer into the eeprom
void FRAMSTORAGE::save() 
{
  for (uint16_t x = 0; x < ee_buffer_size; x++)
  {
    ee_fram.writeBlock(x + ee_start_byte, fram_buffer[x].byteArray, ee_record_size);
  }
}

void FRAMSTORAGE::addUserData (uint8_t dataAddr, uint_least32_t ts, uint8_t * data); /* load userland byte array into buffer */
{
  fram_buffer[dataAddr].data.ts    = ts;
  fram_buffer[dataAddr].data.array = data;
  fram_buffer[dataAddr].data.count = (uint16_t) dataAddr + 1;
}


void FRAMSTORAGE::addArrayData(uint8_t dataAddr, uint8_t * byteArray)           /* load i2c data into buffer */
{
  memcpy(byteArray, fram_buffer[dataAddr].byteArray, 4);
}


void FRAMSTORAGE::addDouble(uint8_t dataAddr, uint32_t ts, double doubleVal) /* update array with a double from userland */
{
  union doubleArray buffer;
  buffer.doubleVal = doubleVal;

  fram_buffer[dataAddr].data.ts    = ts;
  memcpy(buffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  fram_buffer[dataAddr].data.count = (uint16_t) dataAddr + 1;
}

void FRAMSTORAGE::addUInt(uint8_t dataAddr, uint32_t ts, uint32_t uintVal) /* update array with a unsigned int from userland */
{
  union ulongArray buffer;
  buffer.longNumber = uintVal;

  fram_buffer[dataAddr].data.ts    = ts;
  memcpy(buffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  fram_buffer[dataAddr].data.count = (uint16_t) dataAddr + 1;
}

void FRAMSTORAGE::addSInt(uint8_t dataAddr, uint32_t ts, int32_t intVal)   /* update array with a signed integer from userland */
{
  union longArray buffer;
  buffer.longNumber                = intVal;
  fram_buffer[dataAddr].data.ts    = ts;
  memcpy(buffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  fram_buffer[dataAddr].data.count = (uint16_t) dataAddr + 1;
}

void FRAMSTORAGE::addByte(uint8_t dataAddr, uint32_t ts, uint8_t byteVal)   /* update array with a single byte from userland */
{
  fram_buffer[dataAddr].data.ts       = ts;
  fram_buffer[dataAddr].data.array[0] = byteVal;
  fram_buffer[dataAddr].data.count    = (uint16_t) dataAddr + 1;
}

uint32_t FRAMSTORAGE::getTimeStamp(uint8_t dataAddr)                    // get the timestamp for this record
{
  uint32_t ts = fram_buffer[dataAddr].data.ts;
  return ts;
}


uint8_t * FRAMSTORAGE::getByteArray(uint8_t dataAddr)                   // return the entire byte array
{
  uint8_t * byteArray = fram_buffer[dataAddr].data.array;
  return byteArray;
}

uint16_t FRAMSTORAGE::getCount(uint8_t dataAddr)                        // don't know what this would be useful for
{
  uint16_t cnt = fram_buffer[dataAddr].data.count;
  return cnt;
}

uint32_t FRAMSTORAGE::getDataUInt(uint8_t dataAddr)                 // convert byte array to unsigned int
{
  union ulongArray buffer;
  memcpy(buffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  return buffer.longNumber;
}

int32_t FRAMSTORAGE::getDataSInt(uint8_t dataAddr)                  // convert byte array to signed int
{
  union longArray buffer;
  memcpy(buffer.byteArray, fram_buffer[dataAddr].data.array, 4);
  return buffer.longNumber;
}

double FRAMSTORAGE::getDataDouble (uint8_t dataAddr)                 // convert byte array to double precision
{
  union doubleArray buffer;
  memcpy(buffer.byteArray, fram_buffer[dataAddr].data.array, 4);
//   buffer.byteArray = fram_buffer[dataAddr].data.array;
  return buffer.doubleVal;
}


uint8_t FRAMSTORAGE::getDataByte(uint8_t dataAddr)                 // read one byte from byte array
{
  uint8_t byteVal = fram_buffer[dataAddr].data.array[0];
  return byteVal;
}

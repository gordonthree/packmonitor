#include "pm_fram.h"


// provide fram eeprom address and device size
FRAMSTORAGE::FRAMSTORAGE() 
{
  // hello, world
}

// read the eeprom into the buffer
void FRAMSTORAGE::begin(I2C_eeprom &_ee_fram)
{
  ptr = &_ee_fram;
  ptr->begin();                                             // init eeprom library
}

void FRAMSTORAGE::load()

{
  for (uint16_t x = 0; x < ee_buffer_size; x++)
  {
    ptr->readBlock((x * ee_record_size) + ee_start_byte, fram_buffer[x].byteArray, ee_record_size);
  }
}

uint16_t FRAMSTORAGE::buffer_size()
{
  return ee_buffer_size;
}

uint16_t FRAMSTORAGE::record_size()
{
  return ee_record_size;
}

// write the buffer into the eeprom
bool FRAMSTORAGE::save() 
{
  bool result;
  for (uint16_t x = 0; x < ee_buffer_size; x++)
  {
    result = ptr->writeBlockVerify((x * ee_record_size) + ee_start_byte, fram_buffer[x].byteArray, ee_record_size);
  }

  return result;
}

uint8_t * FRAMSTORAGE::dumpRecord(uint16_t x)                            // return everything
{
    ptr->readBlock((x * ee_record_size) + ee_start_byte, fram_buffer[0].byteArray, ee_record_size);
    return fram_buffer[0].byteArray;
}


void FRAMSTORAGE::addByteArray(uint8_t dataAddr, uint32_t ts, uint8_t * byteArray)           /* load i2c data into buffer */
{ // memcpy(dst, src, len)
  fram_buffer[dataAddr].data.ts = ts;
  memcpy(fram_buffer[dataAddr].data.array, byteArray, 4);
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
  fram_buffer[dataAddr].data.ts     = ts;
  memcpy(fram_buffer[dataAddr].data.array, lbuffer.byteArray, 4);
}

void FRAMSTORAGE::addByte(uint8_t dataAddr, uint32_t ts, uint8_t byteVal)   /* update array with a single byte from userland */
{
  fram_buffer[dataAddr].data.ts       = ts;
  fram_buffer[dataAddr].data.array[0] = byteVal;
}

void FRAMSTORAGE::addRaw(uint8_t dataAddr, uint32_t ts, int32_t rawVal)   /* update array with a raw adc data (16 bits) */
{
  fram_buffer[dataAddr].data.ts       = ts;     // update record timestamp
  fram_buffer[dataAddr].data.raw      = rawVal; // update raw value
}

uint32_t FRAMSTORAGE::getTimeStamp(uint8_t dataAddr)                    // get the timestamp for this record
{
  return fram_buffer[dataAddr].data.ts;
}


uint8_t * FRAMSTORAGE::getByteArray(uint8_t dataAddr)                   // return the entire byte array
{
  uint8_t * byteArray = fram_buffer[dataAddr].data.array;
  return byteArray;
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

uint8_t FRAMSTORAGE::getDataByte(uint8_t dataAddr)                 // read one byte from byte array
{
  return fram_buffer[dataAddr].data.array[0];;
}

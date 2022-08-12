#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <I2C_eeprom.h>

#define eedata_size 80                                      // constant for size of eedata structure

// a class to manage the fram data buffer
class FRAMSTORAGE 
{
  private:
    // the actual eeprom gets stored here
    // the struct is overlayed onto this storage
    union longArray
    {
    int32_t longNumber=0;
    uint8_t byteArray[4];
    } lbuffer;

    union ulongArray
    {
    uint32_t longNumber=0;
    uint8_t  byteArray[4];
    } ubuffer;

    union doubleArray
    {
    double doubleVal;
    uint8_t byteArray[4];
    } dbuffer;    
    
    typedef struct eedata 
    {
    uint32_t ts;                                             // Timestamp for when this record was created (4 bytes)
    uint8_t  array[4];                                       // Data for this address
    int32_t  raw;                                            // raw data storage (adc raw)
    } ;

    union EERECORD                                           // create union that converts custom structure into byte array
    {
    eedata  data;                                            // userland data
    uint8_t byteArray[eedata_size];                          // byte array to send over i2c or store in fram
    };

    const uint16_t ee_buffer_size = 70;               // number of recordds in the buffer array
    static const uint16_t ee_record_size = sizeof(EERECORD); // calculate size of a record in bytes
    const uint16_t ee_start_byte  = 0x64;                    // eeprom offset is 100 bytes (0x64), save that space for other uses
    
    EERECORD fram_buffer[ee_buffer_size];

    I2C_eeprom *ptr;

    uint8_t  deviceAddress;
    uint32_t deviceSize;

  public:
    FRAMSTORAGE();                                                           // empty constructor 

    void begin (I2C_eeprom &_ee_fram);                                       // setup fram connection
    void load ();                                                            // fills buffer from fram
    bool save ();                                                            // writes buffer to fram, returns false if write failed to verify
    
    void addByteArray  (uint8_t dataAddr, uint32_t ts, uint8_t * byteArray); // update array with a raw byte array (from eeprom really)

    void addDouble     (uint8_t dataAddr, uint32_t ts, double doubleVal);    // update array with a double from userland
    void addUInt       (uint8_t dataAddr, uint32_t ts, uint32_t uintVal);    // update array with a unsigned int from userland
    void addSInt       (uint8_t dataAddr, uint32_t ts, int32_t intVal);      // update array with a signed integer from userland
    void addByte       (uint8_t dataAddr, uint32_t ts, uint8_t byteVal);     // update array with a single byte from userland
    void addRaw        (uint8_t dataAddr, uint32_t ts, int32_t rawVal);      // update array with raw adc data

    uint32_t  getTimeStamp  (uint8_t dataAddr);                              // get the timestamp for the record
    uint8_t * getByteArray  (uint8_t dataAddr);                              // return the raw byte array
    int32_t   getRaw        (uint8_t dataAddr);                              // get raw adc data
    uint32_t  getDataUInt   (uint8_t dataAddr);                              // transform byte array into usigned int
    double    getDataDouble (uint8_t dataAddr);                              // transform byte array into double precision (float)
    int32_t   getDataSInt   (uint8_t dataAddr);                              // transform byte array into signed int
    uint8_t   getDataByte   (uint8_t dataAddr);                              // read one byte from byte array
    uint8_t * dumpRecord    (uint16_t recordAddr);                           // return everything
    uint16_t  record_size   (void);
    uint16_t  buffer_size   (void);
};


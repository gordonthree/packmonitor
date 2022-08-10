#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <I2C_eeprom.h>

#define PM_REGISTER_HIGHCURRENTLIMIT    0x21
#define PM_REGISTER_HIGHTEMPLIMIT       0x22
#define PM_REGISTER_LOWTEMPLIMIT        0x23
#define PM_REGISTER_HIGHVOLTLIMIT       0x24
#define PM_REGISTER_LOWVOLTLIMIT        0x25
#define PM_REGISTER_CONFIGB0YTE         0x26
#define PM_REGISTER_CONFIG1BYTE         0x27
#define PM_REGISTER_CONFIG2BYTE         0x28
#define PM_REGISTER_STATUS0BYTE         0x2C
#define PM_REGISTER_STATUS1BYTE         0x2D
#define PM_REGISTER_CLEARCOLCNTR        0x30
#define PM_REGISTER_READCOLCNTR         0x31
#define PM_REGISTER_CLEARAMPSCNTRS      0x32
#define PM_REGISTER_READLOADAMPS        0x33
#define PM_REGISTER_TOTALAMPSIN         0x34
#define PM_REGISTER_TOTALAMPSOUT        0x35
#define PM_REGISTER_LIFEAMPSIN          0x36
#define PM_REGISTER_LIFEAMPSOUT         0x37
#define PM_REGISTER_CLEARVOLTMEM        0x38
#define PM_REGISTER_READPACKVOLTS       0x39
#define PM_REGISTER_READLOWVOLTS        0x3A
#define PM_REGISTER_READLOWVOLTSTIME    0x3B
#define PM_REGISTER_READHIVOLTS         0x3C
#define PM_REGISTER_READHIVOLTSTIME     0x3D
#define PM_REGISTER_READBUSVOLTS        0x3E


// a class to manage the fram data buffer
class FRAMSTORAGE 
{
  private:
    // the actual eeprom gets stored here
    // the struct is overlayed onto this storage

    typedef struct  
    {
    uint32_t ts;                             // Timestamp for when this record was created (4 bytes)
    uint8_t  array[4];                       // Data for this address
    uint16_t raw;                            // raw data storage (adc raw)
    } eedata;

    const int eedata_size = sizeof(eedata);  // save constant for size of eedata structure

    union EERECORD                           // create union that converts custom structure into byte array
    {
    eedata  data;                            // userland data
    uint8_t byteArray[10];                   // byte array to send over i2c or store in fram
    } ;

    const uint16_t ee_buffer_size = 70;      // number of recordds in the buffer array
    const uint16_t ee_record_size = sizeof(EERECORD);
    const uint16_t ee_start_byte  = 0x64;    // eeprom offset is 100 bytes (0x64), save that space for other uses

    union longArray
    {
    int32_t longNumber=0;
    uint8_t byteArray[4];
    };

    union ulongArray
    {
    uint32_t longNumber=0;
    uint8_t  byteArray[4];
    };

    union doubleArray
    {
    double doubleVal;
    uint8_t byteArray[4];
    };    
 
    EERECORD fram_buffer[70];

    I2C_eeprom *ptr;

    uint8_t  deviceAddress;
    uint32_t deviceSize;

  public:
    FRAMSTORAGE();                                                       // empty constructor 

    void begin (I2C_eeprom &_ee_fram);                                   // setup fram connection
    void load ();                                                        // fills buffer from fram
    void save ();                                                        // writes buffer to fram

    void addArrayData(uint8_t dataAddr, uint8_t * byteArray);            // update array with a raw byte array (from eeprom really)

    void addDouble   (uint8_t dataAddr, uint32_t ts, double doubleVal);  // update array with a double from userland
    void addUInt     (uint8_t dataAddr, uint32_t ts, uint32_t uintVal);  // update array with a unsigned int from userland
    void addSInt     (uint8_t dataAddr, uint32_t ts, int32_t intVal);    // update array with a signed integer from userland
    void addByte     (uint8_t dataAddr, uint32_t ts, uint8_t byteVal);   // update array with a single byte from userland
    void addRaw      (uint8_t dataAddr, uint32_t ts, uint16_t rawVal); // update array with raw adc data

    uint32_t  getTimeStamp  (uint8_t dataAddr);                          // get the timestamp for the record
    uint8_t * getByteArray  (uint8_t dataAddr);                          // return the raw byte array
    uint16_t  getRaw        (uint8_t dataAddr);                          // get raw adc data
    uint32_t  getDataUInt   (uint8_t dataAddr);                          // transform byte array into usigned int
    double    getDataDouble (uint8_t dataAddr);                          // transform byte array into double precision (float)
    int32_t   getDataSInt   (uint8_t dataAddr);                          // transform byte array into signed int
    uint8_t   getDataByte   (uint8_t dataAddr);                          // read one byte from byte array
};


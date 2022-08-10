#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <I2C_eeprom.h>

#define PM_REGISTER_HIGHCURRENTLIMIT    0x21 // read / write high current cut off register
#define PM_REGISTER_HIGHTEMPLIMIT       0x22 // read / write high temperature cut off register
#define PM_REGISTER_LOWTEMPLIMIT        0x23 // read / write low temperature cut off register
#define PM_REGISTER_HIGHVOLTLIMIT       0x24 // read / write high voltage cut off register
#define PM_REGISTER_LOWVOLTLIMIT        0x25 // read / writte low voltage cut off register
#define PM_REGISTER_CONFIGB0YTE         0x26 // read / write concig0 register
#define PM_REGISTER_CONFIG1BYTE         0x27 // read / write config1 register
#define PM_REGISTER_CONFIG2BYTE         0x28 // read / write config2 register
#define PM_REGISTER_STATUS0BYTE         0x2C // read status0 register byte
#define PM_REGISTER_STATUS1BYTE         0x2D // read status1 register byte
#define PM_REGISTER_CLEARCOLCNTR        0x30 // clear coulomb counter
#define PM_REGISTER_READCOLCNTR         0x31 // read coulomb counter
#define PM_REGISTER_CLEARAMPSCNTRS      0x32 // clear amperage counters
#define PM_REGISTER_READLOADAMPS        0x33 // display current load amperage
#define PM_REGISTER_TOTALAMPSIN         0x34 // total amps in
#define PM_REGISTER_TOTALAMPSOUT        0x35 // total amps out
#define PM_REGISTER_LIFEAMPSIN          0x36 // display lifetime total charge amps
#define PM_REGISTER_LIFEAMPSOUT         0x37 // display lifetime total discharge amps
#define PM_REGISTER_CLEARVOLTMEM        0x38 // clear voltage record memory
#define PM_REGISTER_READPACKVOLTS       0x39 // read pack voltage right now
#define PM_REGISTER_READLOWVOLTS        0x3A // read low pack voltage record
#define PM_REGISTER_READLOWVOLTSTIME    0x3B // read low pack voltage record timestamp 
#define PM_REGISTER_READHIVOLTS         0x3C // read high pack voltage record
#define PM_REGISTER_READHIVOLTSTIME     0x3D // read high pack voltage record timestamp
#define PM_REGISTER_READBUSVOLTS        0x3E // read bus voltage right now
#define PM_REGISTER_CLEARTEMPS          0x40 // clear temperature record memory
#define PM_REGISTER_READDEGCT0          0x41 // read t0 degrees c
#define PM_REGISTER_READDEGCT1          0x42 // read t1 degrees c
#define PM_REGISTER_READDEGCT2          0x43 // read t2 degrees c
#define PM_REGISTER_READT0LOW           0x44 // read t0 low temp record
#define PM_REGISTER_READT1LOW           0x45 // read t1 low temp record
#define PM_REGISTER_READT2LOW           0x46 // read t2 low temp record
#define PM_REGISTER_READT0HIGH          0x47 // read t0 high temp record
#define PM_REGISTER_READT1HIGH          0x48 // read t1 high temp record
#define PM_REGISTER_READT2HIGH          0x49 // read t2 high temp record
#define PM_REGISTER_READT0LOWTS         0x4A // read t0 low temp record timestamp
#define PM_REGISTER_READT1LOWTS         0x4B // read t1 low temp record timestamp
#define PM_REGISTER_READT2LOWTS         0x4C // read t2 low temp record timestamp
#define PM_REGISTER_READT0HITS          0x4D // read t0 high temp record timestamp
#define PM_REGISTER_READT1HITS          0x4E // read t1 high temp record timestamp
#define PM_REGISTER_READT2HITS          0x4F // read t2 high temp record timestamp
#define PM_REGISTER_CLEARDISCHIST       0x50 // clear disconnect history
#define PM_REGISTER_TOTOVRCURDISC       0x51 // total over current disconnects
#define PM_REGISTER_TOTUNDRVLTDISC      0x52 // total under voltage disconnects
#define PM_REGISTER_TOTOVRVLTDISC       0x53 // total over voltage disconnects
#define PM_REGISTER_TOTLOWRTEMPDISC     0x54 // total low temp disconnects
#define PM_REGISTER_TOTHITEMPDISC       0x55 // total high temp disconnects
#define PM_REGISTER_LASTDISCTIME        0x56 // timestatmp for last disconnect
#define PM_REGISTER_LASTDISCREASON      0x57 // last disconnect reason code
#define PM_REGISTER_SETEPOCHTIME        0x60 // set epoch time
#define PM_REGISTER_FIRSTINITTIME       0x61 // timestamp of last eeprom initilization
#define PM_REGISTER_CURRENTTIME         0x62 // read current epoch time
#define PM_REGISTER_TIMESYNC            0x63 // elapsed time since last sync
#define PM_REGISTER_UPTIME              0x64 // elapsed time since last power-on reset






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


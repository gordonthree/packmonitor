#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include <TimeLib.h>
#include <I2C_eeprom.h>

#include "NAU7802.h"
#include "pm_defs.h"
#include "pm_pins.h"
#include "pm_struct.h"
#include "pm_fram.h"

volatile bool unknownCmd       = false;                  // flag indicating unknown command received
volatile bool txtmsgWaiting    = false;                  // flag indicating message from Host is waiting
volatile bool reqEvnt          = false;                  // flag set when the requestEvent ISR fires
volatile bool recvEvnt         = false;                  // flag set when the receiveEvent ISR fires
volatile bool HostsetTime      = false;                  // flag that is set when Host has sent time
volatile bool _I2C_DATA_RDY    = false;                  // flag that is set when data is ready to send to Host
volatile bool _I2C_CMD_RECV    = false;                  // flag that is set when host sends a command
volatile bool purgeTXBuffer    = true;                   // tell loop() to clear the TX buffer
volatile bool dumpEEprom       = false;                  // tell loop to print contents of eeprom buffer to serial
volatile char txtMessage[50];                            // alternate buffer for message from Host

volatile uint8_t messageLen     = 0;                      // message from Host length
volatile time_t  lasttimeSync   = 0;                      // when's the last time Host sent us time?
volatile time_t  firsttimeSync  = 0;                      // record the timestamp after boot

volatile DEBUG_MSGS dbgMsgs[50];                         // room for messages from inside isr to be printed outside
volatile uint8_t    dbgMsgCnt  = 0;

// volatile I2C_RX_DATA rxData;
volatile I2C_TX_DATA txData;
volatile ADC_DATA    adcDataBuffer[adcBufferSize];  // Enough room to store three adc readings

NAU7802     extAdc;                                      // NAU7802 ADC device
FRAMSTORAGE fram;                                        // access the array for storing eeprom contents
I2C_eeprom  ee_fram(0x50, I2C_DEVICESIZE_24LC64);        // setup the eeprom here in the global scope

char buff[200];                                          // temporary buffer for working with char strings
int I2C_CLIENT_ADDR = 0x34;                              // base address, modified by pins PF0 / PF2
bool FirstRun = false;

void      scanI2C            (void);                                // scan local client bus
void      receiveEvent       (size_t howMany);                      // triggered after address match with write bit set
void      requestEvent       (void) ;                               // triggered after address match with read bit set
void      clearTXBuffer      (void);                                // reset transmit buffer to null
void      updateIload        (uint8_t reg);                         // read external adc channel, store results in buffer
void      updateVpack        (uint8_t chan, uint8_t reg);           // read external adc channel, store results in buffer
int32_t   getLong            (uint8_t * byteArray);                 // convert byte array to signed int
uint32_t  getULong           (uint8_t * byteArray);                 // convert byte array to unsigned int
double    getDouble          (uint8_t * byteArray);                 // convert byte array to double
uint32_t  readADC            (uint8_t adcPin, uint8_t noSamples);   // return value from internal ADC

#if defined(MCU_AVR128DA32)
  //HardwareI2C &i2c_host = TWI1;
  TwoWire &i2c_host = TWI1;
#endif

void setup() {
  pinMode(PIN_PF0, INPUT_PULLUP); // pins for client address configuration
  pinMode(PIN_PF1, INPUT_PULLUP);

  bool addr0 = digitalRead(PIN_PF0); // see what our hardwired address is
  bool addr1 = digitalRead(PIN_PF1);

  #ifndef PM_CLIENT_ADDRESS
  I2C_CLIENT_ADDR |= (addr0 | (addr1<<1)); // compute address
  #else
  I2C_CLIENT_ADDR = PM_CLIENT_ADDRESS;     // use what was sent during compile
  #endif

  // initialize LEDs outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // init ADC pins
  pinMode(ADC0, INPUT);
  pinMode(ADC1, INPUT);
  pinMode(ADC2, INPUT);
  pinMode(ADC3, INPUT);

  // init Serial port 1
  Serial1.begin(115200); 

  delay(2000);
  Serial1.println("\n\nHello, world!");
  
  // setup i2c bus(es) dedpending on what chip we are compiling for
  #if defined(MCU_AVR128DA32)                  // Setup both TWO0 and TWI1
    i2c_host.begin();                          // Host on TWI1, default pins SDA PF2, SCL PF3
    i2c_host.setClock(100000);                 // bus speed 100khz
    Wire.pins(PIN_PA2, PIN_PA3);
    Wire.begin(I2C_CLIENT_ADDR, false);        // Client on TWI0, default pins, SDA PA2, SCL PA3
    Serial1.printf("Client address: 0x%X\nUsing twi0 and twi1\n", I2C_CLIENT_ADDR);
  #elif defined(MCU_AVR128DA28)                // Setup TWI0 for dual mode ... TWI_MANDS_SINGLE
    Wire.enableDualMode(false);                // enable fmp+ is false
    Wire.begin();                              // setup host default pins SDA PA2, SCL PA3
    Wire.begin(I2C_CLIENT_ADDR, false);        // setup client with address, ignore broadcast, default pins SDA PC2, SCL PC3
    Wire.setClock(100000);                     // bus speed 100khz
    Serial1.printf("Client address: 0x%X\nDUALCTRL register: 0x%X\n", I2C_CLIENT_ADDR, TWI0_DUALCTRL);
  #else
    Wire.begin(I2C_CLIENT_ADDR);               // client only for some reason
    Serial1.printf("Client address: 0x%X\nClient-only mode\n", I2C_CLIENT_ADDR);
  #endif

  delay(2000);

  Serial1.flush();

  scanI2C();  // scan bus?!

  Wire.onRequest(&requestEvent); // register requestEvent interrupt handler
  Wire.onReceive(&receiveEvent); // register receiveEvent interrupt handler

  extAdc.begin(0x2A, Wire);      // pass through TWI to adc library
  extAdc.avcc4V5();              // set voltage regulator and analog reference to internal at 4.5v
  extAdc.rate010sps();           // set conversion rate to 320sps

  Serial1.print("Loading data from FRAM... ");

  fram.begin(ee_fram);

  Serial1.print("complete.\n");

  #ifdef PM_FIRSTRUN
    FirstRun = true;
  #endif
}

uint16_t      i=0;
uint16_t iFive = 0;
uint16_t      x=0;
uint8_t       ledX=0;
uint8_t       adcUpdateCnt      = 0;
const uint8_t adcUpdateInterval = 20;
uint32_t      rawAdc            = 0;
uint32_t      timeStamp         = 0;
double        rawDouble         = 0.0;

// the loop function runs over and over again forever
void loop() {
  i++;
  iFive++;
  timeStamp = now();
  adcUpdateCnt++;

  digitalWrite(LED2, reqEvnt);
  digitalWrite(LED3, recvEvnt);
  digitalWrite(LED4, HostsetTime);

  if (dumpEEprom) {
    Serial1.println("Dumping eeprom contents!");
    byte xx=0x21; // start here
    while (xx<0x65) 
    {
      Serial1.printf("Addr: 0x%X TS: %lu Double: %f UINT: %lu SINT: %li RAW: %u\n",
        xx,
        fram.getTimeStamp(xx), 
        fram.getDataDouble(xx),
        fram.getDataUInt(xx),
        fram.getDataSInt(xx),
        fram.getRaw(xx));
      xx++;
    }
    Serial1.println("Complete.");
    dumpEEprom = false;
  }

  if (timeSet && FirstRun) {
    fram.addUInt(PM_REGISTER_FIRSTINITTIME, now(), now());
    FirstRun = false;
  }

  if (purgeTXBuffer) clearTXBuffer(); 

  if (adcUpdateCnt > adcUpdateInterval) {     
    // read and store temperature data in the FRAM buffer using internal ADC
    rawAdc = readADC(ADC1, 20); // update in-memory value for internal adc1
    fram.addRaw(PM_REGISTER_READDEGCT0, timeStamp, rawAdc);                      // store unprocessed raw value
    fram.addDouble(PM_REGISTER_READDEGCT0, timeStamp, raw2temp(rawAdc));         // use LUT to find approximate temperature
    rawAdc = readADC(ADC2, 20); // update in-memory value for internal adc2
    fram.addRaw(PM_REGISTER_READDEGCT1, timeStamp, rawAdc);                      // store unprocessed raw value
    fram.addDouble(PM_REGISTER_READDEGCT1, timeStamp, raw2temp(rawAdc));         // use LUT to find approximate temperature
    rawAdc = readADC(ADC3, 20); // update in-memory value for internal adc3
    fram.addRaw(PM_REGISTER_READDEGCT2, timeStamp, rawAdc);                      // store unprocessed raw value 
    fram.addDouble(PM_REGISTER_READDEGCT2, timeStamp, raw2temp(rawAdc));         // use LUT to find approximate temperature
    rawAdc = readADC(ADC0, 20); // update in-memory value for internal adc0
    fram.addRaw(PM_REGISTER_READBUSVOLTS, timeStamp, rawAdc);                    // store unprocessed raw value
    fram.addDouble(PM_REGISTER_READBUSVOLTS, timeStamp, raw2volts(rawAdc, 1.0)); // calculate volts from raw value and divider (1.0)
    // Serial1.println("");
    // Serial1.printf("ADC 1: %.2f ADC 2: %.2f ADC 3: %.2f ADC 0: %.2f\n", 
    //   fram.getDataDouble(PM_REGISTER_READDEGCT0),
    //   fram.getDataDouble(PM_REGISTER_READDEGCT1),
    //   fram.getDataDouble(PM_REGISTER_READDEGCT2),
    //   fram.getDataDouble(PM_REGISTER_READBUSVOLTS));
    // Serial1.printf("ADC 1: %u ADC 2: %u ADC 3: %u ADC 0: %u\n", 
    //   fram.getRaw(PM_REGISTER_READDEGCT0),
    //   fram.getRaw(PM_REGISTER_READDEGCT1),
    //   fram.getRaw(PM_REGISTER_READDEGCT2),
    //   fram.getRaw(PM_REGISTER_READBUSVOLTS));

    updateExtAdc(1, PM_REGISTER_READPACKVOLTS);
    updateExtAdc(2, PM_REGISTER_READLOADAMPS);

    adcUpdateCnt = 0; // reset counter for adc update delay
  }
    
  // debug print
  // Serial1.printf("extADC 1: %f (%u) ADC 2: %f (%u)\n", 
  //   fram.getDataDouble(PM_REGISTER_READPACKVOLTS)/1000.0,
  //   fram.getRaw(PM_REGISTER_READPACKVOLTS),
  //   fram.getDataDouble(PM_REGISTER_READLOADAMPS)/1000.0,
  //   fram.getRaw(PM_REGISTER_READLOADAMPS));

  
  if (txtmsgWaiting) {            // print message sent by Host
    txtmsgWaiting = false;        // clear flag
    sprintf(buff, "Message from Host: %s", txtMessage);
    Serial1.println(buff);
  }

  if (i>1000) {                   // fires roughly every 1 second
    i=0;
    ledX = ledX ^ 1;              // xor previous state
    digitalWrite(LED1, ledX);     // turn the LED on and off to show program running
    
    recvEvnt = false; // reset flag
    reqEvnt  = false; // reset flag

    // update uptime if clock has been set
    if (firsttimeSync) fram.addUInt(PM_REGISTER_UPTIME, timeStamp, timeStamp - firsttimeSync);

    // update last time sync if needed
    if (fram.getDataUInt(PM_REGISTER_TIMESYNC)!=lasttimeSync) fram.addUInt(PM_REGISTER_TIMESYNC, timeStamp, lasttimeSync);
  }

  if (iFive>5000) {               // roughly every 5 seconds
    fram.save();                    // write memory cache to fram for backup
    iFive = 0;
  }

  if (dbgMsgCnt>0) { // display any debug messages generated inside the ISRs
    for (int ptr=0; ptr < dbgMsgCnt; ptr++) {
      Serial1.printf("Diag message %u: %s\n", ptr + 1, dbgMsgs[ptr].messageTxt);
    }
    Serial1.println("");
    dbgMsgCnt = 0;
  }

  delay(1);
}

// function that executes whenever data is received from Host
// this function is registered as an event, see setup()
void receiveEvent(size_t howMany) {
  const uint8_t    _isr_dataSize    = 4;             // constant size for byte array to/from i2c, or to/from fram
  uint8_t          _isr_HostByte    = 0;             // byte size data 8 bits
  uint32_t         _isr_HostUlong   = 0;             // unsigned long integer 32 bits
  int32_t          _isr_HostLong    = 0;             // signed long integer 32 bits
  double           _isr_HostDouble  = 0.0;           // double precision aka float, 32 bits
  uint32_t         _isr_timeStamp   = now();         // current timestamp
  
  uint8_t          _isr_cmdAddr     = 0;             // command / register address sent by host
  uint8_t          _isr_dataLen     = 0;             // number of data bytes sent by host
  uint8_t          _isr_byteArray[4];                // storage for four bytes of data of course!
  uint8_t          _isr_cmdData[64];                 // storage for incoming data from host
  union ulongArray  ulongbuffer;                     // convert between byte array and ulong int

  _isr_cmdAddr                      = Wire.read();   // read first byte, store it as command address
  recvEvnt                          = true;          // set flag to toggle LED in loop()
  txData.dataLen                    = _isr_dataSize; // set data length here just in case I forgot later
  _I2C_DATA_RDY                     = false;         // nothing to send yet

  while (Wire.available()) {
    _isr_HostByte = (uint8_t) Wire.read();
    // if (_isr_HostByte != 0xFF) 
    _isr_cmdData[_isr_dataLen] = _isr_HostByte; // keep reading until no more bytes available
    _isr_dataLen++;
    // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "RX cmd 0x%X data 0x%X", _isr_cmdAddr, _isr_HostByte);
    // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
    // dbgMsgCnt++;                                                        // increment debug message counter

  }

  // rxData.dataLen           = _isr_dataLen;      // store bytes sent inside struct
  // rxData.cmdAddr           = _isr_cmdAddr;      // store command byte inside struct

  // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "RX cmd 0x%X plus %u data bytes", _isr_cmdAddr, _isr_dataLen);
  // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
  // dbgMsgCnt++;                                                        // increment debug message counter
  
  // events that store or return the same data can be stacked like seen below, 
  // the break command stops the current event
  switch  (_isr_cmdAddr) {
    case 0x21: // get / set high current limit, double
    case 0x22: // get / set high-temp limit, double
    case 0x23: // get / set low-temp limit, double
    case 0x24: // get / set high-voltage limit, double
    case 0x25: // get / set low-voltage limit, double
      if (_isr_dataLen>0) // check if this is a read or write?
      // more than 0 bytes available, this is a write
      {
        _isr_HostDouble = getDouble(_isr_cmdData);                        // convert byte array into double
        fram.addDouble(_isr_cmdAddr, _isr_timeStamp, _isr_HostDouble);    // store a double in memory buffer
      }
      else
      // no data was sent, this is a read
      {
        memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
        txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
        _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      }
      break;

    case 0x26: // get / set config0, byte
    case 0x27: // get / set config1, byte
    case 0x28: // get / set config2, byte
      if (_isr_dataLen>0) // check if this is a read or write?
      // more than 0 bytes available, this is a write
      {
        _isr_HostByte = _isr_cmdData[0];                         // single byte!
        fram.addByte(_isr_cmdAddr, _isr_timeStamp, _isr_HostByte);  // save to buffer
      }
      else
      {
        txData.cmdData[0] = fram.getDataByte(_isr_cmdAddr);         // read single byte from buffer
        txData.dataLen = 1;                                         // single byte to send
        _I2C_DATA_RDY = true;                                       // let loop know data is ready
      }
      break;
    case 0x29: // r/w current sensor mva, double
    case 0x2A: // r/w pack voltage divisor, double
    case 0x2B: // r/w bus voltage divisor, double
    case 0x2E: // r/w thermistor scaling value, double
      if (_isr_dataLen>0) // check if this is a read or write?
      // more than 0 bytes available, this is a write
      {
        _isr_HostDouble = getDouble(_isr_cmdData);                        // convert host byte array into double
        fram.addDouble(_isr_cmdAddr, _isr_timeStamp, _isr_HostDouble);    // store double in memory buffer
      }
      else
      // no data was sent, this is a read
      {
        memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
        txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
        _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      }
      break;
    case 0x2C: // read status1, byte (read only)
    case 0x2D: // read status2, byte (read only)
      txData.cmdData[0] = fram.getDataByte(_isr_cmdAddr);         // read single byte from buffer
      txData.dataLen    = 1;                                      // single byte to send
      _I2C_DATA_RDY = true;                                       // let loop know data is ready
      break;

    case 0x2E: // diagnostic turn off LED4
      {
        digitalWrite(LED4, LOW);                            // turn off LED4
      }
      break;
    case 0x2F: // diagnostic turn on LED4
      {
        digitalWrite(LED4, HIGH);                           // turn on LED4
      }
      break;

    case 0x30: // clear coulomb-counter (write only)
      fram.addDouble(_isr_cmdAddr, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x31: // read coulomb counter, double (read only)
      memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x32: // clear total amps counter (cmds 33, 34) (write only)
      fram.addDouble(0x33, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x34, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x33: // read instant amps, double (read only)
    case 0x34: // read total amps in counter, double (read only)
    case 0x35: // read total amps out counter, double (read only)
    case 0x36: // read lifetime amps in, double (read only)
    case 0x37: // read lifetime amps out, double (read only)
      memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x38: // clear voltage memory, (write only)
      fram.addDouble(0x3A, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x3B, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x3C, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x3D, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x39: // read instant pack voltage, double (read only)
    case 0x3A: // read lowest voltage memory, double (read only)
    case 0x3B: // read lowest voltage timestamp, double (read only)
    case 0x3C: // read highest voltage memory, double (read only)
    case 0x3D: // read highest voltage timestamp, double (read only)
    case 0x3E: // read instant bus voltage, double (read only)
      memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x40: // clear temperature memories, (write only)
      fram.addDouble(0x42, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x43, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x44, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x45, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x46, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x47, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x48, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x49, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x41: // read t0 instant, double (read only)
    case 0x42: // read t1 instant, double (read only)
    case 0x43: // read t2 instant, double (read only)
    case 0x44: // read t0 lowest, double (read only)
    case 0x45: // read t1 lowest, double (read only)
    case 0x46: // read t2 lowest, double (read only)
    case 0x47: // read t0 highest, double (read only)
    case 0x48: // read t1 highest, double (read only)
    case 0x49: // read t2 highest, double (read only)
      // dblbuffer.doubleVal = fram.getDataDouble(_isr_cmdAddr);
      // memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "TX %f", dblbuffer.doubleVal);
      // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
      // dbgMsgCnt++;
      // txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      // break;
    case 0x4A: // read t0 low timestamp, unsigned long (read only)
    case 0x4B: // read t1 low timestamp, unsigned long (read only)
    case 0x4C: // read t2 low timestamp, unsigned long (read only)
    case 0x4D: // read t0 high timestamp, unsigned long (read only)
    case 0x4E: // read t1 high timestamp, unsigned long (read only)
    case 0x4F: // read t1 high timestamp, unsigned long (read only)
      // ulongbuffer.longNumber = fram.getDataUInt(_isr_cmdAddr);
      memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x50: // clear disconnect history (write only)
      fram.addUInt(0x51, _isr_timeStamp, 0UL);
      fram.addUInt(0x52, _isr_timeStamp, 0UL);
      fram.addUInt(0x53, _isr_timeStamp, 0UL);
      fram.addUInt(0x54, _isr_timeStamp, 0UL);
      fram.addUInt(0x55, _isr_timeStamp, 0UL);
      fram.addUInt(0x56, _isr_timeStamp, 0UL);
      fram.addUInt(0x57, _isr_timeStamp, 0UL);
      break;
    
    case 0x51: // read total over-current disconnects, unsigned int (read only)
    case 0x52: // read total under-voltage discon, unsigned int (read only)
    case 0x53: // read total over-volt discon, uint (read only)
    case 0x54: // read total under-temp discon, uint (read only)
    case 0x55: // read total over-temp discon, uint (read only)
    case 0x56: // read last discon timestamp, ulong (read only)
      memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x57: // read last discon reason code, byte (read only)
      txData.cmdData[0] = fram.getDataByte(_isr_cmdAddr);         // read single byte from buffer
      txData.dataLen    = 1;                                      // single byte to send
      _I2C_DATA_RDY     = true;                                   // let loop know data is ready
      break;

    case 0x60: // set time from Host (write only)
      memcpy(ulongbuffer.byteArray, _isr_cmdData, 4);
      _isr_timeStamp = ulongbuffer.longNumber;

      if (_isr_timeStamp>1000000000) {
        setTime(_isr_timeStamp);                            // fingers crossed
        HostsetTime = true;                                 // set flag
        lasttimeSync = _isr_timeStamp;                      // record timestamp of sync
        if (!firsttimeSync) firsttimeSync = _isr_timeStamp; // if it's our first sync, record in separate variable
        // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Timestamp %lu", _isr_timeStamp);
        // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        // dbgMsgCnt++;
      } else {
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Error decoding timestamp! %lu", _isr_timeStamp);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;
      }
      break;

    case 0x61: // read first-init timestamp, ulong (read only)
      memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;
    case 0x62: // read current timestamp, ulong (read only)
      ulongbuffer.longNumber = _isr_timeStamp;
      memcpy(txData.cmdData, ulongbuffer.byteArray, _isr_dataSize);          // convert to byte array and copy to tx buffer
      txData.dataLen = 4;                                               // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                             // let loop know data is ready
      break;
    case 0x63: // read time since last sync, ulong (read only)
      ulongbuffer.longNumber = lasttimeSync;
      memcpy(txData.cmdData, ulongbuffer.byteArray, _isr_dataSize);          // convert to byte array and copy to tx buffer
      txData.dataLen = 4;                                               // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                             // let loop know data is ready
      break;
    case 0x64: // read time since last sync, ulong (read only)
      ulongbuffer.longNumber = lasttimeSync;
      memcpy(txData.cmdData, ulongbuffer.byteArray, _isr_dataSize);          // convert to byte array and copy to tx buffer
      txData.dataLen = 4;                                               // tell requestEvent to send this many bytes
      _I2C_DATA_RDY = true;                                             // let loop know data is ready
      break;
    case 0x77: // dump eeprom
      dumpEEprom = true;
      break;
    default:// unknown command
      {
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Unknown command ignored: cmd 0x%X", _isr_cmdAddr);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;                                                        // increment debug message counter
      }
      break;
  } // end switch
  // sprintf(buff, "Receive event triggered. Command 0x%X", rxData.cmdAddr);
  // Serial1.println(buff);
  // purgeRXBuffer = true; // ask main loop() to purge buffer
} // end of handleEvent

// function that executes whenever data is requested by Host
// this function is registered as an event, see setup()
void requestEvent() {   
  // char reqBuff[80];                          // Host has requested data
  reqEvnt = true;   
  // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "TX event; i2c_data_rdy=0x%X and %u data bytes", _I2C_DATA_RDY, txData.dataLen);
  // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
  // dbgMsgCnt++;                                                        // increment debug message counter

  if (_I2C_DATA_RDY) {
    _I2C_DATA_RDY = false;                              // set flag that we had this interaction
    Wire.write((const uint8_t *) txData.cmdData, txData.dataLen);
  } else {
    sprintf((char) txData.cmdData,"Client 0x%X ready!", I2C_CLIENT_ADDR);
    Wire.write((char) txData.cmdData);                         // didn't have anything to send? respond with message of 6 bytes
  }
  purgeTXBuffer=true;                                // purge TX buffer
}

void scanI2C() {
  byte error, address;
  int nDevices;

  Serial1.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial1.print("I2C device found at address 0x");
      if (address<16) 
      Serial1.print("0");
      Serial1.print(address, HEX);
      Serial1.println(" !");

      nDevices++;
    }
     else if (error==4) 
    {
      Serial1.print("Unknow error at address 0x");
      if (address<16) 
      Serial1.print("0");
      Serial1.println(address, HEX);
    } 
  }
  
  if (nDevices == 0)
    Serial1.println("No I2C devices found");
  else
    Serial1.println("done.");
 
} // end of scani2c

uint32_t readADC(uint8_t adcPin, uint8_t noSamples) {
  uint32_t adcResult = 0;
  int      sample    = 0;
  uint8_t  adcX      = 0;

  for (int x=0; x < noSamples; x++) {
    sample = analogRead(adcPin);                    // sample adc pin
    adcResult = adcResult + sample;                 // add sample for averaging
    delayMicroseconds(250);
  }
  
  adcResult = adcResult / noSamples;
  
  return (long) adcResult;
}

void clearTXBuffer() {
  uint16_t myPtr = 0;
  while (myPtr < txBufferSize) {
    txData.cmdData[myPtr] = '\0';
    myPtr++;
  }
  purgeTXBuffer = false;
}

double getDouble(uint8_t * byteArray)
{
  union doubleArray buffer;
  const uint8_t dataLen = 4;
  memcpy(buffer.byteArray, byteArray, dataLen);

  return buffer.doubleVal;
}

uint32_t getULong(uint8_t * byteArray)
{
  union ulongArray buffer;
  const uint8_t dataLen = 4;
  memcpy(buffer.byteArray, byteArray, dataLen);

  return buffer.longNumber;
}

int32_t getLong(uint8_t * byteArray)
{
  union ulongArray buffer;
  const uint8_t dataLen = 4;
  memcpy(buffer.byteArray, byteArray, dataLen);

  return buffer.longNumber;
}

void updateVpack(uint8_t chan, uint8_t reg)                          // read external adc channel, store results in buffer
{
    uint32_t tS = now();
    uint32_t rawAdc = 0;
    double rawDouble = 0.0;
    double vDiv = fram.getDataDouble(PM_REGISTER_VPACKDIVISOR);

    extAdc.selectCh2();                                              // vpack divider on adc ch2
    rawAdc    = extAdc.readADC();                                    // get raw 24-bit value
    rawDouble = extAdc.readmV() / vDiv;                              // get voltage as double
    
    fram.addRaw(reg, tS, rawAdc);                                    // store raw value in buffer
    fram.addDouble(reg, tS, rawDouble);                              // store processed value in buffer
}

void updateIload(uint8_t reg)                                        // read external adc channel, store results in buffer
{
    uint32_t tS = now();
    uint32_t rawAdc = 0;
    double rawDouble = 0.0;
    double vDiv = fram.getDataDouble(PM_REGISTER_VPACKDIVISOR);
    
    extAdc.selectCh1();                                               // current sensor on adc ch1
    rawAdc    = extAdc.readADC();                                     // get raw 24-bit value
    rawDouble = extAdc.readmV() / vDiv;                               // get voltage as double
    
    fram.addRaw(reg, tS, rawAdc);        // store raw value in buffer
    fram.addDouble(reg, tS, raw2amps rawDouble);  // store processed value in buffer
}

void checkParameters
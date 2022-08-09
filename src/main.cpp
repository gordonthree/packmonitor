#include <Arduino.h>
#include <Wire.h>
#include <packmonlib.h>
#include <time.h>
#include <TimeLib.h>
#include <I2C_eeprom.h>
#include <NAU7802.h>

#include "pm_pins.h"
#include "pm_struct.h"

volatile bool unknownCmd       = false;                  // flag indicating unknown command received
volatile bool txtmsgWaiting    = false;                  // flag indicating message from Host is waiting
volatile bool reqEvnt          = false;                  // flag set when the requestEvent ISR fires
volatile bool recvEvnt         = false;                  // flag set when the receiveEvent ISR fires
volatile bool HostsetTime      = false;                  // flag that is set when Host has sent time
volatile bool _I2C_DATA_RDY    = false;                  // flag that is set when data is ready to send to Host
volatile bool _I2C_CMD_RECV    = false;                  // flag that is set when host sends a command
volatile bool purgeTXBuffer    = true;                   // tell loop() to clear the TX buffer
volatile bool purgeRXBuffer    = true;                   // tell loop() to clear the RX buffer
volatile char txtMessage[50];                            // alternate buffer for message from Host

volatile uint8_t messageLen    = 0;                      // message from Host length
volatile time_t lasttimeSync   = 0;                      // when's the last time Host sent us time?
volatile time_t firsttimeSync  = 0;                      // record the timestamp after boot

volatile DEBUG_MSGS dbgMsgs[10];                         // room for messages from inside isr to be printed outside
volatile uint8_t    dbgMsgCnt  = 0;

volatile I2C_RX_DATA rxData;
volatile I2C_TX_DATA txData;
volatile ADC_DATA    adcDataBuffer[adcBufferSize];  // Enough room to store three adc readings

PackMonLib  toolbox();                                     // collection of routines used by both client and host applications
NAU7802     extAdc();                                      // NAU7802 ADC device
I2C_eeprom  fram(0x50, I2C_DEVICESIZE_24LC64);             // ferro-electric memory baby!

char buff[200];
int I2C_CLIENT_ADDR = 0x34;                                 // base address, modified by pins PF0 / PF2

void scanI2C(void);

void receiveEvent(size_t howMany);


uint16_t framLookupAddr     (uint8_t  cmdAddress);
uint32_t framReadUlong      (uint16_t dataAddress);
int32_t  framReadInt        (uint16_t dataAddress);
uint8_t  framReadByte       (uint16_t dataAddress);
double   framReadDouble     (uint16_t dataAddress);

void     framWriteUlong     (uint16_t dataAddress, uint32_t framData);
void     framWriteInt       (uint16_t dataAddress, int32_t  framData);
void     framWriteByte      (uint16_t dataAddress, uint8_t  framData);
void     framWriteDouble    (uint16_t dataAddress, double   framData);

int32_t  getLong            (uint8_t * byteArray);
uint32_t getULong           (uint8_t * byteArray);
double   getDouble          (uint8_t * byteArray);

long readADC(uint8_t adcPin, uint8_t noSamples);

void clearTXBuffer(void);

void clearRXBuffer(void);

// function that executes whenever data is requested by Host
// this function is registered as an event, see setup()
void requestEvent(void) ;

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

  // init f-ram eeprom
  fram.begin();

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
}

uint16_t      i=0;
uint16_t      x=0;
uint8_t       ledX=0;
uint8_t       adcUpdateCnt      = 0;
const uint8_t adcUpdateInterval = 20;

// the loop function runs over and over again forever
void loop() {
  i++;
  adcUpdateCnt++;

  digitalWrite(LED2, reqEvnt);
  digitalWrite(LED3, recvEvnt);
  digitalWrite(LED4, HostsetTime);

  if (purgeTXBuffer) clearTXBuffer(); 
  if (purgeRXBuffer) clearRXBuffer();

  if (adcUpdateCnt > adcUpdateInterval) {
    long rawAdc    = 0;
    //int acsOffset  = 514;
    float acsmvA  = 0.136;  // 0.136v or 136mV per amp
    float Amps    = 0.0;
    float Volts   = 0.0;
    float sysVcc  = 5.09;
    float vDiv2   = 1.0;
    float vDiv3   = 1.0;
  
// #ifdef MCU_NANOEVERY
//     sysVcc        = 4.300;
// #endif
 
    // rawAdc = readADC(ADC0, 20);
    rawAdc = analogRead(ADC0);
    //rawAdc = rawAdc; // subtrack offset
    adcDataBuffer[0].adcRaw = rawAdc;
    Volts = (float)(rawAdc * (sysVcc / 1024.0)) - (sysVcc / 2);
    //Amps =  (float)Volts / acsmvA;
    //adcDataBuffer[0].Amps  = Amps;
    adcDataBuffer[0].Volts = Volts;
    // Serial1.print("0: ");
    // Serial1.print(Amps);
    // Serial1.print("0: ");
    // // Serial1.print(Volts, 3);
    // // Serial1.print("v Raw ");
    // Serial1.print(rawAdc);
    // // Serial1.println(" ");
    // Serial1.print(" 1: ");

    rawAdc = readADC(ADC1, 20);
    // Serial1.print(rawAdc);
    // Serial1.print(" 2: ");
    adcDataBuffer[1].adcRaw   = rawAdc;
    Volts = (float)(rawAdc * (sysVcc / 1024.0)) / vDiv2;
    adcDataBuffer[1].Volts = Volts;

    rawAdc = readADC(ADC2, 20);
    // Serial1.println(rawAdc);
    adcDataBuffer[2].adcRaw   = rawAdc;
    Volts = (float)(rawAdc * (sysVcc / 1024.0)) / vDiv3;
    adcDataBuffer[2].Volts = Volts;
    adcUpdateCnt = 0;
  }
  
  if (txtmsgWaiting) {            // print message sent by Host
    txtmsgWaiting = false;        // clear flag
    sprintf(buff, "Message from Host: %s", txtMessage);
    Serial1.println(buff);
  }

  if (i>1000){
    i=0;
    // if (timeStatus()==timeSet) {             // print timestamps once time is set
      ledX = ledX ^ 1;              // xor previous state
      digitalWrite(LED1, ledX);     // turn the LED on (HIGH is the voltage level)
      // Serial1.printf("Timestamp: %lu\n", now());
    // } 
    
    recvEvnt = false; // reset flag
    reqEvnt  = false; // reset flag
  }

  if (dbgMsgCnt>0) {
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
  uint8_t   _isr_HostByte  = 0;
  uint16_t  _isr_HostUint  = 0;
  int16_t   _isr_HostInt   = 0;
  uint32_t  _isr_HostUlong = 0;
  int32_t   _isr_HostLong  = 0;
  uint32_t  _isr_timeStamp = 0;

  uint8_t   _isr_cmdAddr   = 0;                    // command / register address sent by host
  uint8_t   _isr_dataLen   = 0;                    // number of data bytes sent by host

  _isr_cmdAddr             = Wire.read();          // read first byte, store it as command address
  recvEvnt                 = true;                 // set flag to toggle LED in loop()

  while (Wire.available()) {
    rxData.cmdData[_isr_dataLen] = Wire.read();         // keep reading until no more bytes available
    _isr_dataLen++;
  }

  rxData.dataLen           = _isr_dataLen;              // store bytes sent inside struct
  rxData.cmdAddr           = _isr_cmdAddr;         // store command byte inside struct

  sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "RX cmd 0x%X plus %u data bytes", _isr_cmdAddr, _isr_dataLen);
  dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
  dbgMsgCnt++;                                                        // increment debug message counter
  
  switch  (_isr_cmdAddr) {
    case 0x21: // high current limit, unsigned int
      {
        double   cmdData     = getDouble(rxData.cmdData);
        uint16_t dataAddress = framLookupAddr(rxData.cmdAddr);
        framWriteDouble(dataAddress, cmdData);
      }
      break; 
    case 0x22: // high-temp limit, unsigned int
      {
        _isr_HostUint = atol(rxData.cmdData);
         writeFRAMuint(rxData.cmdAddr, _isr_HostUint);
      }
      break;
    case 0x23: // low-temp limit, signed int
      {
        _isr_HostInt = atoi(rxData.cmdData);
        writeFRAMint(rxData.cmdAddr, _isr_HostInt);
      }
      break;
    case 0x24: // high-voltage limit, unsigned int
      {
        _isr_HostUint = atol(rxData.cmdData);
        writeFRAMuint(rxData.cmdAddr, _isr_HostUint);
      }
      break;
    case 0x25: // low-voltage limit, unsigned int
      {
        _isr_HostUint = atol(rxData.cmdData);
        writeFRAMuint(rxData.cmdAddr, _isr_HostUint);
      }
      break;
    case 0x26: // set config0, byte
      {
        _isr_HostByte = rxData.cmdData[0];
        writeFRAMuint(rxData.cmdAddr, _isr_HostByte);
      }
      break;
    case 0x27: // set config1, byte
      {
        _isr_HostByte = rxData.cmdData[0];
        writeFRAMuint(rxData.cmdAddr, _isr_HostByte);
      }
      break;
    case 0x28: // set config2, byte
      {
        _isr_HostByte = rxData.cmdData[0];
        writeFRAMuint(rxData.cmdAddr, _isr_HostByte);
      }
      break;
    case 0x29: // read config0, byte
      {
        _isr_HostByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_HostByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2A: // read config1, byte
      {
        _isr_HostByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_HostByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2B: // read config2, byte
      {
        _isr_HostByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_HostByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2C: // read status1, byte
      {
        _isr_HostByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_HostByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2D: // read status2, byte
      {
        _isr_HostByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_HostByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
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
    case 0x30: // clear coul-counter, no data
      { 
        writeFRAMint(0x31, 0);
      }
      break;
    case 0x31: // read coulomb counter, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x32: // clear total amps counter, no data
      { 
        writeFRAMint(0x34, 0);
        writeFRAMint(0x35, 0);
      }
      break;
    case 0x33: // read instant amps, send raw adc value, ulong
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMuint(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Sending %u", buffer.longNumber);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;
      }
      break;
    case 0x34: // read total amps in counter, unsigned long
      {
        // _isr_HostUlong = readFRAMulong(rxData.cmdAddr);
        union ulongArray buffer;
        buffer.longNumber = readFRAMfloat(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data

      }
      break;
    case 0x35: // read total amps out counter, unsigned long
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMfloat(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data

      }
      break;
    case 0x36: // read lifetime amps in, unsigned long
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMfloat(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data

      }
      break;
    case 0x37: // read lifetime amps out, ubsigned long 
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMfloat(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data

      }
      break;
    case 0x38: // clear voltage memory, no data returned
      { 
        writeFRAMint(0x3A, 0);
        writeFRAMint(0x3B, 0);
        writeFRAMint(0x3C, 0);
        writeFRAMint(0x3D, 0);
      }
      break;
    case 0x39: // read pack voltage (as raw adc value) uint32
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMuint(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Sending %u", buffer.longNumber);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;
      }
      break;
    case 0x3A: // read lowest voltage memory, unsigned int
      {
        _isr_HostUint = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_HostUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3B: // read lowest voltage timestamp, unsigned long
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMfloat(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data

        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
      }
      break;
    case 0x3C: // read highest voltage memory, unsigned int
      {
        _isr_HostUint = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_HostUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3D: // read highest voltage timestamp, unsigned long
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMfloat(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data

      }
      break;
    case 0x3E: // read bus voltage, float
      {
        // _isr_HostUint = readFRAMuint(rxData.cmdAddr);
        // ltoa(_isr_HostUint, txData.cmdData, 10);           // store data as char string in tx buffer
        // dtostrf(framData, 3, 2, txData.cmdData);
        union ulongArray buffer;
        buffer.longNumber = readFRAMuint(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Sending %u", buffer.longNumber);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;

      }
      break;

    case 0x3F: // print diag message from Host
      { 
        strncpy(txtMessage, rxData.cmdData, rxData.dataLen); // copy message into another buffer
        // messageLen = rxData.dataLen; // copy message length too
        txtmsgWaiting = true; // set flag to print the message in loop()
      }
      break;

    case 0x40: // clear temperature memories, no data
      { 
        writeFRAMint(0x42, 0);
        writeFRAMint(0x43, 0);
        writeFRAMint(0x45, 0);
        writeFRAMint(0x46, 0);
        writeFRAMint(0x47, 0);
        writeFRAMint(0x48, 0);
        writeFRAMint(0x49, 0);
        writeFRAMint(0x4A, 0);
      }
      break;
    case 0x41: // read t0 instant, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x42: // read t0 lowest, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x43: // read t0 highest, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x44: // read t1 instant, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x45: // read t1 lowest, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x46: // read t1 highest, signed int
      {
        _isr_HostInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_HostInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x47: // read t0 lowest timestamp, unsigned long
      {
        _isr_HostUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_HostUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x48: // read t0 highest timestamp, unsigned long
      {
        _isr_HostUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_HostUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x49: // read t1 lowest timestamp, unsigned long
      {
        _isr_HostUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_HostUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x4A: // read t1 highest timestamp, unsigned long
      {
        _isr_HostUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_HostUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    
    case 0x50: // clear disconnect history, no data
      { 
        writeFRAMint(0x51, 0);
        writeFRAMint(0x52, 0);
        writeFRAMint(0x53, 0);
        writeFRAMint(0x54, 0);
        writeFRAMint(0x55, 0);
        writeFRAMint(0x56, 0);
        writeFRAMint(0x57, 0);
      }
      break;
    
    case 0x51: // read total over-current disconnects, unsigned int
      {
        _isr_HostUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_HostUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x52: // read total under-voltage discon, unsigned int
      {
        _isr_HostUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_HostUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x53: // read total over-volt discon, uint
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMuint(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x54: // read total under-temp discon, uint
      {
        union ulongArray buffer;
        txData.dataLen = 4;                                 // number of bytes to transmit
        buffer.longNumber = readFRAMuint(rxData.cmdAddr);
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x55: // read total over-temp discon, uint
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMuint(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x56: // read last discon timestamp, ulong
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMulong(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x57: // read last discon reason code, byte
      {
        _isr_HostByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_HostByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x60: // set time from Host, char string
      {
        // Serial1.println((char) rxData.cmdData);
        // for (int ptr=0; ptr < 4; ptr++){
        //   sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "rxdata.cmdData[%u] = 0x%X", ptr, rxData.cmdData[ptr]);
        //   dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        //   dbgMsgCnt++;
        // }
        union ulongArray buffer;
        memcpy(buffer.byteArray, rxData.cmdData, 4);
        _isr_timeStamp = buffer.longNumber;
        // _isr_timeStamp = strtoul(rxData.cmdData, nullptr, 10);
        if (_isr_timeStamp>1000000000) {
          setTime(_isr_timeStamp);                            // fingers crossed
          HostsetTime = true;                               // set flag
          lasttimeSync = _isr_timeStamp;                      // record timestamp of sync
          if (!firsttimeSync) firsttimeSync = _isr_timeStamp; // if it's our first sync, record in separate variable
          sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Timestamp %lu", _isr_timeStamp);
          dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
          dbgMsgCnt++;
        } else {
          sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Error decoding timestamp! %lu", _isr_timeStamp);
          dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
          dbgMsgCnt++;
        }
      }
      break;
    case 0x61: // read first-init timestamp, ulong
      {
        union ulongArray buffer;
        buffer.longNumber = readFRAMulong(rxData.cmdAddr);
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x62: // read current timestamp, ulong
      {
        union ulongArray buffer;
        buffer.longNumber = now();
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Sending %lu", buffer.longNumber);
        // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        // dbgMsgCnt++;
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x63: // read time since last sync
      {
        union ulongArray buffer;
        buffer.longNumber = now() - lasttimeSync;   // subtract last sync timestamp from current timestamp
        txData.dataLen = 4;                                 // number of bytes to transmit
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
    case 0x64: // read time since last sync
      {
        union ulongArray buffer;
        txData.dataLen = 4;                                 // number of bytes to transmit
        buffer.longNumber = now() - firsttimeSync;  // subtract last sync timestamp from current timestamp
        memcpy(txData.cmdData, buffer.byteArray, txData.dataLen);
        _I2C_DATA_RDY = true;                                 // set flag we are ready to send data
      }
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
  purgeRXBuffer = true; // ask main loop() to purge buffer
} // end of handleEvent

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

uint32_t readFRAMuint(uint8_t myAddr) { 
  uint32_t framData = 0;
  if (myAddr==0x39) { // pack voltage
    framData = adcDataBuffer[2].adcRaw;
  } else if (myAddr==0x3E) { // bus voltage
    framData = adcDataBuffer[1].adcRaw;
  } else if (myAddr==0x33) { // active current
    framData = adcDataBuffer[0].adcRaw;
  }
  return framData;
}

// function to read uint from FRAM (eventually)
float readFRAMfloat(uint8_t myAddr) { 
  float framData = 0.0;
  if (myAddr==0x39) { // pack voltage
    framData = adcDataBuffer[2].Volts;
  } else if (myAddr==0x3E) { // bus voltage
    framData = adcDataBuffer[1].Volts;
  } else if (myAddr==0x33) { // active current
    framData = adcDataBuffer[0].Amps;
  }
  return framData;
}
long readADC(uint8_t adcPin, uint8_t noSamples) {
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

// function that executes whenever data is requested by Host
// this function is registered as an event, see setup()
void requestEvent() {   
  char reqBuff[80];                          // Host has requested data
  reqEvnt = true;   
  if (_I2C_DATA_RDY) {
    _I2C_DATA_RDY = false;                              // set flag that we had this interaction
    for (int x = 0; x < txData.dataLen; x++)
    {
      Wire.write(txData.cmdData[x]);
    }
    // Wire.write((char *) txData.cmdData, txData.dataLen);          // dump entire tx buffer to the bus, Host will read as many bytes as it wants
    _I2C_DATA_RDY = false;                          // clear tx flag
    // sprintf(reqBuff, "Request even triggered. Sent: %s", txData.cmdData);
    // Serial1.println(reqBuff);
  } else {
    sprintf((char) txData.cmdData,"Client 0x%X ready!", I2C_CLIENT_ADDR);
    Wire.write((char) txData.cmdData);                         // didn't have anything to send? respond with message of 6 bytes
    purgeTXBuffer=true;                                // purge TX buffer
  }
}

void clearTXBuffer() {
  uint16_t myPtr = 0;
  while (myPtr < txBufferSize) {
    txData.cmdData[myPtr] = '\0';
    myPtr++;
  }
  purgeTXBuffer = false;
}

void clearRXBuffer() {
  uint16_t myPtr = 0;
  rxData.cmdAddr = 0;
  rxData.dataLen = 0;
  while (myPtr < rxBufferSize) {
    rxData.cmdData[myPtr] = '\0';
    myPtr++;
  }
  purgeRXBuffer = false;
}

uint32_t framReadUlong(uint8_t dataAddress, uint8_t dataLen) {
  union ulongArray buffer;
  fram.readBlock(dataAddress, buffer.byteArray, dataLen);
  return buffer.longNumber;
}

void framWriteUlong(uint8_t dataAddress, uint32_t framData, uint8_t dataLen) {
  union ulongArray buffer;
  buffer.longNumber = framData;                               // convert float into byte array 
  fram.writeBlock(dataAddress, buffer.byteArray, dataLen);
}

int32_t framReadInt  (uint8_t dataAddress, uint8_t dataLen)
{
  union longArray buffer;
  fram.readBlock(dataAddress, buffer.byteArray, dataLen);
  return buffer.longNumber;
}

void framWriteInt (uint8_t dataAddress, int32_t  framData, uint8_t dataLen)
{
  union longArray buffer;
  buffer.longNumber = framData;                               // convert float into byte array 
  fram.writeBlock(dataAddress, buffer.byteArray, dataLen);
}

uint8_t framReadByte (uint8_t dataAddress)
{
  uint8_t buffer = fram.readByte(dataAddress);
  return buffer;
}

void framWriteByte(uint8_t dataAddress, uint8_t  framData)
{
  fram.writeByte(dataAddress, framData);
}

void framWriteDouble(uint8_t dataAddress, double framData)
{
  const uint8_t dataLen = 4;
  union doubleArray buffer;
  buffer.doubleVal = framData;
  fram.writeBlock(dataAddress, buffer.byteArray, dataLen);
}

double framReadDouble(uint8_t dataAddress)
{
  const uint8_t dataLen = 4;
  union doubleArray buffer;
  fram.readBlock(dataAddress, buffer.byteArray, dataLen);

  return buffer.doubleVal;
}

uint16_t framLookupAddr (uint8_t cmdAddress);
{

}

double getDouble(uint8_t * byteArray)
{
  union doubleArray buffer;
  const uint8_t dataLen = 4;
  memcpy(byteArray, buffer.byteArray, dataLen);

  return buffer.doubleVal;
}

uint32_t getULong(uint8_t * byteArray)
{
  union ulongArray buffer;
  const uint8_t dataLen = 4;
  memcpy(byteArray, buffer.byteArray, dataLen);

  return buffer.longNumber;
}

int32_t getLong(uint8_t * byteArray)
{
  union ulongArray buffer;
  const uint8_t dataLen = 4;
  memcpy(byteArray, buffer.byteArray, dataLen);

  return buffer.longNumber;
}
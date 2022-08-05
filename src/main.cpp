#include <Arduino.h>
#include <Wire.h>

#include "time.h"
#include "TimeLib.h"
#include "pm_pins.h"
#include "pm_struct.h"

volatile bool unknownCmd       = false;                  // flag indicating unknown command received
volatile bool txtmsgWaiting    = false;                  // flag indicating message from master is waiting
volatile bool reqEvnt          = false;                  // flag set when the requestEvent ISR fires
volatile bool recvEvnt         = false;                  // flag set when the receiveEvent ISR fires
volatile bool mastersetTime    = false;                  // flag that is set when master has sent time
volatile bool txdataReady      = false;                  // flag that is set when data is ready to send to master
volatile bool purgeTXBuffer    = true;                   // tell loop() to clear the TX buffer
volatile bool purgeRXBuffer    = true;                   // tell loop() to clear the RX buffer
volatile char txtMessage[50];                            // alternate buffer for message from master

volatile uint8_t messageLen    = 0;                      // message from master length
volatile time_t lasttimeSync   = 0;                      // when's the last time master sent us time?
volatile time_t firsttimeSync  = 0;                      // record the timestamp after boot

#ifdef DXCORE
#pragma message "Compiled using DxCORE!"
#endif

#ifdef I2C_SLAVE_ADDR
#pragma message "Slave address found in build flag"
#else
#pragma message "Slave address NOT FOUND in build flag, defaulting to 0x40"
#define I2C_SLAVE_ADDR 0x40
#endif

#ifdef  MCU_AVR128DA28
#pragma message "Compiling for AVR128DA28"
#define SERIALBAUD 921600
#elif   MCU_AVR128DA32
#pragma message "Compiling for AVR128DA32"
#define SERIALBAUD 921600
#else
#define SERIALBAUD 115200
#pragma message "Compiling for Unknown MCU"
#endif

char buff[200];

// function to eventually save data to on board FRAM
void writeFRAMuint(uint8_t myAddr, uint32_t myData) { 

}

void writeFRAMint(uint8_t myAddr, int32_t myData) { 

}

// function to read byte from FRAM
uint8_t readFRAMbyte(uint8_t myAddr) { 

}

// function to read uint from FRAM
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


// function to read ulong from FRAM
uint32_t readFRAMulong(uint8_t myAddr) { 

}

// function to read int from FRAM
int16_t readFRAMint(uint8_t myAddr) { 

}

long readADC(uint8_t adcPin, uint8_t noSamples) {
  uint32_t adcResult = 0;
  int      sample    = 0;
  uint8_t  adcX      = 0;
// #ifdef MCU_NANOEVERY
//   analogReference(INTERNAL4V3);  // enable interal 4.3v reference on the Every
// #endif

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

void clearRXBuffer() {
  uint16_t myPtr = 0;
  rxData.cmdAddr = 0;
  while (myPtr < rxBufferSize) {
    rxData.cmdData[myPtr] = '\0';
    myPtr++;
  }
  purgeTXBuffer = false;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {   
  char reqBuff[80];                          // master has requested data
  if (txdataReady) {
    Wire.write((char *) txData.cmdData, txData.dataLen);          // dump entire tx buffer to the bus, master will read as many bytes as it wants
    txdataReady = false;                          // clear tx flag
    // sprintf(reqBuff, "Request even triggered. Sent: %s", txData.cmdData);
    // Serial.println(reqBuff);
  } else {
    sprintf((char) txData.cmdData,"Slave 0x%X ready!", I2C_SLAVE_ADDR);
    Wire.write((char) txData.cmdData);                         // didn't have anything to send? respond with message of 6 bytes
    // Serial.println((char) txData.cmdData);
  }
  reqEvnt = true;                                 // set flag that we had this interaction
  purgeTXBuffer=true;                                // purge TX buffer
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(size_t howMany) {
  uint8_t   _isr_masterByte  = 0;
  uint16_t  _isr_masterUint  = 0;
  int16_t   _isr_masterInt   = 0;
  uint32_t  _isr_masterUlong = 0;
  int32_t   _isr_masterLong  = 0;
  uint32_t  _isr_timeStamp   = 0;


  Wire.readBytes( (uint8_t *) &rxData,  howMany);                  // transfer everything from buffer into memory
  rxData.dataLen = howMany - 1;                                    // save the data length for future use
  // sprintf(buff, "RX cmd 0x%X plus %u data bytes\n", rxData.cmdAddr, rxData.dataLen);
  // Serial.print(buff);
  
  rxData.cmdData[howMany] = '\0'; // set the Nth byte as a null
  // for (int xx = 0; xx<howMany; xx++) {
  //   Serial.print(rxData.cmdData[xx]);
  // }
  // Serial.println(" ");

  recvEvnt = true;                                                 // set event flag
  uint8_t _isr_cmdAddr = rxData.cmdAddr;
  
  switch  (_isr_cmdAddr) {
    case 0x00: // no command received
      Serial.println("Address probe detected.");    
    case 0x21: // high current limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
        writeFRAMuint(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x22: // high-temp limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
         writeFRAMuint(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x23: // low-temp limit, signed int
      {
        _isr_masterInt = atoi(rxData.cmdData);
        writeFRAMint(rxData.cmdAddr, _isr_masterInt);
      }
      break;
    case 0x24: // high-voltage limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
        writeFRAMuint(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x25: // low-voltage limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
        writeFRAMuint(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x26: // set config0, byte
      {
        _isr_masterByte = rxData.cmdData[0];
        writeFRAMuint(rxData.cmdAddr, _isr_masterByte);
      }
      break;
    case 0x27: // set config1, byte
      {
        _isr_masterByte = rxData.cmdData[0];
        writeFRAMuint(rxData.cmdAddr, _isr_masterByte);
      }
      break;
    case 0x28: // set config2, byte
      {
        _isr_masterByte = rxData.cmdData[0];
        writeFRAMuint(rxData.cmdAddr, _isr_masterByte);
      }
      break;
    case 0x29: // read config0, byte
      {
        _isr_masterByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_masterByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2A: // read config1, byte
      {
        _isr_masterByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_masterByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2B: // read config2, byte
      {
        _isr_masterByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_masterByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2C: // read status1, byte
      {
        _isr_masterByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_masterByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x2D: // read status2, byte
      {
        _isr_masterByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_masterByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
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
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x32: // clear total amps counter, no data
      { 
        writeFRAMint(0x34, 0);
        writeFRAMint(0x35, 0);
      }
      break;
    case 0x33: // read instant amps, signed int
      {
        //_isr_masterInt = readFRAMint(rxData.cmdAddr);
        //ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        float framData = readFRAMfloat(rxData.cmdAddr);
        dtostrf(framData, 3, 2, txData.cmdData);
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x34: // read total amps in counter, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x35: // read total amps out counter, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x36: // read lifetime amps in, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x37: // read lifetime amps out, ubsigned long 
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
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
    case 0x39: // read pack voltage, char* array
      {
        // _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        // ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        float framData = readFRAMfloat(rxData.cmdAddr);
        dtostrf(framData, 3, 2, txData.cmdData);
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3A: // read lowest voltage memory, unsigned int
      {
        _isr_masterUint = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3B: // read lowest voltage timestamp, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3C: // read highest voltage memory, unsigned int
      {
        _isr_masterUint = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3D: // read highest voltage timestamp, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x3E: // read bus voltage, char* array
      {
        // _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        // ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        float framData = readFRAMfloat(rxData.cmdAddr);
        dtostrf(framData, 3, 2, txData.cmdData);
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;

    case 0x3F: // print diag message from master
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
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x42: // read t0 lowest, signed int
      {
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x43: // read t0 highest, signed int
      {
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x44: // read t1 instant, signed int
      {
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x45: // read t1 lowest, signed int
      {
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x46: // read t1 highest, signed int
      {
        _isr_masterInt = readFRAMint(rxData.cmdAddr);
        ltoa(_isr_masterInt, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x47: // read t0 lowest timestamp, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x48: // read t0 highest timestamp, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x49: // read t1 lowest timestamp, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x4A: // read t1 highest timestamp, unsigned long
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
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
        _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x52: // read total under-voltage discon, unsigned int
      {
        _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x53: // read total over-volt discon, uint
      {
        _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x54: // read total under-temp discon, uint
      {
        _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 6;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x55: // read total over-temp discon, uint
      {
        _isr_masterUint = readFRAMuint(rxData.cmdAddr);
        ltoa(_isr_masterUint, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x56: // read last discon timestamp, ulong
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x57: // read last discon reason code, byte
      {
        _isr_masterByte = readFRAMbyte(rxData.cmdAddr);
        txData.cmdData[0] = _isr_masterByte;                // store byte in outgoing buffer
        txData.dataLen = 1;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x60: // set time from master, char string
      {
        // Serial.println((char) rxData.cmdData);
        _isr_timeStamp = strtoul(rxData.cmdData, nullptr, 10);
        if (_isr_timeStamp>1000000000) {
          setTime(_isr_timeStamp);                            // fingers crossed
          mastersetTime = true;                               // set flag
          lasttimeSync = _isr_timeStamp;                      // record timestamp of sync
          if (!firsttimeSync) firsttimeSync = _isr_timeStamp; // if it's our first sync, record in separate variable
          // sprintf(buff, "Timestamp %lu", _isr_timeStamp);
          // Serial.println(buff);
        } 
        // else Serial.println("Error receiving timestamp!");
      }
      break;
    case 0x61: // read first-init timestamp, ulong
      {
        _isr_masterUlong = readFRAMulong(rxData.cmdAddr);
        ltoa(_isr_masterUlong, txData.cmdData, 10);         // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x62: // read current timestamp, ulong
      {
        _isr_masterUlong = now();
        ltoa(_isr_masterUlong, txData.cmdData, 10);         // store data as char string in tx buffer
        txData.dataLen = 11;                                 // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;
    case 0x63: // read time since last sync
      {
        _isr_masterUlong = now();                           // read current time from time libvrary
        _isr_timeStamp = _isr_masterUlong - lasttimeSync;   // subtract last sync timestamp from current timestamp
        ltoa(_isr_timeStamp, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
    case 0x64: // read time since last sync
      {
        _isr_masterUlong = now();                           // read current time from time libvrary
        _isr_timeStamp = _isr_masterUlong - firsttimeSync;  // subtract last sync timestamp from current timestamp
        ltoa(_isr_timeStamp, txData.cmdData, 10);           // store data as char string in tx buffer
        txData.dataLen = 11;                                // number of bytes to transmit
        txdataReady = true;                                 // set flag we are ready to send data
      }
      break;

    default:// unknown command
      {
        unknownCmd = true;
      }
      break;
  } // end switch
  // sprintf(buff, "Receive event triggered. Command 0x%X", rxData.cmdAddr);
  // Serial.println(buff);
  purgeRXBuffer = true; // ask main loop() to purge buffer
}

#if defined(TWI_MORS_BOTH)
TwoWire i2c_slave(&TWI0);
TwoWire i2c_master(&TWI1);
#elif defined(TWI_MANDS_SINGLE)
TwoWire i2c_slave(&TWI0);
TwoWire i2c_master(&TWI0);
#endif

void setup() {
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

  #if defined(TWI_MORS_BOTH)
    #pragma message "TWI_MORS_BOTH is defined!"
    i2c_master.begin();                 // i2c_master is TWI1, using pins PF2, PF3 - only option for 32 pin chip
    i2c_master.setClock(100000);        // bus speed 100khz

    // i2c_slave is TWI0, accept default pins, should be PA2, PA3
    i2c_slave.pins(SDA0. SCL0);         // TWI0 on pins PA2, PA3
    i2c_slave.begin(I2C_SLAVE_ADDR); 
  #else 
    // Setup TWI0 for dual mode ... TWI_MANDS_SINGLE
    i2c_master.begin(SDA1, SCL1);       // master on alternate pins
    i2c_master.setClock(100000);        // 100khz clock

    // Setup second instance of TWO0 as slave
    i2c_slave.pins(SDA0, SCL0);         // pins per datasheet per TWIROUTE0, TWI0[1:0]
    i2c_slave.begin(I2C_SLAVE_ADDR);    // set slave address
  #endif

  #ifdef TWI_MANDS_SINGLE 
    #pragma message "TWI_MANDS_SINGLE is defined!"
  #endif

  delay(2000);

  Serial.begin(SERIALBAUD);

  sprintf(buff, "\n\nHello, world!\nSlave address: 0x%X\n", I2C_SLAVE_ADDR);
  Serial.print(buff);

  Wire.onRequest(requestEvent); // register requestEvent interrupt handler
  Wire.onReceive(receiveEvent); // register receiveEvent interrupt handler
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
  digitalWrite(LED4, mastersetTime);

  if (purgeTXBuffer) clearTXBuffer(); 
  if (purgeRXBuffer) clearRXBuffer();

  if (adcUpdateCnt > adcUpdateInterval) {
    long rawAdc    = 0;
    //int acsOffset  = 514;
    float acsmvA  = 0.136;  // 0.136v or 136mV per amp
    float Amps    = 0.0;
    float Volts   = 0.0;
    float sysVcc  = 4.43;
    float vDiv2   = 1.0;
    float vDiv3   = 1.0;
  
// #ifdef MCU_NANOEVERY
//     sysVcc        = 4.300;
// #endif
 
    rawAdc = readADC(ADC0, 20);
    rawAdc = rawAdc;
    //rawAdc = analogRead(ADC0);
    //rawAdc = rawAdc; // subtrack offset
    adcDataBuffer[0].adcRaw = rawAdc;
    Volts = (float)(rawAdc * (sysVcc / 1024.0)) - (sysVcc / 2);
    Amps =  (float)Volts / acsmvA;
    adcDataBuffer[0].Amps  = Amps;
    adcDataBuffer[0].Volts = Volts;
    // Serial.print("0: ");
    // Serial.print(Amps);
    // Serial.print(" 1: ");
    // Serial.print(Volts, 3);
    // Serial.print("v Raw ");
    // Serial.print(rawAdc);
    // Serial.println(" ");

    rawAdc = readADC(ADC1, 20);
    // Serial.print(rawAdc);
    // Serial.print(" <-ADC1 ADC2-> ");
    adcDataBuffer[1].adcRaw   = rawAdc;
    Volts = (float)(rawAdc * (sysVcc / 1024.0)) / vDiv2;
    adcDataBuffer[1].Volts = Volts;

    rawAdc = readADC(ADC2, 20);
    // Serial.println(rawAdc);
    adcDataBuffer[2].adcRaw   = rawAdc;
    Volts = (float)(rawAdc * (sysVcc / 1024.0)) / vDiv3;
    adcDataBuffer[1].Volts = Volts;
    adcUpdateCnt = 0;
  }
  
  if (unknownCmd) {
    unknownCmd = false;

    sprintf(buff, "Command 0x%X: Not recognized\n", rxData.cmdAddr);
    Serial.println(buff);
  }

  if (txtmsgWaiting) {            // print message sent by master
    txtmsgWaiting = false;        // clear flag
    sprintf(buff, "Message from master: %s", txtMessage);
    Serial.println(buff);
  }

  if (i>1000){
    i=0;
    if (timeStatus()==timeSet) {             // print timestamps once time is set
      ledX = ledX ^ 1;              // xor previous state
      digitalWrite(LED1, ledX);     // turn the LED on (HIGH is the voltage level)
      // Serial.printf("Timestamp: %lu\n", now());
    } 
    
    recvEvnt = false; // reset flag
    reqEvnt  = false; // reset flag
  }

  delay(1);
}
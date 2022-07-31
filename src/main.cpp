#include <Arduino.h>
#include <Wire.h>
#include "time.h"
#include "TimeLib.h"

//SDA 18 (A4) SCL 19 (A5)
#define LED1 PD2
#define LED2 PD3
#define LED3 PD4
#define LED4 PD5

#define SCL PC5 // A5
#define SDA PC4 // A4

struct I2C_RX_DATA {
  uint8_t cmdAddr = 0;        // single byte command register
  uint8_t cmdData[50] = {};   // room for twenty bytes of data
  char padding[10] = {};      // padding not sure it's needed
  size_t dataLen = 0;
};

volatile I2C_RX_DATA rxData;

volatile bool unknownCmd       = false;
volatile bool txtmsgWaiting    = false;
volatile bool reqEvnt          = false;
volatile bool recvEvnt         = false;
volatile bool mastersetTime    = false;
volatile char txtMessage[100];
volatile uint8_t messageLen    = 0;

char buff[50];

struct timeArray_t{
  byte regAddr;
  uint32_t timeStamp;
};

const uint8_t timeUnion_size = sizeof(timeArray_t);

union I2C_timePacket_t{
  timeArray_t currentTime;
  byte I2CPacket[timeUnion_size];
};

void updateFRAM(uint8_t myAddr, int32_t myData) { // function to eventually save data to on board FRAM

}
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() { // master has requested data
  Wire.write("hello "); // respond with message of 6 bytes
  reqEvnt = true; // set flag that we had this interaction
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(size_t howMany) {
  uint8_t   _isr_i           = 0;                   // length of string
  uint8_t   _isr_x           = 0;                                // pointer for printing string
  uint16_t  _isr_masterUint  = 0;
  int16_t   _isr_masterInt   = 0;
  uint8_t   _isr_masterByte  = 0;
  uint32_t  _isr_timeStamp   = 0;

  Wire.readBytes( (uint8_t *) &rxData,  howMany);                  // transfer everything from buffer into memory
  rxData.dataLen = howMany - 1;                                    // save the data length for future use
  // Serial.printf("RX cmd 0x%X and %u bytes\n", rxData.cmdAddr, rxData.dataLen);
  recvEvnt = true;                                                 // set event flag
  uint8_t _isr_cmdAddr = rxData.cmdAddr;
  
  switch  (_isr_cmdAddr) {
    case 0x21: // high current limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
        updateFRAM(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x22: // high-temp limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
         updateFRAM(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x23: // low-temp limit, signed int
      {
        int _isr_masterInt = atoi(rxData.cmdData);
        updateFRAM(rxData.cmdAddr, _isr_masterInt);
      }
      break;
    case 0x24: // high-voltage limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
        updateFRAM(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x25: // low-voltage limit, unsigned int
      {
        _isr_masterUint = atol(rxData.cmdData);
        updateFRAM(rxData.cmdAddr, _isr_masterUint);
      }
      break;
    case 0x26: // set config0, byte
      {
        _isr_masterByte = rxData.cmdData[0];
        updateFRAM(rxData.cmdAddr, _isr_masterByte);
      }
      break;
    case 0x27: // set config1, byte
    case 0x28: // set config2, byte
    case 0x29: // read config0, byte
    case 0x2A: // read config1, byte
    case 0x2B: // read config2, byte
    case 0x2C: // read status1, byte
    case 0x2D: // read status2, byte
    case 0x30: // clear coulomb counter, no data
      {
        digitalWrite(LED4, LOW);                                       // turn off LED4
      }
      break;
    case 0x31: // read coulomb counter, signed long
      {
        digitalWrite(LED4, HIGH);                     // turn on LED4
      }
      break;
    case 0x32: // clear total amps counter, no data
      {
        strncpy(txtMessage, rxData.cmdData, rxData.dataLen); // copy message into another buffer
        // messageLen = rxData.dataLen; // copy message length too
        txtmsgWaiting = true; // set flag to print the message in loop()
      }
      break;
    case 0x33: // read instant amps, signed int
    case 0x34: // read total amps in counter, unsigned long
    case 0x35: // read total amps out counter, unsigned long
    case 0x36: // read lifetime amps in, unsigned long
    case 0x37: // read lifetime amps out, ubsigned long 
    case 0x38: // clear voltage memory, no data
    case 0x39: // read pack voltage, unsigned int
    case 0x3A: // read lowest voltage memory, unsigned int
    case 0x3B: // read lowest voltage timestamp, unsigned long
    case 0x3C: // read highest voltage memory, unsigned int
    case 0x3D: // read highest voltage timestamp, unsigned long

    case 0x40: // clear temperature memories, no data
    case 0x41: // read t0 instant, signed int
    case 0x42: // read t0 lowest, signed int
    case 0x43: // read t0 highest, signed int
    case 0x44: // read t1 instant, signed int
    case 0x45: // read t1 lowest, signed int
    case 0x46: // read t1 highest, signed int
    case 0x47: // read t0 lowest timestamp, unsigned long
    case 0x48: // read t0 highest timestamp, unsigned long
    case 0x49: // read t1 lowest timestamp, unsigned long
    case 0x4A: // read t1 highest timestamp, unsigned long
    
    case 0x50: // clear disconnect history, no data
    case 0x51: // read total over-current disconnects, unsigned int
    case 0x52: // read total under-voltage discon, unsigned int
    case 0x53: // read total over-volt discon, uint
    case 0x54: // read total under-temp discon, uint
    case 0x55: // read total over-temp discon, uint
    case 0x56: // read last discon timestamp, ulong
    case 0x57: // read last discon reason code, byte

    case 0x60: // set time from master, char string
      {
        _isr_timeStamp = atol(rxData.cmdData);
        // Serial.printf("Ts from master: %lu\n", _isr_timeStamp);
        setTime(_isr_timeStamp);                                            // fingers crossed
        mastersetTime = true;                                          // set flag
      }
      break;
    case 0x61: // read first-init timestamp, ulong
    case 0x62: // read current timestamp, ulong

    default:// unknown register
      {
        unknownCmd = true;
      }
      break;
  } // end switch
}

void setup() {
  // initialize I2C pins
  pinMode(SCL, INPUT);
  pinMode(SDA, INPUT);

  // initialize LEDs outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  Wire.begin(0x37);                // join i2c bus with address #8
  delay(2000);

  Serial.begin(115200);

  Serial.println("\n\nHello, world!");

  Wire.onRequest(requestEvent); // register requestEvent event handler
  Wire.onReceive(receiveEvent); // register receiveEvent event handler
}

uint16_t i=0;
uint16_t x=0;
uint8_t ledX=0;

// the loop function runs over and over again forever
void loop() {
  i++;
  digitalWrite(LED2, reqEvnt);
  digitalWrite(LED3, recvEvnt);

  if (unknownCmd) {
    unknownCmd = false;
    Serial.printf("Command 0x%X: Not recognized\n", rxData.cmdAddr);
  }

  if (txtmsgWaiting) {            // print message sent by master
    txtmsgWaiting = false;        // clear flag
    Serial.printf("Message from master: %s", txtMessage);
    Serial.println();
  }

  if (i>1000){
    i=0;
    ledX = ledX ^ 1;              // xor previous state
    digitalWrite(LED1, ledX);     // turn the LED on (HIGH is the voltage level)
    if (timeStatus()==timeSet) {             // print timestamps once time is set
      Serial.printf("Timestamp: %lu\n", now());
    } 
  }

  delay(1);
}
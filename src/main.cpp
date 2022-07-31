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

I2C_RX_DATA rxData;

uint16_t i=0;
bool reqEvnt          = false;
bool recvEvnt         = false;
bool mastersetTime    = false;
bool timeisSet        = false;
uint8_t ledX          = 0;

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
  uint8_t i = 0;                   // length of string
  uint8_t x = 0;                                // pointer for printing string
  Wire.readBytes( (uint8_t *) &rxData,  howMany);                  // transfer everything from buffer into memory
  rxData.dataLen = howMany - 1;                                    // save the data length for future use
  Serial.printf("RX %u bytes\n", (uint8_t) rxData.dataLen);
  recvEvnt = true;                                                 // set event flag
  switch  (rxData.cmdAddr) {
    case 0x20: // set time from master, char string
      unsigned long timeStamp = atol(rxData.cmdData);
      //Serial.printf("Command 0x20: Received timestamp %lu\n", timeStamp);
      setTime(timeStamp);                                            // fingers crossed
      mastersetTime = true;                                          // set flag
    case 0x21: // high current limit, unsigned int
      uint16_t masterData = atol(rxData.cmdData);
      updateFRAM(rxData.cmdAddr, masterData);
    case 0x22: // high-temp limit, unsigned int
      uint16_t masterData = atol(rxData.cmdData);
      updateFRAM(rxData.cmdAddr, masterData);
    case 0x23: // low-temp limit, signed int
      int masterData = atoi(rxData.cmdData);
      updateFRAM(rxData.cmdAddr, masterData);
    case 0x24: // high-voltage limit, unsigned int
      uint16_t masterData = atol(rxData.cmdData);
      updateFRAM(rxData.cmdAddr, masterData);
    case 0x25: // low-voltage limit, unsigned int
      uint16_t masterData = atol(rxData.cmdData);
      updateFRAM(rxData.cmdAddr, masterData);
    case 0x26: // set config0, byte
      masterData = rxData.cmdData[0];
      updateFRAM(rxData.cmdAddr, masterData);
    case 0x27: // read config0, byte
    case 0x28: // read status0, byte
    case 0x29:
    case 0x2A:
    case 0x2B:
    case 0x2C:
    case 0x2D:
    case 0x2E:
    case 0x2F:
    case 0x30:
      digitalWrite(LED4, LOW);                                       // turn off LED4
      //Serial.println("Command 0x30: LED 4 off");
    case 0x31:
      digitalWrite(LED4, HIGH);                     // turn on LED4
      //Serial.println("Command 0x31: LED 4 on");
    case 0x32: 
      Serial.print("Command 0x32: Received ");
      i = rxData.dataLen;                   // length of string
      while (x<i) {                                 // print string one char at a time
        Serial.print((char) rxData.cmdData[x]);
        x++;
      }
      Serial.println();                             // send a newline  
    case 0x33:
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
    case 0x38:
    case 0x39:
    case 0x3A:
    case 0x3B:
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
    case 0x40:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
    case 0x45:
    case 0x46:
    case 0x47:
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
    case 0x4C:
    case 0x4D:
    case 0x4E:
    case 0x4F:
    case 0x50:
    case 0x51:
    case 0x52:
    case 0x53:
    case 0x54:
    case 0x55:
    case 0x56:
    case 0x57:
    case 0x58:
    case 0x59:
    case 0x5A:
    case 0x5B:
    case 0x5C:
    case 0x5D:
    case 0x5E:
    case 0x5F:
    case 0x60:
    case 0x61:
    case 0x62:
    case 0x63:
    case 0x64:
    case 0x65:
    case 0x66:
    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F:

    default:// unknown register
      Serial.printf("Command 0x%X: Not recognized\n", rxData.cmdAddr);
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

// the loop function runs over and over again forever
void loop() {
  i++;
  digitalWrite(LED2, reqEvnt);
  digitalWrite(LED3, recvEvnt);

  if (i>1000){
    i=0;
    ledX = ledX ^ 1; // xor previous state
    digitalWrite(LED1, ledX);   // turn the LED on (HIGH is the voltage level)
    Serial.print("Timestamp ");
    Serial.println(now());
  }

  delay(1);
}
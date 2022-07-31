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
bool reqEvnt = false;
bool recvEvnt = false;
uint8_t ledX = 0;

char buff[50];

struct timeArray_t{
  byte regAddr;
  uint32 timeStamp;
};

const uint8_t timeUnion_size = sizeof(timeArray_t);

union I2C_timePacket_t{
  timeArray_t currentTime;
  byte I2CPacket[timeUnion_size];
};

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() { // master has requested data
  Wire.write("hello "); // respond with message of 6 bytes
  reqEvnt = true; // set flag that we had this interaction
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(size_t howMany) {
  Wire.readBytes( (uint8_t *) &rxData, howMany);  // transfer everything from buffer into memory
  rxData.dataLen = howMany - 1;                   // save the data length for future use
  Serial.printf("RX %u bytes: ", (uint8_t) rxData.dataLen);
  recvEvnt = true;                                // set event flag
  if (rxData.cmdAddr==0x20) {
    Serial.println("Command 0x20: Set time");
  } else if (rxData.cmdAddr==0x30) {
    digitalWrite(LED4, LOW);                      // turn off LED4
    Serial.println("Command 0x30: LED 4 off");
  } else if (rxData.cmdAddr==0x31) {
    digitalWrite(LED4, HIGH);                     // turn on LED4
    Serial.println("Command 0x31: LED 4 on");
  } else if (rxData.cmdAddr==0x32) {
    uint8_t i = rxData.dataLen;                   // length of string
    uint8_t x = 0;                                // pointer for printing string
    Serial.print("Command 0x32: Message is ");
    while (x<i) {                                 // print string one char at a time
      Serial.print((char) rxData.cmdData[x]);
      x++;
    }
    Serial.println();                             // send a newline  
  } else {                                        // unknown register
    Serial.printf("Command 0x%X: Not recognized\n", rxData.cmdAddr);
  }
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

  if (i>500){
    if (reqEvnt) {
      //Serial.println(" TX ");
      reqEvnt = false;
    }
    if (recvEvnt) {
      //Serial.println(" RX ");
      recvEvnt = false;
    }
    i=0;
    ledX = ledX ^ 1; // xor previous state
    digitalWrite(LED1, ledX);   // turn the LED on (HIGH is the voltage level)
    
  }

  delay(1);
}
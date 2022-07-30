#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//SDA 18 (A4) SCL 19 (A5)
#define LED1 PD2
#define LED2 PD3
#define LED3 PD4
#define LED4 PD5

#define SCL PC5 // A5
#define SDA PC4 // A4
int i=0;
bool reqEvnt = false;
bool recvEvnt = false;
uint8_t ledX = 0;

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() { // master has requested data
  Wire.write("hello "); // respond with message of 6 bytes
  reqEvnt = true; // set flag that we had this interaction
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  (void)howMany;                    // cast unused parameter to void to avoid compiler warning
  uint16_t recvCnt = 0;             // counter for bytes received
  recvEvnt = true;                  // set event flag
  while (1 < Wire.available()) {    // loop through all but the last
    char c = Wire.read();           // receive byte as a character
    Serial.print(c);                // print the character
    recvCnt++;                      // increment counter
    recvEvnt = true;                // set event flag
  }
  if (Wire.available()) {           // receive last byte
    uint8_t x = Wire.read();        // receive byte as an integer
    Serial.println(x);              // print the integer
  } else {                          // no data received?
    //if (recvCnt==0) { requestEvent(); }
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

  Serial.begin(115200);

  Serial.println("Hello, world!");

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
      Serial.print("Request event ");
      reqEvnt = false;
    }
    if (recvEvnt) {
      Serial.print("Receive event ");
      recvEvnt = false;
    }
    i=0;
    ledX = ledX ^ 1; // xor previous state
    digitalWrite(LED1, ledX);   // turn the LED on (HIGH is the voltage level)
    
  }

  delay(1);
}
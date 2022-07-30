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
bool wireEvnt = false;
uint8_t ledX = 0;

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write("hello "); // respond with message of 6 bytes
  wireEvnt = true;
  // as expected by master
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

  Wire.onRequest(requestEvent); // register event
}

// the loop function runs over and over again forever
void loop() {
  i++;
  digitalWrite(LED2, wireEvnt);

  if (i>500){
    i=0;
    ledX = ledX ^ 1; // xor previous state
    digitalWrite(LED1, ledX);   // turn the LED on (HIGH is the voltage level)
    wireEvnt = false;
  }

  delay(1);
}
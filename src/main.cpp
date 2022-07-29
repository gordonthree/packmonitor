#include <Arduino.h>
#include <SPI.h>

//SCK D13, MISO D12, MOSI D11, SS D10
#define LED1 PD2
#define LED2 PD3
#define LED3 PD4

int i=0;
volatile boolean received;

volatile byte Slavereceived,Slavesend;

int buttonvalue;

uint8_t ledX = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(MISO, OUTPUT); 
  pinMode(SS, INPUT);
  
  // enable spi slave mode
  SPCR |= _BV(SPE); 

  
  SPI.attachInterrupt();   


  Serial.begin(115200);

}

// SPI Slave ISR
ISR (SPI_STC_vect)
{
  Slavereceived = SPDR;                  
  received = true;                       
}

// the loop function runs over and over again forever
void loop() {
  i++;

  if (Slavereceived==1){
    digitalWrite(LED1, HIGH); //Sets pin as HIGH LED ON
    //Serial.println("Slave LED ON");
  } else {
    digitalWrite(LED1, LOW);     //Sets pin as LOW LED OFF
    //Serial.println("Slave LED OFF");
  }

  if (i>1000){
    i=0;
    ledX = ledX ^ 1; // xor previous state
    digitalWrite(LED2, ledX);   // turn the LED on (HIGH is the voltage level)
    //Serial.println("Toggle LED");
  }

  delay(1);
}
#include <Arduino.h>
#include <SPI.h>

//SCK D13, MISO D12, MOSI D11, SS D10
#define LED1 PD2
#define LED2 PD3
#define LED3 PD4

int i=0;
volatile boolean received = false;

volatile uint8_t Slavereceived, Slavesend;

int buttonvalue;

uint8_t ledX = 0;
uint8_t ledSPIF = 0;

uint8_t SPI_SlaveReceive(void); // define routine found toward end of code

void setup() {
  // initialize LEDs outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // MISO as output, everything else input
  pinMode(MISO, OUTPUT); 
  pinMode(SS, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT); // pin shared with LED_BUILTIN on NANO
    
  // from the Microchip datasheet page 140
  SPCR = (1<<SPIE); // enable SPI interrupt
  SPCR = (1<<SPE);  // enable slave mode 
  
  SPI.attachInterrupt(); // enable SPI receive flag interrupt, takes no arguments

  Serial.begin(115200);

  Serial.println("Hello, world!");
}

// SPI Slave ISR
ISR (SPI_STC_vect)
{
  Slavereceived = SPI_SlaveReceive(); // read data from register
  ledSPIF = ledSPIF ^ 1; // toggle led as interrupt fires
  received = true;                       
}

uint8_t SPI_SlaveReceive(void)
{
  uint8_t recv = 0;
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  recv = SPDR; // read data register
  SPDR = recv; // send some data back?

  return recv;
}

// the loop function runs over and over again forever
void loop() {
  i++;
  digitalWrite(LED2, ledSPIF);

  if (received){
    Serial.print("Slave recv: ");
    Serial.println(Slavereceived);
    received = false; // clear flag
  }

  if (i>1000){
    i=0;
    ledX = ledX ^ 1; // xor previous state
    digitalWrite(LED1, ledX);   // turn the LED on (HIGH is the voltage level)
  }

  delay(1);
}
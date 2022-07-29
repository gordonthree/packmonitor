#include <Arduino.h>
#include <SPI.h>

//SCK D13, MISO D12, MOSI D11, SS D10
#define LED1 PD2
#define LED2 PD3
#define LED3 PD4

int i=0;
volatile boolean received;

volatile uint8_t Slavereceived, Slavesend;

int buttonvalue;

uint8_t ledX = 0;
uint8_t ledSPIF = 0;

char SPI_SlaveReceive(void);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize LEDs as outputs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // spi stuff
  pinMode(MISO, OUTPUT); 
  pinMode(SS, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT); // LED_BUILTIN
  
  // enable spi slave mode
  //SPCR |= _BV(SPE); 
  
  // from the Microchip datasheet page 140
  SPCR = (1<<SPIE); // enable SPI interrupt
  SPCR = (1<<SPE); // enable slave mode 
  
  SPI.attachInterrupt();   

  Serial.begin(115200);

  Serial.println("Hello, world!");
  
  received = false;
}

// SPI Slave ISR
ISR (SPI_STC_vect)
{
  Slavereceived = SPI_SlaveReceive(); // read data from register
  ledSPIF = ledSPIF ^ 1; // toggle led as interrupt fires
  received = true;                       
}

char SPI_SlaveReceive(void)
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
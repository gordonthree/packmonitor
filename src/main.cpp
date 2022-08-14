#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include <TimeLib.h>
#include <I2C_eeprom.h>

#include "NAU7802.h"
#include "pm_defs.h"
#include "pm_pins.h"
#include "pm_struct.h"
#include "pm_fram.h"

volatile bool unknownCmd       = false;                             // flag indicating unknown command received
volatile bool txtmsgWaiting    = false;                             // flag indicating message from Host is waiting
volatile bool reqEvnt          = false;                             // flag set when the requestEvent ISR fires
volatile bool recvEvnt         = false;                             // flag set when the receiveEvent ISR fires
volatile bool HostsetTime      = false;                             // flag that is set when Host has sent time
volatile bool _I2C_DATA_RDY    = false;                             // flag that is set when data is ready to send to Host
volatile bool _I2C_CMD_RECV    = false;                             // flag that is set when host sends a command
volatile bool purgeTXBuffer    = true;                              // tell loop() to clear the TX buffer
volatile bool dumpEEprom       = false;                             // tell loop to print contents of eeprom buffer to serial
volatile char txtMessage[50];                                       // alternate buffer for message from Host
volatile bool readNext         = false;

volatile uint8_t   messageLen     = 0;                              // message from Host length
volatile uint32_t  lasttimeSync   = 0;                              // when's the last time Host sent us time?
volatile uint32_t  firsttimeSync  = 0;                              // record the timestamp after boot

volatile DEBUG_MSGS  dbgMsgs[50];                                   // room for messages from inside isr to be printed outside
volatile uint8_t     dbgMsgCnt  = 0;                                // counter for how many debug messages are waiting

volatile I2C_TX_DATA txData;
volatile uint8_t     i2cRegAddr = 0;                                // keep track of the command address for multi-register reads

NAU7802     extAdc;                                                 // NAU7802 ADC device
FRAMSTORAGE fram;                                                   // access the array for storing eeprom contents
I2C_eeprom  ee_fram(0x50, I2C_DEVICESIZE_24LC64);                   // setup the eeprom here in the global scope

const uint8_t ee_record_size    = 24;                               // save constant for size of eedata structure
const uint8_t ee_buffer_size    = 0x7F;                             // number of recordds in the buffer array
const uint8_t ee_start_offset   = 0x64;                             // eeprom offset is 100 bytes (0x64), save that space for other uses
const uint8_t adcUpdateInterval = 250;                              // every 250 msec approximately

uint8_t   hiTalarm_cnt;                                             // counter for high temp alarm trigger
uint8_t   loTalarm_cnt;                                             // same as above for low temop
uint8_t   hiValarm_cnt;                                             // counter for high voltage alarm trigger
uint8_t   loValarm_cnt;                                             // same as above for under-voltage
uint8_t   hiIalarm_cnt;                                             // counter for high ampes alarm trigger
uint8_t   tAlarm_threshold = 20;                                    // need this many consecutive alarms to engage temp protection
uint8_t   vAlarm_threshold = 20;                                    // same as above for voltage
uint8_t   iAlarm_threshold = 5;                                     // need this many alarms to trigger over-current protection
uint8_t   framRegCnt       = 0x70;                                  // room for registers
uint8_t   framRegSize      = 24;                                    // bytes of memory for every fram register
char      buff[200];                                                // temporary buffer for working with char strings
int       I2C_CLIENT_ADDR = 0x34;                                   // base address, modified by pins PF0 / PF2
bool      FirstRun = false;                                         // flag to init fram memory and write first-init timestamp

void      framSave();                                               // write in-ram buffer to fram chip
void      framLoad();                                               // populate in-ram buffer from fram chip
void      framDump();                                               // sync buffer contents with fram chip, dump contents of buffer
void      printConfig        (void);                                // dump various config elements to serial port
void      updateReadings     (void);                                // read various sensors and update memory
void      scanI2C            (void);                                // scan local client bus
void      receiveEvent       (size_t howMany);                      // triggered after address match with write bit set
void      requestEvent       (void) ;                               // triggered after address match with read bit set
void      clearTXBuffer      (void);                                // reset transmit buffer to null
void      updateIload        (uint8_t reg);                         // read external adc channel, store results in buffer
void      updateVpack        (uint8_t chan, uint8_t reg);           // read external adc channel, store results in buffer
int32_t   getLong            (uint8_t * byteArray);                 // convert byte array to signed int
uint32_t  getULong           (uint8_t * byteArray);                 // convert byte array to unsigned int
double    getDouble          (uint8_t * byteArray);                 // convert byte array to double
uint32_t  readADC            (uint8_t adcPin, uint8_t noSamples);   // return value from internal ADC

tempsensor_t tempReadings[3];                                              // buffer to store three temperature readings in

#if defined(MCU_AVR128DA32)
  //HardwareI2C &i2c_host = TWI1;
  TwoWire &i2c_host = TWI1;
#endif

void(* resetFunc) (void) = 0;//declare reset function at address 0

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

  extAdc.begin(0x2A, Wire);      // pass through TWI to adc library
  extAdc.avcc4V5();              // set voltage regulator and analog reference to internal at 4.5v
  extAdc.rate010sps();           // set conversion rate to 320sps

  Serial1.print("Loading data from FRAM... ");

  // fram.begin(ee_fram);
  fram.begin();
  framLoad();

  Serial1.print("complete.\n");

  #ifdef PM_FIRSTRUN
    FirstRun = true;
  #endif

  uint8_t STATUS0 = 0;
  uint8_t CONFIG0 = fram.getDataByte(PM_REGISTER_CONFIG0BYTE); // grab config0 byte
  if (CONFIG0!=0) STATUS0 = 0x80;                              // if config0 is not zero, assume a config has been set, update status
  fram.addByte(PM_REGISTER_STATUS0BYTE, 0, STATUS0);           // init status0
  fram.addByte(PM_REGISTER_STATUS1BYTE, 0, 0);                 // init status1
  setTime(fram.getDataUInt(PM_REGISTER_TIMESYNC));             // set time based on last-sync time from fram
  
  printConfig();
} // end setup()

uint16_t      i                 = 0;
uint16_t      iFive             = 0;
uint16_t      x                 = 0;
uint8_t       ledX              = 0;
uint8_t       adcUpdateCnt      = 0;
uint32_t      timeStamp         = 0;

// the loop function runs over and over again forever
void loop() {
  i++;
  iFive++;
  timeStamp = now();
  adcUpdateCnt++;

  digitalWrite(LED2, reqEvnt);
  digitalWrite(LED3, recvEvnt);
  digitalWrite(LED4, HostsetTime);

  if (dumpEEprom) {
    framDump();
  }

  if (timeSet && FirstRun) {
    fram.addUInt(PM_REGISTER_FIRSTINITTIME, now(), now());
    FirstRun = false;
  }

  if (purgeTXBuffer) clearTXBuffer(); 

  if (adcUpdateCnt > adcUpdateInterval) {   
    updateReadings();
    adcUpdateCnt = 0; // reset counter for adc update delay
  }
  
  if (txtmsgWaiting) {            // print message sent by Host
    txtmsgWaiting = false;        // clear flag
    sprintf(buff, "Message from Host: %s", txtMessage);
    Serial1.println(buff);
  }

  if (i>1000) {                   // fires roughly every 1 second
    i=0;
    ledX = ledX ^ 1;              // xor previous state
    digitalWrite(LED1, ledX);     // turn the LED on and off to show program running
    
    recvEvnt = false;             // reset flag
    reqEvnt  = false;             // reset flag

    // update current time register
    if (timeSet()) fram.addUInt(PM_REGISTER_CURRENTTIME, timeStamp, timeStamp); 

    // update uptime if clock has been set
    if (firsttimeSync!=0) fram.addUInt(PM_REGISTER_UPTIME, timeStamp, timeStamp - firsttimeSync);

    // update last time sync if needed
    if (fram.getDataUInt(PM_REGISTER_TIMESYNC)!=lasttimeSync) fram.addUInt(PM_REGISTER_TIMESYNC, timeStamp, lasttimeSync);
  }

  if (iFive>5000) {              // roughly every 5 seconds
    framSave();                  // write memory cache to fram for backup
  
    hiTalarm_cnt = 0;            // reset temp alarm counter
    loTalarm_cnt = 0;            // same
    hiValarm_cnt = 0;            // reset voltage alarm counter
    loValarm_cnt = 0;            // same
    hiIalarm_cnt = 0;            // same


    iFive = 0;                   // reset five second counter
  }

  if (dbgMsgCnt>0) { // display any debug messages generated inside the ISRs
    for (int ptr=0; ptr < dbgMsgCnt; ptr++) {
      Serial1.printf("Diag message %u: %s\n", ptr + 1, dbgMsgs[ptr].messageTxt);
    }
    Serial1.println("");
    dbgMsgCnt = 0;
  }

  delay(1);
}

void updateReadings() {
  double  vBusDiv      = fram.getDataDouble(PM_REGISTER_VBUSDIVISOR);          // grab bus voltage divisor from buffer
  double  vPackDiv     = fram.getDataDouble(PM_REGISTER_VPACKDIVISOR);         // resistor divider scale for pack voltage
  double  hiTemp       = fram.getDataDouble(PM_REGISTER_HIGHTEMPLIMIT);        // grab high temp limit value from buffer
  double  loTemp       = fram.getDataDouble(PM_REGISTER_LOWTEMPLIMIT);         // grab low temp limit value from buffer
  double  vPackLow     = fram.getDataDouble(PM_REGISTER_LOWVOLTLIMIT);         // pack low voltage limit
  double  vPackHigh    = fram.getDataDouble(PM_REGISTER_HIGHVOLTLIMIT);        // pack high voltage limit
  double  iLoadHigh    = fram.getDataDouble(PM_REGISTER_HIGHCURRENTLIMIT);     // load high current limit
  double  mvA          = fram.getDataDouble(PM_REGISTER_CURRENTMVA);           // millivolts per amp divisor
  uint8_t CONFIG0      = fram.getDataByte(PM_REGISTER_CONFIG0BYTE);            // grab CONFIG0 byte from buffer
  uint8_t STATUS0      = fram.getDataByte(PM_REGISTER_STATUS0BYTE);            // grab STATUS0 byte from buffer
  uint8_t STATUS1      = fram.getDataByte(PM_REGISTER_STATUS1BYTE);            // grab STATUS0 byte from buffer
  double  aVcc         = 4.500;                                                // reference voltage for precision adc
  bool    packVerror   = false;                                                // error flag pack voltage sensor out of range
  bool    loadIerror   = false;                                                // error flag current sensor out of range
  bool    tempError    = false;                                                // error flag temp sensor out of range
  bool    hiIalarm     = false;                                                // alarm flag load current is too high
  bool    hiTalarm     = false;                                                // alarm flag pack temp too high
  bool    loTalarm     = false;                                                // alarm flag pack temp too low
  bool    packVwarn    = false;                                                // warning flag low pack voltage
  bool    hiIwarn      = false;                                                // warning flag high current limit
  bool    tempWarn     = false;                                                // warning flag pack temp limits
  bool    loVbus       = false;                                                // flag set when bus voltage is too low
  bool    hiVbus       = false;                                                // flag set when bus voltage too high
  bool    loVpack      = false;                                                // flag set when pack voltage is too low
  bool    hiVpack      = false;                                                // flag when pack voltage is too high
  bool    vbusWarn     = false;                                                // flag indicating problem with bus voltage
  bool    alarmDisable = bit_is_set(CONFIG0, PM_CONFIG0_DISABLEPROTS);         // test if under-temp protection is enabled
  bool    hiAalarm_ena = bit_is_set(CONFIG0, PM_CONFIG0_ENAOVRCURPROT);        // test if under-temp protection is enabled
  bool    hiTalarm_ena = bit_is_set(CONFIG0, PM_CONFIG0_ENAOVRTMPPROT);        // test if over-temp protection is enabled
  bool    loTalarm_ena = bit_is_set(CONFIG0, PM_CONFIG0_ENAUNDTMPPROT);        // test if under-temp protection is enabled
  bool    loValarm_ena = bit_is_set(CONFIG0, PM_CONFIG0_EMAUNDVLTPROT);        // test if under-temp protection is enabled
  bool    hiValarm_ena = bit_is_set(CONFIG0, PM_CONFIG0_ENAOVRVLTPROT);        // test if under-temp protection is enabled
  
  int32_t  rawAdc      = 0;
  double   rawDouble   = 0.0;
  uint32_t timeStamp   = now();

  rawAdc               = readADC(ADC1, 20);                                    // update in-memory value for internal adc1
  rawDouble            = raw2temp(rawAdc);                                     // convert raw to temperature using LUT
  
  if (rawDouble>100.0 || rawDouble<-30.0) {                                    // test for temperature sensor error    
    tempError = true;                                                          // set temperature error flag if temp outside rational limits
  } else {                                                                     // temp seems normal, check conditions
    fram.addRaw(PM_REGISTER_READDEGCT0, timeStamp, rawAdc);                    // store unprocessed raw value
    fram.addDouble(PM_REGISTER_READDEGCT0, timeStamp, rawDouble);              // use LUT to find approximate temperature

    if (rawDouble>hiTemp)                                 hiTalarm = true;     // set high temp alarm if temp exceeds limit
    if (rawDouble<loTemp)                                 loTalarm = true;     // set low temp alarm if temp exceeds limit
    if (rawDouble>hiTemp - 2.0 || rawDouble<loTemp + 2.0) tempWarn = true;     // set a warning if temp is within two degrees high or low
    
    // update in-memory buffer
    tempReadings[0].sensorAddr = PM_REGISTER_READDEGCT0;
    tempReadings[0].tempReading = rawDouble;

    // update low temp record
    if (rawDouble<tempReadings[0].lowReading) {
      tempReadings[0].lowReading = rawDouble;
      fram.addDouble(PM_REGISTER_READT0LOW, timeStamp, tempReadings[0].lowReading);
    } 

    // update high temp record
    if (rawDouble>tempReadings[0].highReading) {
      tempReadings[0].highReading = rawDouble;
      fram.addDouble(PM_REGISTER_READT0HIGH, timeStamp, tempReadings[0].highReading);
    }
  }

  rawAdc    = readADC(ADC2, 20);                                               // update value for internal adc2
  rawDouble = raw2temp(rawAdc);                                                // convert raw to temperature using LUT

  if (rawDouble>100.0 || rawDouble<-30.0) {                                    // test for temperature sensor error    
    tempError = true;                                                          // set temperature error flag if temp outside rational limits
  } else {                                                                     // temp seems normal, check conditions
    fram.addRaw(PM_REGISTER_READDEGCT1, timeStamp, rawAdc);                    // store unprocessed raw value
    fram.addDouble(PM_REGISTER_READDEGCT1, timeStamp, rawDouble);              // use LUT to find approximate temperature

    if (rawDouble>hiTemp)                                 hiTalarm = true;     // set high temp alarm if temp exceeds limit
    if (rawDouble<loTemp)                                 loTalarm = true;     // set low temp alarm if temp exceeds limit
    if (rawDouble>hiTemp - 2.0 || rawDouble<loTemp + 2.0) tempWarn = true;     // set a warning if temp is within two degrees high or low
    
    // update in-memory buffer
    tempReadings[1].sensorAddr = PM_REGISTER_READDEGCT1;
    tempReadings[1].tempReading = rawDouble;

    // update low temp record
    if (rawDouble<tempReadings[1].lowReading) {
      tempReadings[1].lowReading = rawDouble;
      fram.addDouble(PM_REGISTER_READT1LOW, timeStamp, tempReadings[1].lowReading);
    } 

    // update high temp record
    if (rawDouble>tempReadings[1].highReading) {
      tempReadings[1].highReading = rawDouble;
      fram.addDouble(PM_REGISTER_READT1HIGH, timeStamp, tempReadings[1].highReading);
    }

  }

  rawAdc = readADC(ADC3, 20);                                                  // update value for internal adc3
  rawDouble = raw2temp(rawAdc);                                                // convert raw to temperature using LUT

  if (rawDouble>100.0 || rawDouble<-30.0) {                                    // test for temperature sensor error    
    tempError = true;                                                          // set temperature error flag if temp outside rational limits
  } else {                                                                     // temp seems normal, check conditions
    fram.addRaw(PM_REGISTER_READDEGCT2, timeStamp, rawAdc);                    // store unprocessed raw value 
    fram.addDouble(PM_REGISTER_READDEGCT2, timeStamp, rawDouble);              // use LUT to find approximate temperature

    if (rawDouble>hiTemp)                                 hiTalarm = true;     // set high temp alarm if temp exceeds limit
    if (rawDouble<loTemp)                                 loTalarm = true;     // set low temp alarm if temp exceeds limit
    if (rawDouble>hiTemp - 2.0 || rawDouble<loTemp + 2.0) tempWarn = true;     // set a warning if temp is within two degrees high or low
    
    // update in-memory buffer
    tempReadings[2].sensorAddr = PM_REGISTER_READDEGCT2;
    tempReadings[2].tempReading = rawDouble;

    // update low temp record
    if (rawDouble<tempReadings[2].lowReading) {
      tempReadings[2].lowReading = rawDouble;
      fram.addDouble(PM_REGISTER_READT2LOW, timeStamp, tempReadings[2].lowReading);
    } 

    // update high temp record
    if (rawDouble>tempReadings[2].highReading) {
      tempReadings[2].highReading = rawDouble;
      fram.addDouble(PM_REGISTER_READT2HIGH, timeStamp, tempReadings[2].highReading);
    }
  }

  if (hiTalarm && hiTalarm_ena) hiTalarm_cnt++;                                // increment high alarm counter
  if (loTalarm && loTalarm_ena) loTalarm_cnt++;                                // increment low temp alarm counter

  if (tempError) {
    STATUS0 |= 1<<PM_STATUS0_RANGETSNS;                                        // set flag in status0 register
    Serial1.printf("%lu: ERROR: Temperature sensor out of range\n", timeStamp);
    tempError = false;} 
  else STATUS0 |= 1<<PM_STATUS0_RANGETSNS;                                     // set flag in status0 register

  if (tempWarn) {
    STATUS0 |= 1<<PM_STATUS0_WARNTEMP;                                         // set flag in status0 register
    Serial1.printf("%lu: WARN: Temperature sensor near threshold\n", timeStamp);
    tempWarn = false;}
  else STATUS0 |= 0<<PM_STATUS0_WARNTEMP;                                      // set flag in status0 register

  rawAdc    = readADC(ADC0, 20);                                               // read bus voltage from internal adcd
  rawDouble = raw2volts(rawAdc, vBusDiv);                                      // convert raw value into volts

  if ((rawDouble<4.70) || (rawDouble>5.10))
    STATUS1 |= 1<<PM_STATUS1_RANGEVBUS;                                        // set low bus voltage status bit
  else 
    STATUS1 |= 0<<PM_STATUS1_RANGEVBUS;                                        // clear low bus voltage status bit

  extAdc.selectCh1();                                                          // current sensor on adc ch1
  rawAdc    = extAdc.readADC();                                                // get raw 24-bit value
  //  readMV() = (_avcc/(float)16777216)*(float)readADC();
  rawDouble = (aVcc / (double)16777216) * (double)rawAdc;                      // get current, divide millivolts by millivolts per amp
  rawDouble = rawDouble / mvA;                                                 // convert millivolts into amps

  if (rawDouble<-30.0 || rawDouble>30.0) loadIerror = true;                    // Current sense is out of range, malfunction
  else 
  {
    fram.addRaw(PM_REGISTER_READLOADAMPS, timeStamp, rawAdc);                  // store data
    fram.addDouble(PM_REGISTER_READLOADAMPS, timeStamp, rawDouble);            // store amps value in memory
    if (rawDouble>iLoadHigh - 1.0) hiIwarn = true;                             // Current sense near high limit
    if (rawDouble>iLoadHigh) 
    {
                                   hiIalarm_cnt++;                             // Current sense beyond high limit
                                   hiIalarm = true;                            // set alarm flag
    }               
    else
    {
                                   hiIalarm_cnt=0;                             // reset alarm counter
                                   hiIalarm = false;                           // clear alarm flag
    }
  }

  if (hiIwarn) {
    Serial1.printf("%lu WARN: Load current near threshold (%.2f)\n", timeStamp, rawDouble);
    STATUS0 |= 1<<PM_STATUS0_WARNCURRENT;
    hiIwarn = false; // clear warning
  }

  if (loadIerror) {
    Serial1.printf("%lu: ERROR: Load current sensor out of range (%.2f)\n", timeStamp, rawDouble);
    STATUS0 |= 1<<PM_STATUS0_RANGEISNS;
    loadIerror = false; // clear error
  }

  extAdc.selectCh2();                                                             // vpack divider on adc ch2
  rawAdc    = extAdc.readADC();                                                   // get raw 24-bit value
  rawDouble = (aVcc / (double)16777216) * (double)rawAdc;                         // convert raw reading into millivolts
  rawDouble = rawDouble / vPackDiv;                                               // scale number based on resistor divider ratio

  if (rawDouble<1.0 || rawDouble>20.0) packVerror = true;                         // voltage out of bounds
  else
  { 
    fram.addRaw(PM_REGISTER_READPACKVOLTS, timeStamp, rawAdc);                    // store data
    fram.addDouble(PM_REGISTER_READPACKVOLTS, timeStamp, rawDouble);              // store data

    if (rawDouble<vPackLow)            loValarm_cnt++;                            // increase count for low voltage alarm
    else if (rawDouble>vPackHigh)      hiValarm_cnt++;                            // increase count for high voltage alarm
    else if (rawDouble<vPackLow + 0.25 || rawDouble>vPackHigh - 0.25)
      packVwarn = true;  // voltage near threshold
    else { // no problems, reset alarm counters
      loValarm_cnt = 0;
      hiValarm_cnt = 0;
    }
  }

  if (packVerror) {
    Serial1.printf("%lu: ERROR: Pack voltage sensor out of range. (%.3f)\n", timeStamp, rawDouble);
    STATUS0 |= 1<<PM_STATUS0_RANGEVSNS;
    packVerror = false; // clear error
  }

  if (packVwarn) {
    Serial1.printf("%lu: WARN: Pack voltage near threshold. (%.3f)\n", timeStamp, rawDouble);
    STATUS0 |= 1<<PM_STATUS0_WARNVOLTAGE;
    packVwarn = false; // clear warning
  }

  // *** TODO alarms section
  if (!alarmDisable) {                                                            // user can disable alarms
    if (hiIalarm_cnt>iAlarm_threshold) {
      Serial1.printf("%lu: ALARM: Current exceeding limit.\n", timeStamp);
      hiIalarm_cnt = iAlarm_threshold; // reset alarm
    }

    if (hiTalarm_cnt>tAlarm_threshold ) {        // take action on temperature alarm
      Serial1.printf("%lu: ALARM: Pack temperature exceeding upper limit.\n", timeStamp);
      hiTalarm_cnt = tAlarm_threshold;

    }

    if (loTalarm_cnt>tAlarm_threshold) {        // take action on temperature alarm
      Serial1.printf("%lu: ALARM: Pack temperature exceeding lower limit.\n", timeStamp);
      loTalarm_cnt = tAlarm_threshold;
    }

    if (loValarm_cnt>vAlarm_threshold) { // handle low pack voltage alarm condition
      Serial1.printf("%lu: ALARM: Pack voltage is too low.\n", timeStamp);
      loValarm_cnt = vAlarm_threshold; // reset alarm
    }

    if (hiValarm_cnt>vAlarm_threshold) { // handle high pack voltage alarm condition
      Serial1.printf("%u: ALARM: Pack voltage is too high.\n", timeStamp);
      hiValarm_cnt = vAlarm_threshold; // reset alarm
    }
  }

  // update status registers
  fram.addByte(PM_REGISTER_STATUS0BYTE, timeStamp, STATUS0);
  fram.addByte(PM_REGISTER_STATUS1BYTE, timeStamp, STATUS1);

}

// function that executes whenever data is received from Host
// this function is registered as an event, see setup()
void receiveEvent(size_t howMany) {
  const uint8_t    _isr_dataSize    = 4;             // constant size for byte array to/from i2c, or to/from fram
  uint8_t          _isr_HostByte    = 0;             // byte size data 8 bits
  uint32_t         _isr_HostUlong   = 0;             // unsigned long integer 32 bits
  int32_t          _isr_HostLong    = 0;             // signed long integer 32 bits
  double           _isr_HostDouble  = 0.0;           // double precision aka float, 32 bits
  uint32_t         _isr_timeStamp   = now();         // current timestamp
  
  uint8_t          _isr_cmdAddr     = 0;             // command / register address sent by host
  uint8_t          _isr_dataLen     = 0;             // number of data bytes sent by host
  uint8_t          _isr_byteArray[4];                // storage for four bytes of data of course!
  uint8_t          _isr_cmdData[64];                 // storage for incoming data from host
  union ulongArray  ulongbuffer;                     // convert between byte array and ulong int

  _isr_cmdAddr                      = Wire.read();   // read first byte, store it as command address
  _isr_cmdAddr                     &= 0x7F;          // prevent reading past end of the array, loop address back to zero
  howMany--;                                         // decrement available bytes counter
  recvEvnt                          = true;          // set flag to toggle LED in loop()
  txData.dataLen                    = _isr_dataSize; // set data length here just in case I forgot later
  _I2C_DATA_RDY                     = false;         // nothing to send yet

  if (howMany>0) 
  {
    sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Host writing reg 0x%X byte count %u", _isr_cmdAddr, howMany);
    dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
    dbgMsgCnt++;                                                        // increment debug message counter
  }
  else
  {
    sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Host reading reg 0x%X", _isr_cmdAddr);
    dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
    dbgMsgCnt++;                                                        // increment debug message counter
  }

  while (Wire.available()) {
    _isr_HostByte = (uint8_t) Wire.read();
    // if (_isr_HostByte != 0xFF) 
    _isr_cmdData[_isr_dataLen] = _isr_HostByte; // keep reading until no more bytes available
    _isr_dataLen++;
    // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "RX cmd 0x%X data 0x%X", _isr_cmdAddr, _isr_HostByte);
    // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
    // dbgMsgCnt++;                                                        // increment debug message counter

  }

  // rxData.dataLen           = _isr_dataLen;      // store bytes sent inside struct
  // rxData.cmdAddr           = _isr_cmdAddr;      // store command byte inside struct

  // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "RX cmd 0x%X plus %u data bytes", _isr_cmdAddr, _isr_dataLen);
  // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
  // dbgMsgCnt++;                                                        // increment debug message counter
  
  // events that store or return the same data can be stacked like seen below, 
  // the break command stops the current event
  switch  (_isr_cmdAddr) {
    case 0x21: // get / set high current limit, double
    case 0x22: // get / set high-temp limit, double
    case 0x23: // get / set low-temp limit, double
    case 0x24: // get / set high-voltage limit, double
    case 0x25: // get / set low-voltage limit, double
      if (_isr_dataLen>0) // check if this is a read or write?
      { // more than 0 bytes available, this is a write
        _isr_HostDouble = getDouble(_isr_cmdData);                        // convert byte array into double
        // fram.addDouble(_isr_cmdAddr, _isr_timeStamp, _isr_HostDouble);    // store a double in memory buffer
        fram.addDouble(_isr_cmdAddr, _isr_timeStamp,  _isr_HostDouble);
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "RX cmd 0x%X data %f", _isr_cmdAddr, _isr_HostDouble);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;    
      // }
      // else
      // { // no data was sent, this is a read
      //   memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      //   txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      //   _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      }
      break;

    case 0x26: // get / set config0, byte
    case 0x27: // get / set config1, byte
    case 0x28: // get / set config2, byte
      if (_isr_dataLen>0) // check if this is a read or write?
      // more than 0 bytes available, this is a write
      {
        _isr_HostByte = _isr_cmdData[0];                         // single byte!
        fram.addByte(_isr_cmdAddr, _isr_timeStamp, _isr_HostByte);  // save to buffer
      // }
      // else
      // {
      //   txData.cmdData[0] = fram.getDataByte(_isr_cmdAddr);         // read single byte from buffer
      //   txData.dataLen = 1;                                         // single byte to send
      //   _I2C_DATA_RDY = true;                                       // let loop know data is ready
      }
      break;
    case 0x29: // r/w current sensor mva, double
    case 0x2A: // r/w pack voltage divisor, double
    case 0x2B: // r/w bus voltage divisor, double
    case 0x2E: // r/w thermistor scaling value, double
      if (_isr_dataLen>0) // check if this is a read or write?
      // more than 0 bytes available, this is a write
      {
        _isr_HostDouble = getDouble(_isr_cmdData);                        // convert byte array into double
        // fram.addDouble(_isr_cmdAddr, _isr_timeStamp, _isr_HostDouble);    // store a double in memory buffer
        fram.addDouble(_isr_cmdAddr, _isr_timeStamp,  _isr_HostDouble);
      // }
      // else
      // // no data was sent, this is a read
      // {
      //   memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      //   txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      //   _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      }
      break;
      
    case 0x2C: // read status0, byte (read only)
    case 0x2D: // read status1, byte (read only)
      txData.cmdData[0] = fram.getDataByte(_isr_cmdAddr);         // read single byte from buffer
      txData.dataLen = 1;                                         // single byte to send
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "TX cmd 0x%X data 0x%x", _isr_cmdAddr, txData.cmdData[0]);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;    
      _I2C_DATA_RDY = true;                                       // let loop know data is ready
      break;

    case 0x30: // clear coulomb-counter (write only)
      fram.addDouble(PM_REGISTER_READCOLCNTR, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x31: // clear total amps counter (cmds 33, 34) (write only)
      fram.addDouble(PM_REGISTER_TOTALAMPSIN, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(PM_REGISTER_TOTALAMPSOUT, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x32: // read coulomb counter, double (read only)
    case 0x33: // read instant amps, double (read only)
    case 0x34: // read total amps in counter, double (read only)
    case 0x35: // read total amps out counter, double (read only)
    case 0x36: // read lifetime amps in, double (read only)
    case 0x37: // read lifetime amps out, double (read only)
    case 0x38: // read instant bus voltage, double (read only)
    case 0x39: // read instant pack voltage, double (read only)
    case 0x3A: // read lowest voltage memory, double (read only)
    case 0x3B: // read highest voltage memory, double (read only)
    case 0x3C: // read lowest voltage timestamp, double (read only)  DEPRECATED
    case 0x3D: // read highest voltage timestamp, double (read only) DEPRECATED
      // memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      // _isr_HostDouble = getDouble(fram.getByteArray(_isr_cmdAddr));
      // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "TX cmd 0x%X data %f", _isr_cmdAddr, _isr_HostDouble);
      // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
      // dbgMsgCnt++;

      // txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x3E: // clear voltage memory, (write only)
      fram.addDouble(0x3A, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x3B, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x3C, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x3D, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x40: // clear temperature memories, (write only)
      fram.addDouble(0x42, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x43, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x44, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x45, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x46, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x47, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x48, _isr_timeStamp, 0.0);    // store a double in memory buffer
      fram.addDouble(0x49, _isr_timeStamp, 0.0);    // store a double in memory buffer
      break;

    case 0x41: // read t0 instant, double (read only)
    case 0x42: // read t1 instant, double (read only)
    case 0x43: // read t2 instant, double (read only)
    case 0x44: // read t0 lowest, double (read only)
    case 0x45: // read t1 lowest, double (read only)
    case 0x46: // read t2 lowest, double (read only)
    case 0x47: // read t0 highest, double (read only)
    case 0x48: // read t1 highest, double (read only)
    case 0x49: // read t2 highest, double (read only)
      // dblbuffer.doubleVal = fram.getDataDouble(_isr_cmdAddr);
      // memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "TX %f", dblbuffer.doubleVal);
      // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
      // dbgMsgCnt++;
      // txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      // break;
    case 0x4A: // read t0 low timestamp, unsigned long (read only)    DEPRECATED
    case 0x4B: // read t1 low timestamp, unsigned long (read only)    DEPRECATED
    case 0x4C: // read t2 low timestamp, unsigned long (read only)    DEPRECATED
    case 0x4D: // read t0 high timestamp, unsigned long (read only)   DEPRECATED
    case 0x4E: // read t1 high timestamp, unsigned long (read only)   DEPRECATED
    case 0x4F: // read t1 high timestamp, unsigned long (read only)   DEPRECATED
      // ulongbuffer.longNumber = fram.getDataUInt(_isr_cmdAddr);
      // memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      // txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x50: // clear disconnect history (write only)
      fram.addUInt(0x51, _isr_timeStamp, 0UL);
      fram.addUInt(0x52, _isr_timeStamp, 0UL);
      fram.addUInt(0x53, _isr_timeStamp, 0UL);
      fram.addUInt(0x54, _isr_timeStamp, 0UL);
      fram.addUInt(0x55, _isr_timeStamp, 0UL);
      fram.addUInt(0x56, _isr_timeStamp, 0UL);
      fram.addUInt(0x57, _isr_timeStamp, 0UL);
      break;
    
    case 0x51: // read total over-current disconnects, unsigned int (read only)
    case 0x52: // read total under-voltage discon, unsigned int (read only)
    case 0x53: // read total over-volt discon, uint (read only)
    case 0x54: // read total under-temp discon, uint (read only)
    case 0x55: // read total over-temp discon, uint (read only)
    case 0x56: // read last discon timestamp, ulong (read only) DEPRECATED
      // memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      // txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;

    case 0x57: // read last discon reason code, byte (read only)
      // txData.cmdData[0] = fram.getDataByte(_isr_cmdAddr);         // read single byte from buffer
      // txData.dataLen    = 1;                                      // single byte to send
      // _I2C_DATA_RDY     = true;                                   // let loop know data is ready
      break;

    case 0x60: // set time from Host (write only)
      memcpy(ulongbuffer.byteArray, _isr_cmdData, 4);
      _isr_timeStamp = ulongbuffer.longNumber;

      if (_isr_timeStamp>1000000000) {
        setTime(_isr_timeStamp);                                            // fingers crossed
        HostsetTime = true;                                                 // set flag
        lasttimeSync = _isr_timeStamp;                                      // record timestamp of sync
        if (firsttimeSync==0) firsttimeSync = _isr_timeStamp;               // if it's the first sync, record in separate variable
        
        uint8_t STATUS0 = fram.getDataByte(PM_REGISTER_STATUS0BYTE);        // retrieve status0 byte
        bitSet(STATUS0, PM_STATUS0_TIMESET);                                // set TIMESET bit in status0
        fram.addByte(PM_REGISTER_STATUS0BYTE, _isr_timeStamp, STATUS0);     // write status0 back to buffer

        // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Timestamp %lu", _isr_timeStamp);
        // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        // dbgMsgCnt++;
      } else {
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Error decoding timestamp! %lu", _isr_timeStamp);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;
      }
      break;

    case 0x61: // read first-init timestamp, ulong (read only)
      // memcpy(txData.cmdData, fram.getByteArray(_isr_cmdAddr), _isr_dataSize);  // grab data from memory buffer and copy to tx buffer
      // txData.dataLen = 4;                                                      // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                                    // let loop know data is ready
      break;
    case 0x62: // read current timestamp, ulong (read only)
      // ulongbuffer.longNumber = _isr_timeStamp;
      // memcpy(txData.cmdData, ulongbuffer.byteArray, _isr_dataSize);     // convert to byte array and copy to tx buffer
      // txData.dataLen = 4;                                               // tell requestEvent to send this many bytes
      // fram.addUInt(PM_REGISTER_CURRENTTIME, _isr_timeStamp, _isr_timeStamp);
      // _I2C_DATA_RDY = true;                                             // let loop know data is ready
      break;
    case 0x63: // read time since last sync, ulong (read only)
      // ulongbuffer.longNumber = now() - lasttimeSync;
      // memcpy(txData.cmdData, ulongbuffer.byteArray, _isr_dataSize);          // convert to byte array and copy to tx buffer
      // fram.addUInt(PM_REGISTER_TIMESYNC, _isr_timeStamp, ulongbuffer.longNumber);
      // txData.dataLen = 4;                                               // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                             // let loop know data is ready
      break;
    case 0x64: // read time since boot (uptime), ulong (read only)
      // ulongbuffer.longNumber = now() - firsttimeSync;
      // memcpy(txData.cmdData, ulongbuffer.byteArray, _isr_dataSize);          // convert to byte array and copy to tx buffer
      // fram.addUInt(PM_REGISTER_UPTIME, _isr_timeStamp, ulongbuffer.longNumber);
      // txData.dataLen = 4;                                               // tell requestEvent to send this many bytes
      // _I2C_DATA_RDY = true;                                             // let loop know data is ready
      break;
    case 0x7F: // dump eeprom
      dumpEEprom = true;
      break;
    default:// unknown command
      {
        sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "Unknown command ignored: cmd 0x%X", _isr_cmdAddr);
        dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
        dbgMsgCnt++;                                                        // increment debug message counter
      }
      break;
  } // end switch

} // end of handleEvent

// function that executes whenever data is requested by Host
// this function is registered as an event, see setup()
void requestEvent() {   
  reqEvnt = true;   
  // sprintf(dbgMsgs[dbgMsgCnt].messageTxt, "TX event; i2c_data_rdy=0x%X and %u data bytes", _I2C_DATA_RDY, txData.dataLen);
  // dbgMsgs[dbgMsgCnt].messageNo = dbgMsgCnt;
  // dbgMsgCnt++;                                                        // increment debug message counter

  // if (_I2C_DATA_RDY) {
  //   _I2C_DATA_RDY = false;                              // set flag that we had this interaction
  //   Wire.write((const uint8_t *) txData.cmdData, txData.dataLen);
  // } else {
  //   sprintf((char) txData.cmdData,"Client 0x%X ready!", I2C_CLIENT_ADDR);
  //   Wire.write((char) txData.cmdData);                         // didn't have anything to send? respond with message of 6 bytes
  // }
  // purgeTXBuffer=true;                                // purge TX buffer

  // rather than formatting data in the ISR, send entire register records (24 bytes),
  // and let the host do what it wants with it

  i2cRegAddr &= 0x7F;                                           // wrap pointer if needed
  Wire.write(fram.getArrayData(i2cRegAddr), ee_record_size);    // write register contents to bus
  i2cRegAddr++;                                                 // increment register pointer
  
}

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
    {
      Serial1.println("No I2C devices found");
      delay(5000);
      resetFunc();
    }
  else
    Serial1.println("done.");
 
} // end of scani2c

uint32_t readADC(uint8_t adcPin, uint8_t noSamples) {
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

void clearTXBuffer() {
  uint16_t myPtr = 0;
  while (myPtr < txBufferSize) {
    txData.cmdData[myPtr] = '\0';
    myPtr++;
  }
  purgeTXBuffer = false;
}

double getDouble(uint8_t * byteArray)
{
  union doubleArray buffer;
  const uint8_t dataLen = 4;
  memcpy(buffer.byteArray, byteArray, dataLen);

  return buffer.doubleVal;
}

uint32_t getULong(uint8_t * byteArray)
{
  union ulongArray buffer;
  const uint8_t dataLen = 4;
  memcpy(buffer.byteArray, byteArray, dataLen);

  return buffer.longNumber;
}

int32_t getLong(uint8_t * byteArray)
{
  union ulongArray buffer;
  const uint8_t dataLen = 4;
  memcpy(buffer.byteArray, byteArray, dataLen);

  return buffer.longNumber;
}

void printConfig(){
  double   vBusDiv     = fram.getDataDouble(PM_REGISTER_VBUSDIVISOR);          // grab bus voltage divisor from buffer
  double   hiTemp      = fram.getDataDouble(PM_REGISTER_HIGHTEMPLIMIT);        // grab high temp limit value from buffer
  double   loTemp      = fram.getDataDouble(PM_REGISTER_LOWTEMPLIMIT);         // grab low temp limit value from buffer
  double   vPackDiv    = fram.getDataDouble(PM_REGISTER_VPACKDIVISOR);
  double   vPackLow    = fram.getDataDouble(PM_REGISTER_LOWVOLTLIMIT);
  double   vPackHigh   = fram.getDataDouble(PM_REGISTER_HIGHVOLTLIMIT);
  double   iLoadHigh   = fram.getDataDouble(PM_REGISTER_HIGHCURRENTLIMIT);
  double   mvA         = fram.getDataDouble(PM_REGISTER_CURRENTMVA);
  uint8_t  CONFIG0     = fram.getDataByte(PM_REGISTER_CONFIG0BYTE);
  uint32_t lastSync    = fram.getDataUInt(PM_REGISTER_TIMESYNC);

  Serial1.printf("Last sync %lu\n", lastSync);
  Serial1.printf("CONFIG0 0x%X\n", CONFIG0);
  Serial1.printf("Current limit %f\n", iLoadHigh);
  Serial1.printf("Low voltaget %f\n", vPackLow);
  Serial1.printf("High voltage %f\n", vPackHigh);
  Serial1.printf("Low temp %f\n", loTemp);
  Serial1.printf("High temp %f\n", hiTemp);
  Serial1.printf("vBusDiv %f\n", vBusDiv);
  Serial1.printf("vPackDiv %f\n", vPackDiv);
  Serial1.printf("mvA %f\n", mvA);
}

void framLoad()
{
  for (uint16_t x = 0; x < ee_buffer_size; x++)
  {
    uint8_t byteArray[24];
    ee_fram.readBlock((x * ee_record_size) + ee_start_offset, byteArray, ee_record_size);
    fram.addArrayData(x, byteArray);
    // Serial1.printf("0x%x: ", x);
    // for (int y=0; y<ee_record_size; y++)
    //   Serial1.printf("%x ", byteArray[y]);
    // Serial1.print("\n");
  }
}

// write the buffer into the eeprom
void framSave() 
{
  for (uint16_t x = 0; x < ee_buffer_size; x++)
  {
    ee_fram.writeBlock((x * ee_record_size) + ee_start_offset, fram.getArrayData(x), ee_record_size);
  }
}

void framDump()
{
  Serial1.print("Flush memory buffer to f-ram... ");
  framSave();
  Serial1.println("done.\nRereading f-ram contents...");
  framLoad();
  Serial1.println("done.\nPrinting memory buffer contents...");

  byte xx=0x21; // start here
  while (xx<0x65) 
  {
    Serial1.printf(
      "Addr: 0x%X TS: %lu Double: %f UINT: %lu SINT: %li RAW: %li\n",
      xx,
      fram.getTimeStamp(xx), 
      fram.getDataDouble(xx),
      fram.getDataUInt(xx),
      fram.getDataSInt(xx),
      fram.getRaw(xx)
    );
    xx++;
  }
  Serial1.println("Complete.");
  dumpEEprom = false;
}
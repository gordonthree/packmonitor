# PackMonitor Registers

## Programming registers for packmonitor controller

### These registers are identified by a single command byte and zero or more data bytes

#### 0x00 to 0x20

* (reserved)

#### 0x21 R/W High-current limit, double / float

* Set high current limit in amps, recommended 2 decimal precion
* Example 10.00 (default)

#### 0x22 R/W High-temp limit, double / float

* Set high temperature limit in degrees C, recommend 1 decimal precision
* Example 55.0 (default)

#### 0x23 R/W Low-temp limit, double / float

* Set low temperature limit in degrees C, recommend 1 decimal precision
* Example 10.0 (default)

#### 0x24 R/W Pack high-voltage limit, double / float

* Set pack high voltage disconnect limit, recommend 2 decimal precision
* Example 14.80  (default)

#### 0x25 R/W Pack low-voltage limit, double / float

* valid range 600 to 26000, values outside this range will be ignored
* default 10800 or 10.8 volts
* Send data as char string

#### 0x26 R/W config0 bits (byte) (0 disabled, 1 enabled)

* Bit 7: Disable all protection (default 0)
  * 1: Monitor pack only, ignore all fault conditions
  * 0: Monitor fault conditions as configured (default)
* Bit 6: Over-current protection
  * 1: Disconnect pack when current draw exceeds set-point (default)
  * 0: Disable over-current protection
* Bit 5: Over-temperature protection
  * 1: Disconnect pack when temperature above set-point (default)
  * 0: Ignore low temperature condition
* Bit 4: Under-temperature protection
  * 1: Disconnect pack when temperature below set-point
  * 0: Ignore low temperature condition (default)
* Bit 3: Under-voltage protection
  * 1: Disconnect pack when voltage below set point (default)
  * 0: Ignore under-voltage condition
* Bit 2: Over-voltage protection
  * 1: Disconnect pack when voltage exceeds set point
  * 0: Ignore over-voltage conditions (default)
* Bit 1: (reserved)
* Bit 0: Enable status leds
  * 1: Status LEDs work with button (default)
  * 0: Status LEDs disabled

#### 0x27 R/W config1 bits, return byte

* Bit 0 to 7: (reserved)

#### 0x28 R/W config2 bits, return byte

* Bit 0 to 7: (reserved)

#### 0x29 R/W current sensor mv/A value, double / float

* Read or write the millivolts per amp scaling factor for the current sensor
* Default 0.100 or 100mvA

#### 0x2A R/W pack voltage divider scaling factor, double / float

* Read or write the divisor for pack voltage sensing circuit
* Default 0.33

#### 0x2B R/W bus voltage divider scaling factor, double / float

* Read or write the divisor for the bus voltage sensing circuit
* Default is 1.0
  
#### 0x2C R/O Read status0 bits, return byte

* Bit 7: Config set
* Bit 6: Time set
* Bit 5: Temperature warning (within 2 deg of limits)
* Bit 4: Current warning (within 1 amps of limit)
* Bit 3: Voltage warning (within 250mV of limits)
* Bit 2: T-sense out of range
* Bit 1: I-sense out of range
* Bit 0: V-sense out of range

#### 0x2D R/O Read status1 bits

* Bit 7: (reserved)
* Bit 6: (reserved)
* Bit 5: (reserved)
* Bit 4: (reserved)
* Bit 3: (reserved)
* Bit 2: (reserved)
* Bit 1: V-bus voltage high
* Bit 0: V-bus voltage low
  
#### 0x2E R/W scaling number for thermistor divider circuit

* Read or write the scaling factor / divisor for the thermistor circuits
  
#### 0x2F

* (reserved)

#### 0x30 W/O Clear coulomb counter, no data

* Reset coulomb counter

#### 0x31 R/O Coulomb counter. double / float

* Shows surplus amps in, or deficit amps out

#### 0x32 W/O Clear total amps counter, except lifetime

#### 0x33 Read load amperage, double / float

* Current load in amps

#### 0x34 R/O total pack amps in, double / float

#### 0x35 R/O total pack amps out, double / float

#### 0x36 R/O lifetime amps in, double / float

#### 0x37 R/O lifetime amps out, double / float

#### 0x38 W/O Clear voltage memory, no data

#### 0x39 R/O pack voltage, double / float

* Returns the current pack voltage in volts

#### 0x3A R/O lowest voltage memory, double / float

* Lowest voltage seen since last clear in volts

#### 0x3B R/O lowest voltage timestamp, unsigned long

* Timestamp for when the lowest voltage was recorded

#### 0x3C R/O highest voltage memory, double / float

* Highest voltage seen since last clear in volts

#### 0x3D R/O highest voltage timestamp, unsigned long

* Timestamp for when the highest voltage was recorded

#### 0x3E R/O bus voltage, double / float

* Returns the current bus voltage in volts

#### 0x3F 

* (reserved)

#### 0x40 W/O Clear temperature memories, no data

#### 0x41 R/O Read T0 thermistor instant, double / float

#### 0x42 R/O Read T1 thermistor instant, double / float

#### 0x43 R/O Read T2 thermistor instant, double / float

#### 0x44 R/O Read T0 lowest, double / float

#### 0x45 R/O Read T1 lowest, double / float

#### 0x46 R/O Read T2 lowest, double / float

#### 0x47 R/O Read T0 highest, double / float

#### 0x48 R/O Read T1 highest, double / float

#### 0x49 R/O Read T2 highest, double / float

#### 0x4A R/O T0 lowest memory timestamp, unsigned long

#### 0x4B R/O T1 lowest memory timestamp, unsigned long

#### 0x4C R/O T2 lowest memory timestamp, unsigned long

#### 0x4D R/O T0 highest memory timestamp, unsigned long

#### 0x4E R/O T1 highest memory timestamp, unsigned long

#### 0x4F R/O T2 highest memory timestamp, unsigned long

#### 0x50 W/O Clear disconnect history

#### 0x51 R/O total over-current disconnects unsigned long

#### 0x52 R/O total under-voltage disconnects unsigned long

#### 0x53 R/O total over-voltage disconnects unsigned long

#### 0x54 R/O total under-temp disconnects unsigned long

#### 0x55 R/O total over-temp disconnects unsigned long

#### 0x56 R/O last disconnect timestamp unsigned long

#### 0x57 R/O last disconnect reason code byte

#### 0x58 through 0x5F

* (reserved)

#### 0x60 W/O Set time ulong

* Tranfer time from master to slave
* Expects unix timestamp, aka epoch time, seconds from 1-1-1970

#### 0x61 R/O first-initialized timestamp unsigned long

#### 0x62 R/O current timestamp unsigned long

#### 0x63 R/O time since last sync unsigned long

#### 0x64 Read uptime unsigned long

#### 0x65 through 0xFF

* (reserved)

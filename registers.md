# Programming registers for packmonitor controller

## These registers are identified by a single command byte and one or more data bytes

### 0x20 (reserved)

### 0x21 Set high-current limit (unsigned int)

* Set in milliamps, range 0 to 65535 
* Default is 10000 (10a)

### 0x22 Set high-temp limit (unsigned int)

* Set in millidegrees C, range 0 to 65535 or 65.535c
* Default 45000 or 45c
* Send data as char string

### 0x23 Set low-temp limit (int / signed 16-bit)

* Set in millidegrees C, range -32768 to 32767 or -32.768c to 32.767c
* Default 0c
* Send data as char string

### 0x24 Set pack high-voltage limit in millivolts (unsigned int)

* valid range 9600 to 26000, values outside this range will be ignored
* default 14800
* Send data as char string

### 0x25 Set pack low-voltage limit in millivolts (unsigned int)

* valid range 600 to 26000, values outside this range will be ignored
* default 10800 or 10.8 volts
* Send data as char string

### 0x26 Set config0 bits (byte) (0 disabled, 1 enabled)

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

### 0x27 Set config1 bits (byte)

* Bit 0 to 7: (reserved)

### 0x28 Set config2 bits (byte)

* Bit 0 to 7: (reserved)

### 0x29 Read config0 (byte)

* See set config0 for details
### 0x2A Read config1 (byte)

* See set config1 for details
### 0x2B Read config2 (byte)

* See set config2 for details
### 0x2C Read status0 bites

* Bit 7: Config set
* Bit 6: Time set
* Bit 5: Temperature warning (within 3 deg of limits)
* Bit 4: Current warning (within 1 amps of limit)
* Bit 3: Voltage warning (within 250mV of limits)
* Bit 2: T-sense out of range
* Bit 1: I-sense out of range
* Bit 0: V-sense out of range

### 0x2D Read status1 bits

### 0x2E through 0x2F (reserved)
### 0x30 Clear coulomb counter

### 0x31 Read coulomb counter

### 0x32 Clear total amps counter, except lifetime

### 0x33 Read instant amperage

### 0x34 Read total amps in

### 0x35 Read total amps out

### 0x36 Read lifetime amps in

### 0x37 Read lifetime amps out

### 0x38 Clear voltage memory

### 0x39 Read instant voltage

### 0x3A Read lowest voltage memory, unsigned int

### 0x3B Read lowest voltage timestamp, unsigned long
### 0x3D Read highest voltage memory, unsigned int

### 0x3E Read highest voltage timestamp, unsigned long
### 0x3F (reserved)

### 0x40 Clear temperature memories

### 0x41 Read T0 thermistor

### 0x42 Read T0 lowest

### 0x43 Read T0 highest

### 0x44 Read T1 thermistor

### 0x45 Read T1 lowest

### 0x46 Read T1 highest

### 0x47 through 0x4F (reserved)

### 0x50 Clear disconnect history

### 0x51 Read total over-current disconnects (usigned int)

### 0x52 Read total under-voltage disconnects (usigned int)

### 0x53 Read total over-voltage disconnects (usigned int)

### 0x54 Read total under-temp disconnects (usigned int)

### 0x55 Read total over-temp disconnects (usigned int)

### 0x56 Read last disconnect timestamp (char *)

### 0x57 Read last disconnect reason code (byte)

### 0x58 through 0x5F (reserved)

### 0x60 Set time (char *)

* Tranfer time from master to slave
* Expects unix timestamp sent as char string
### 0x61 Read first-initialized timestamp (char *)

### 0x62 Read current timestamp (char *)

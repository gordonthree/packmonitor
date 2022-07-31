# Programming registers for packmonitor controller

## These registers are identified by a single command byte and one or more data bytes

### 0x20 Set time (char *)

* Tranfer time from master to slave
* Expects unix timestamp sent as char string
### 0x21 Set high-current limit (unsigned word)

* Set in milliamps, range 0 to 65535 
* Default is 10000 (10a)

### 0x22 Set high-temp limit (unsigned word)

* Set in millidegrees C, range 0 to 65535 or 65.535c
* Default 45000 or 45c
* Send data as char string

### 0x23 Set low-temp limit (signed word)

* Set in millidegrees C, range -32768 to 32767 or -32.768c to 32.767c
* Default 0c
* Send data as char string

### 0x24 Set pack high-voltage limit in millivolts (unsigned word)

* valid range 9600 to 26000, values outside this range will be ignored
* default 14800
* Send data as char string

### 0x25 Set pack low-voltage limit in millivolts (unsigned word)

* valid range 600 to 26000, values outside this range will be ignored
* default 10800 or 10.8 volts
* Send data as char string

### 0x26 Set config bits (byte) (0 disabled, 1 enabled)

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

### 0x27 Read config bits (defined above)

### 0x28 Read status bits (0 no 1 yes/ok)

* Bit 7: Config set
* Bit 6: Time set
* Bit 5: Temperature warning (within 3 deg of limits)
* Bit 4: Current warning (within 1 amps of limit)
* Bit 3: Voltage warning (within 250mV of limits)
* Bit 2: T-sense out of range
* Bit 1: I-sense out of range
* Bit 0: V-sense out of range

### 0x29 Clear coulomb counter

### 0x2A Read coulomb counter

### 0x2B Clear total amps counter, except lifetime

### 0x2C Read instant amperage

### 0x2D Read total amps in

### 0x2E Read total amps out

### 0x2F Read lifetime amps in

### 0x30 Read lifetime amps out

### 0x31 Clear voltage memory

### 0x32 Read instant voltage

### 0x33 Read lowest voltage memory

### 0x34 Read highest voltage memory

### 0x35 Clear temperature memories

### 0x36 Read T0 thermistor

### 0x37 Read T0 lowest

### 0x38 Read T0 highest

### 0x39 Read T1 thermistor

### 0x3A Read T1 lowest

### 0x3B Read T1 highest

### 0x3C Clear disconnect history

### 0x3D Read total over-current disconnects

### 0x3E Read total under-voltage disconnects

### 0x3F Read total over-voltage disconnects

### 0x40 Read total under-temp disconnects

### 0x41 Read total over-temp disconnects

### 0x42 Read first-initialized epoch

### 0x43 Read current epochSet low-temp limit -15 to 5c in 0.5c steps, default 0c (see table TBD)

### 0x44 Set pack high-voltage in 10mv steps, range 9.6 to 26.0 (see table TBD)

### 0x45 Set low-voltage limit, 2.50 to 3.00 in 10mV steps, default 2.55v (see table TBD)

### 0x46 Set config bits (0 disabled, 1 enabled)

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

### 0x47 Read config bits (defined above)

### 0x48 Read status bits (0 no 1 yes/ok)

* Bit 7: Config set
* Bit 6: Time set
* Bit 5: Temperature warning (within 3 deg of limits)
* Bit 4: Current warning (within 1 amps of limit)
* Bit 3: Voltage warning (within 250mV of limits)
* Bit 2: T-sense out of range
* Bit 1: I-sense out of range
* Bit 0: V-sense out of range

### 0x49 Clear coulomb counter

### 0x4A Read coulomb counter

### 0x4B Clear total amps counter, except lifetime

### 0x4C Read instant amperage

### 0x4D Read total amps in

### 0x4E Read total amps out

### 0x4F Read lifetime amps in

### 0x50 Read lifetime amps out

### 0x51 Clear voltage memory

### 0x52 Read instant voltage

### 0x53 Read lowest voltage memory

### 0x54 Read highest voltage memory

### 0x55 Clear temperature memories

### 0x56 Read T0 thermistor

### 0x57 Read T0 lowest

### 0x58 Read T0 highest

### 0x59 Read T1 thermistor

### 0x5A Read T1 lowest

### 0x5B Read T1 highest

### 0x5C Clear disconnect history

### 0x5D Read total over-current disconnects

### 0x5E Read total under-voltage disconnects

### 0x5F Read total over-voltage disconnects

### 0x60 Read total under-temp disconnects

### 0x61 Read total over-temp disconnects

### 0x62 Read first-initialized epoch

### 0x63 Read current epoch
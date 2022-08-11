#pragma once

#define PM_REGISTER_HIGHCURRENTLIMIT    0x21 // read / write high current cut off register
#define PM_REGISTER_HIGHTEMPLIMIT       0x22 // read / write high temperature cut off register
#define PM_REGISTER_LOWTEMPLIMIT        0x23 // read / write low temperature cut off register
#define PM_REGISTER_HIGHVOLTLIMIT       0x24 // read / write high voltage cut off register
#define PM_REGISTER_LOWVOLTLIMIT        0x25 // read / writte low voltage cut off register
#define PM_REGISTER_CONFIG0BYTE         0x26 // read / write concig0 register
#define PM_REGISTER_CONFIG1BYTE         0x27 // read / write config1 register
#define PM_REGISTER_CONFIG2BYTE         0x28 // read / write config2 register
#define PM_REGISTER_CURRENTMVA          0x29 // read / write current sensor mva value in millivolts
#define PM_REGISTER_VPACKDIVISOR        0X2A // read / write pack voltage sense divisor
#define PM_REGISTER_VBUSDIVISOR         0x2B // read / write bus voltage sense divisor
#define PM_REGISTER_STATUS0BYTE         0x2C // read status0 register byte
#define PM_REGISTER_STATUS1BYTE         0x2D // read status1 register byte
#define PM_REGISTER_THERMDIVISOR        0x2E // read / write thermistor scaling factor
#define PM_REGISTER_CLEARCOLCNTR        0x30 // clear coulomb counter
#define PM_REGISTER_READCOLCNTR         0x31 // read coulomb counter
#define PM_REGISTER_CLEARAMPSCNTRS      0x32 // clear amperage counters
#define PM_REGISTER_READLOADAMPS        0x33 // display current load amperage
#define PM_REGISTER_TOTALAMPSIN         0x34 // total amps in
#define PM_REGISTER_TOTALAMPSOUT        0x35 // total amps out
#define PM_REGISTER_LIFEAMPSIN          0x36 // display lifetime total charge amps
#define PM_REGISTER_LIFEAMPSOUT         0x37 // display lifetime total discharge amps
#define PM_REGISTER_CLEARVOLTMEM        0x38 // clear voltage record memory
#define PM_REGISTER_READPACKVOLTS       0x39 // read pack voltage right now
#define PM_REGISTER_READLOWVOLTS        0x3A // read low pack voltage record
#define PM_REGISTER_READLOWVOLTSTIME    0x3B // read low pack voltage record timestamp 
#define PM_REGISTER_READHIVOLTS         0x3C // read high pack voltage record
#define PM_REGISTER_READHIVOLTSTIME     0x3D // read high pack voltage record timestamp
#define PM_REGISTER_READBUSVOLTS        0x3E // read bus voltage right now
#define PM_REGISTER_CLEARTEMPS          0x40 // clear temperature record memory
#define PM_REGISTER_READDEGCT0          0x41 // read t0 degrees c
#define PM_REGISTER_READDEGCT1          0x42 // read t1 degrees c
#define PM_REGISTER_READDEGCT2          0x43 // read t2 degrees c
#define PM_REGISTER_READT0LOW           0x44 // read t0 low temp record
#define PM_REGISTER_READT1LOW           0x45 // read t1 low temp record
#define PM_REGISTER_READT2LOW           0x46 // read t2 low temp record
#define PM_REGISTER_READT0HIGH          0x47 // read t0 high temp record
#define PM_REGISTER_READT1HIGH          0x48 // read t1 high temp record
#define PM_REGISTER_READT2HIGH          0x49 // read t2 high temp record
#define PM_REGISTER_READT0LOWTS         0x4A // read t0 low temp record timestamp
#define PM_REGISTER_READT1LOWTS         0x4B // read t1 low temp record timestamp
#define PM_REGISTER_READT2LOWTS         0x4C // read t2 low temp record timestamp
#define PM_REGISTER_READT0HITS          0x4D // read t0 high temp record timestamp
#define PM_REGISTER_READT1HITS          0x4E // read t1 high temp record timestamp
#define PM_REGISTER_READT2HITS          0x4F // read t2 high temp record timestamp
#define PM_REGISTER_CLEARDISCHIST       0x50 // clear disconnect history
#define PM_REGISTER_TOTOVRCURDISC       0x51 // total over current disconnects
#define PM_REGISTER_TOTUNDRVLTDISC      0x52 // total under voltage disconnects
#define PM_REGISTER_TOTOVRVLTDISC       0x53 // total over voltage disconnects
#define PM_REGISTER_TOTLOWRTEMPDISC     0x54 // total low temp disconnects
#define PM_REGISTER_TOTHITEMPDISC       0x55 // total high temp disconnects
#define PM_REGISTER_LASTDISCTIME        0x56 // timestatmp for last disconnect
#define PM_REGISTER_LASTDISCREASON      0x57 // last disconnect reason code
#define PM_REGISTER_SETEPOCHTIME        0x60 // set epoch time
#define PM_REGISTER_FIRSTINITTIME       0x61 // timestamp of last eeprom initilization
#define PM_REGISTER_CURRENTTIME         0x62 // read current epoch time
#define PM_REGISTER_TIMESYNC            0x63 // elapsed time since last sync
#define PM_REGISTER_UPTIME              0x64 // elapsed time since last power-on reset

#define PM_CONFIG0_DISABLEPROTS         0x07 // set to disable all protections, monitor pack only
#define PM_CONFIG0_ENAOVRCURPROT        0x06 // set to enable over-current protection
#define PM_CONFIG0_ENAOVRTMPPROT        0x05 // set to enable over-temp protection
#define PM_CONFIG0_ENAUNDTMPPROT        0x04 // set to enable under-temp protection
#define PM_CONFIG0_EMAUNDVLTPROT        0x03 // set to enable under-voltage protection
#define PM_CONFIG0_ENAOVRVLTPROT        0x02 // set to enable over-voltage protection
#define PM_CONFIG0_ENASTATUSLEDS        0x00 // set to enable status leds if present

#define PM_STATUS0_CONFIGSET            0x07 // set when config0 contains vaild configuration
#define PM_STATUS0_TIMESET              0x06 // set when system clock has been set
#define PM_STATUS0_WARNTEMP             0x05 // set when temperature within 3 degrees of either limit
#define PM_STATUS0_WARNCURRENT          0x04 // set when current within 1 amp of limit
#define PM_STATUS0_WARNVOLTAGE          0x03 // set when voltage within 0.25v of limits
#define PM_STATUS0_RANGETSNS            0x02 // set when temperature sensor value is out of range
#define PM_STATUS0_RANGEISNS            0x01 // set when current sensor value is out of range
#define PM_STATUS0_RANGEVSNS            0x00 // set when voltage sensor value is out of range

#define PM_STATUS1_VBUSLOW              0x00 // set when bus voltage is too low
#define PM_STATUS1_VBUSHIGH             0x01 // set when bus voltage is too high
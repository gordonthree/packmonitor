# Smart battery power monitor project

## A slim circuit board for managing charge and discharge of multi-cell LiFePo4 battery packs. Intended to track state of charge by coulomb counting and prevent battery pack damage from over discharge, over current and over temperature conditions.


## Goals:
1. Monitor and record total amperage in and out
2. Monitor and record temperature, ideally three places
3. Monitor and record pack voltage
4. Provide under-voltage disconnect 
5. Provide over-temperature disconnect

## Features:
1. Hall effect current sensor: TMCS1108
    1. Check existing inventory for ACS7xx sensors!
2. Atmel AVR Micro: ATMEGA4808 TQFP-32
3. FRAM data storage (SPI): FM25640B-G
4. Precision ADC: NAU7802
5. I2C Hot Swap Buffer: TCA4311ADR, PCA9508
6. Max PCB width 18mm length is flexible
7. Precision fixed resistor for thermistors?
8. Power selector circuit/IC? Prefer bus power, accept battery power.
9. Low Iq power supply for battery side, switching or linear?


## Connectors:
1. Battery side
    1. Battery Pos In and Out (14 ga)
    2. Battery Ground (14 ga)
    3. 4-pin jst T0, T1 thermistors
2. Upstream interface: SPI
    4. 6-pin connector of some sort; +5v, SCK, MOSI, MISO, CS, GND

## Indicators: (maybe)
1. Bus power
2. Bus good (from TCA)
3. Battery SOC (4 led) 
4. Battery temp good/bad (dual color?)

## i2c Client Commands:
1. Set time (how many bytes?)
2. Set high-current limit, 0 to 50 in 500ma steps, default 10a
3. Set high-temp limit, 30 to 60c in 1c steps, default 45c
4. Set low-temp limit -15 to 5c in 1c steps, default 0c
5. Set high-voltage limit 3.65 to 4.30 in 10mV steps, default 4.20v
6. Set low-voltage limit, 2.50 to 3.00 in 10mV steps, default 2.55v
7. Set Protection bits
    1. Disable all protection (default 0)
    2. Over-current protection (default 1)
    3. Over-temperature protection (default 1)
    4. Under-temperature protection (default 0)
    5. Under-voltage protection (default 1)
    6. Over-voltage protection (default 0)
    7. (bit 6)
    8. (bit 7)
8. Read status bits (0 no 1 yes/ok)
    1. Config set 
    2. Time set
    3. Temperature warning (within 3 deg of limits)
    4. Current warning (within 1 amps of limit)
    5. Voltage warning (within 250mV of limits)
    6. T-sense out of range
    7. I-sense out of range
    8. V-sense out of range
9. Clear coulomb counter
10. Read coulomb counter
11. Clear total amps counter, except lifetime
12. Read instant amperage
13. Read total amps in
14. Read total amps out
15. Read lifetime amps in
16. Read lifetime amps out
17. Clear voltage memory
18. Read instant voltage
19. Read lowest voltage memory
20. Read highest voltage memory
21. Clear temperature memories
22. Read T0 thermistor
23. Read T0 lowest
24. Read T0 highest
25. Read T1 thermistor
26. Read T1 lowest
27. Read T1 highest
28. Clear disconnect history
29. Read total over-current disconnects
30. Read total under-voltage disconnects
31. Read total over-voltage disconnects
32. Read total under-temp disconnects
33. Read total over-temp disconnects
34. Read initialized epoch
35. Read current epoch

##Questions: 
1. Data logging without a RTC?
2. High side or low side disconnect?
3. No need to track amperage in, assuming an external charger will take care of this?
4. More complicated than it needs to be?

##Resources:

ATMega4808:

[https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA4808-AUR/10444912](https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA4808-AUR/10444912)

[https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf)

[https://github.com/MCUdude/MegaCoreX#pinout](https://github.com/MCUdude/MegaCoreX#pinout)

Thermistor:  

[https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/1753.html](https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/1753.html)

Reading precise voltages:

[https://skillbank.co.uk/arduino/measure.htm](https://skillbank.co.uk/arduino/measure.htm)

Attiny Arduino:

[https://github.com/SpenceKonde/megaTinyCore](https://github.com/SpenceKonde/megaTinyCore)

Bootloader:

[https://github.com/Optiboot/optiboot](https://github.com/Optiboot/optiboot)

FRAM:

[https://www.digikey.com/en/products/detail/cypress-semiconductor-corp/FM25640B-G/3788664](https://www.digikey.com/en/products/detail/cypress-semiconductor-corp/FM25640B-G/3788664)

Hall Effect Sensor:

[https://www.ti.com/product/TMCS1108](https://www.ti.com/product/TMCS1108)

Hot swap buffers:

[https://www.nxp.com/docs/en/data-sheet/PCA9508.pdf](https://www.nxp.com/docs/en/data-sheet/PCA9508.pdf)

https://www.ti.com/general/docs/suppproductinfo.tsp?distId=10&gotoUrl=https%3A%2F%2Fwww.ti.com%2Flit%2Fgpn%2Ftca4311a

Switching regulator (tiny):

[https://www.diodes.com/assets/Datasheets/AP62200_AP62201_AP62200T.pdf](https://www.diodes.com/assets/Datasheets/AP62200_AP62201_AP62200T.pdf)

ADC:

https://www.nuvoton.com/export/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf

# Smart battery power monitor project

### A slim circuit board for managing charge and discharge of multi-cell LiFePo4 battery packs. Intended to track state of charge by coulomb counting and prevent battery pack damage from over discharge, over current and over temperature conditions.

## Goals:

1. Monitor and record total amperage in and out
2. Monitor and record temperature, ideally three places
3. Monitor and record pack voltage
4. Provide under-voltage disconnect
5. Provide over-temperature disconnect

## Features:
1. Atmel AVR Micro: ATMEGA4808 TQFP-32
1. Hall effect current sensor: TMCS1108
   * Check my inventory for ACS7xx sensors!
3. FRAM data storage : i2c FM25640B-G
4. Precision ADC: i2c NAU7802
5. Hot swap buffer: 74HC243
6. Max PCB width 18mm length is flexible
7. Precision fixed resistor for thermistors?
   * Thermistors connect to uC ADC
8. Power selector circuit/IC? Prefer bus power, accept battery power.
9. Low Iq power supply for battery side, switching or linear?
10. SMT button for battiery status

## Connectors:

1. Battery side
   * Battery Pos In and Out (14 ga)
   * Battery Ground (14 ga)
   * 6-pin jst for three thermistors
1. Upstream interface: SPI
   * 6-pin connector of some sort; +5v, SCK, MOSI, MISO, CS, GND
   * 74HC243 quad tri-state buffer for hot insertion protection

## Indicators:

1. Bus controlled:
   * Bus power (from vcc)
   * Bus active (from CS line)
2. uC controlled:
   * Battery SOC (three leds)
   * Battery health (red/green)

## Client Commands:

1. Set time (how many bytes?)
2. Set high-current limit, 0 to 50 in 500ma steps, default 10a (see table TBD)
3. Set high-temp limit, 30 to 60c in 0.5c steps, default 45c (see table TBD)
4. Set low-temp limit -15 to 5c in 0.5c steps, default 0c (see table TBD)
5. Set pack high-voltage in 10mv steps, range 9.6 to 26.0 (see table TBD)
6. Set low-voltage limit, 2.50 to 3.00 in 10mV steps, default 2.55v (see table TBD)
7. Set config bits (0 disabled, 1 enabled)
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
8. Read config bits (defined above)
9. Read status bits (0 no 1 yes/ok)
   * Bit 7: Config set
   * Bit 6: Time set
   * Bit 5: Temperature warning (within 3 deg of limits)
   * Bit 4: Current warning (within 1 amps of limit)
   * Bit 3: Voltage warning (within 250mV of limits)
   * Bit 2: T-sense out of range
   * Bit 1: I-sense out of range
   * Bit 0: V-sense out of range
10. Clear coulomb counter
11. Read coulomb counter
12. Clear total amps counter, except lifetime
13. Read instant amperage
14. Read total amps in
15. Read total amps out
16. Read lifetime amps in
17. Read lifetime amps out
18. Clear voltage memory
19. Read instant voltage
20. Read lowest voltage memory
21. Read highest voltage memory
22. Clear temperature memories
23. Read T0 thermistor
24. Read T0 lowest
25. Read T0 highest
26. Read T1 thermistor
27. Read T1 lowest
28. Read T1 highest
29. Clear disconnect history
30. Read total over-current disconnects
31. Read total under-voltage disconnects
32. Read total over-voltage disconnects
33. Read total under-temp disconnects
34. Read total over-temp disconnects
35. Read first-initialized epoch
36. Read current epoch

## Questions:

1. Data logging without a RTC?
1. High side or low side disconnect?


## Resources:

#### ATMega4808:

* [https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA4808-AUR/10444912](https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA4808-AUR/10444912)
* [https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf)
* [https://github.com/MCUdude/MegaCoreX#pinout](https://github.com/MCUdude/MegaCoreX#pinout)

#### Thermistor:

* [https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/1753.html](https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/1753.html)

#### Readng precise voltages:

[https://skillbank.co.uk/arduino/measure.htm](https://skillbank.co.uk/arduino/measure.htm)

#### ATMEGA Arduino:

[https://github.com/MCUdude/MegaCoreX](https://github.com/MCUdude/MegaCoreX)

#### AVR DA / DB / DD Series:

[https://github.com/SpenceKonde/DxCore](https://github.com/SpenceKonde/DxCore)
#### Bootloader:

[https://github.com/Optiboot/optiboot](https://github.com/Optiboot/optiboot)

#### FRAM:

[https://www.digikey.com/en/products/detail/cypress-semiconductor-corp/FM24C64B-GTR/3788931](https://www.digikey.com/en/products/detail/cypress-semiconductor-corp/FM24C64B-GTR/3788931)

#### Hall Effect Sensor:

[https://www.ti.com/product/TMCS1108](https://www.ti.com/product/TMCS1108)

#### Hot swap buffers:

#### Switching regulator (tiny):

[https://www.diodes.com/assets/Datasheets/AP62200_AP62201_AP62200T.pdf](https://www.diodes.com/assets/Datasheets/AP62200_AP62201_AP62200T.pdf)

#### ADC:

[https://www.nuvoton.com/export/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf](https://www.nuvoton.com/export/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf)

### Logic Buffer:

[https://www.ti.com/product/CD74HCT243#product-detail](https://www.ti.com/product/CD74HCT243#product-detail)
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
3. FRAM data storage : I2C FM24C64B
4. Precision ADC: I2C NAU7802
5. Hot swap buffer: I2C TCA4311a, PCA9508
1. Host side i2c mux: PCA9545A
6. Max PCB width 18mm
7. Precision fixed resistor for thermistors?
   * Thermistors: NRL1504F3950B1F
   * Thermistors connect to uC ADC
8. Power selector IC: AP2411
9. Low Iq power supply for battery side, switching or linear?
10. SMT button for battiery status

## Connectors:

1. Battery side
   * Battery In and Out (14 ga) - WAGO screw cage
   * 6-pin jst-xh vertical for three thermistors
1. Upstream interface: I2C
   * JST-xh 6-pin connector +5v, GND, SCL, GND, SDA, GND
   * TCA4311 on battery side
   * I2C switch on host side (might not need)

## Indicators:

1. Bus controlled:
   * Bus power (from vcc)
   * Bus active (from CS line)
2. uC controlled:
   * Battery SOC (three leds)
   * Battery health (red/green)

## Client Commands:

* See registers.md for addressing and more details

## Questions:

1. Data logging without a RTC?
  * Chip seems to keep time well enough with occasional updates from master
1. High side or low side disconnect?


## Resources:

#### ATMega4808:

* [https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA4808-AUR/10444912](https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA4808-AUR/10444912)
* [https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-09-DataSheet-DS40002173C.pdf)
* [https://github.com/MCUdude/MegaCoreX#pinout](https://github.com/MCUdude/MegaCoreX#pinout)

#### Thermistor:

* [https://www.digikey.com/en/products/detail/eaton-electronics-division/NRL1504F3950B1F/15927588](https://www.digikey.com/en/products/detail/eaton-electronics-division/NRL1504F3950B1F/15927588)
* [https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/1753.html](https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/1753.html)
* [https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html](https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html)

#### Readng precise voltages:

* [https://skillbank.co.uk/arduino/measure.htm](https://skillbank.co.uk/arduino/measure.htm)

#### ATMEGA Arduino:

* [https://github.com/MCUdude/MegaCoreX](https://github.com/MCUdude/MegaCoreX)

#### AVR DA / DB / DD Series:

* [https://github.com/SpenceKonde/DxCore](https://github.com/SpenceKonde/DxCore)
#### Bootloader:

* [https://github.com/Optiboot/optiboot](https://github.com/Optiboot/optiboot)

#### FRAM:

* [FM24C64B](https://www.digikey.com/en/products/detail/cypress-semiconductor-corp/FM24C64B-GTR/3788931)

#### Hall Effect Sensor:

* [https://www.ti.com/product/TMCS1108](https://www.ti.com/product/TMCS1108)

#### Hot swap buffers:

* [https://www.digikey.com/en/products/detail/nxp-usa-inc/PCA9545AD-118/789953](https://www.digikey.com/en/products/detail/nxp-usa-inc/PCA9545AD-118/789953)

* [https://www.ti.com/product/TCA4311A?qgpn=tca4311a](https://www.ti.com/product/TCA4311A?qgpn=tca4311a)
#### Switching regulator (tiny):

* [https://www.diodes.com/assets/Datasheets/AP62200_AP62201_AP62200T.pdf](https://www.diodes.com/assets/Datasheets/AP62200_AP62201_AP62200T.pdf)

#### ADC:

* [https://www.nuvoton.com/export/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf](https://www.nuvoton.com/export/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf)

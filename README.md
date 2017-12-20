# Octopus_BSEC
Octopus Board with ESP8266/BME680 runs Bosch Sensortec BSEC for AQI, indicates on Neopixels and pushes data to ThingSpeak. 
Based on BSEC example by Bosch Sensortec GmbH

Octopus Board, s. here:
https://www.tindie.com/products/FabLab/iot-octopus-badge-for-iot-evaluation/#

The BSEC library is (c) by Robert Bosch GmbH / Bosch Sensortec GmbH and is available here:
https://www.bosch-sensortec.com/bst/products/all_products/bsec

Tested with BSEC_1.4.5.1_Generic_Release_20171214

The BME680 driver is (c) by Robert Bosch GmbH / Bosch Sensortec GmbH and is available here:
https://github.com/BoschSensortec/BME680_driver

Alternative IAQ calculation (c) David Bird

# Installation
According to Application Note BST-BME680-AN008-45.pdf of BSEC documentation

* Download BME680 driver from https://github.com/BoschSensortec/BME680_driver
* Files needed - copy these in the sketch's directory:
  * BME680.c
  * BME680.h
  * BME680_defs.h
  
* Download & extract BSEC library from https://www.bosch-sensortec.com/bst/products/all_products/bsec
* Files needed - copy these in the sketch's directory:
  * bsec_integration.c
  * bsec_integration.h
  * bsec_datatypes.h
  * bsec_interface.h
  
* Add the corresponding library to Arduino. Needed file:
  * BSEC_1.4.5.1_Generic_Release_20171214\algo\bin\esp8266\libalgobsec.a

* Excerpt from BSEC application note, (c) Robert Bosch GmbH:
  * The last step we need to do is to ensure that the pre-build libalgobsec.a library is linked when we compile our project. Unfortunately, this process is somewhat tricky when it comes to Arduino IDE. We first need to find where the board support package for our board is installed. On Windows, this could be for example in <USER_HOME>\AppData\Local\Arduino15\packages\esp8266\hardware or in <ARDUINO_ROOT>\hardware . Once we found the location, we need to perform the following steps. Please keep in mind that the target paths might differ slightly depending on the ESP8266 package version you are using.
  * We need to copy the file algo\bin\esp8266\libalgobsec.a from the BSEC package into the hardware\esp8266\2.3.0\tools\sdk\lib folder.
  * The linker file found at hardware\esp8266\2.3.0\tools\sdk\ld\eagle.app.v6.common.ld needs to be modifed by inserting the line libalgobsec.a:(.literal .text .literal.∗ .text.∗) after the line ∗libm.a:(.literal .text .literal.∗ .text.∗).
  * Finally, we need to change the linker argument, telling the linker to include BSEC. This is achieved by adding the argument -lalgobsec to the line compiler.c.elf.libs=-lm -lgcc ... found in hardware\esp8266\2.3.0\platform.txt.


Don't forget to set Arduino IDE to "Upload Speed" 921600 =)
Output can also be followed via Serial Monitor

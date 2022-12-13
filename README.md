# Vaisala CO2 Controller

### Final project for ARM processors and Embedded Systems operations course.
Created by Metropolia University of Applied Science students, 3rd year, Smart IoT Systems major. <br> Contributors: Manish Subedi, Ahmed Al-Tuwaijari, Melany Macias, Janne Kinnunen. <br> <br>
__More detailed documentation about the project architecture can be found in: [LINK](https://docs.google.com/document/d/1tOOWB005brw4m02iWUmryYpSSYYO36w1/edit#heading=h.gjdgxs)__


# Contents
- [Description](#description)
- [User manual](#user-manual)
  * [Product specifications](#product-specifications)
  * [Intent use](#intent-use)
  * [Installation instructions](#installation-instructions)
  * [How to operate the product?](#product-operation)

# Description
As it is well-known, carbon dioxide supplementation on plant growth and production is a key factor in a healthy greenhouse environment. Carbon dioxide (CO2) is an essential component of photosynthesis (also called carbon assimilation), whose process is of extremely importance in human’s wellbeing as it is the source of energy and food for all organisms. <br> <br>
As there is a need to create a system that helps the healthy growth of plants, especially in enclosed spaces, the project has been developed. Therefore, the project's objective is to create a greenhouse's CO2 fertilization controller system, where the CO2 level is maintained by a controller at the level chosen from the local user interface.  

# User manual
## Product specifications
The hardware used during the project development is shown in the picture below:
![Ventilation System](documentation_pictures/physical%20ventilation%20system.jpg)
                Ventilation System
The system consists of several components:
* LPC1549 (MCU):
LPC1549, an ARM-cortex M3 based microcontroller was used in this project, that features a rich peripheral set with very low power consumption. The LPC1549 operate at CPU frequencies of up to 72 MHz. The LPC1549 include up to 256 kB of flash memory, 32 kB of ROM, a 4 kB EEPROM, and up to 36 kB of SRAM. Among the features, mainly, NVIC, Systick Timer, Watchdog timer, communication protocols: I2C, Modbus, and UART, were used for this project.
* LCD Screen:
The LCD (Liquid Crystal Display) screen is an electronic display module and has a wide range of applications. A 16x2 LCD display is very basic module which is very commonly used in various devices and circuits, as it can display 16 characters per line and there are 2 such lines. It is used to show the current Co2 levels and display the information provided by the sensors.
* Rotary Encoder:
A rotary encoder is a type of position sensor, they measure rotary movements and displacement and can either be absolute or incremental. The systems uses it to get information on position, speed, count or direction that it is placed on, and adjusting the Co2 levels to the user’s input.
* Arduino Uno:
Arduino is an open hardware development board that is used to design and build devices that interact with the real world. During the development process, the Arduino Uno has been used to simulate input from the temperature, humidity and CO2 sensors and its purpose is to serve for basic testing of the software. When the product is used with real sensors, the software behaves in the same way as with the simulator and it does not make use of the Arduino Uno.
* Wifi Module:
The ESP8266 WiFi Module is a self contained SOC with integrated TCP/IP protocol stack that can give any microcontroller access to your WiFi network. Therefore, this module brings to the system the possibility to connect to internet, which is a key factor in the data transmission.<br>

The sensors used to collect data in the embedded system are the following:
* GMP252 co2 probe: Vaisala's GMP252 co2 probe was used to measure and collect information on the co2 concentration. GMP252 is designed for CO2 measurement in demanding applications that require reliable and accurate performance. The measurement range is 0 - 10 000 ppmCO2 (measurements can be carried out in the 10 000 - 30 000 ppmCO2 range with reduced accuracy). It is an intelligent, stand-alone, ppm-level probe. It's intended for measuring CO2 igreenhouses, among other places.
* HMP60: Another one from Vaisala, HMP60 Humidity and Temperature Probe measures and transmittes temperature and humidity readings. HMP60 series probes use the interchangeable Vaisala INTERCAP® sensor. No recalibration is required after sensor replacement. It is a simple, durable and cost-effective humidity probe suitable for greenhouses.

## Intent use
The software’s purpose is to be used in a greenhouse environment, as shown below:
![Physical Greenhouse](documentation_pictures/physical%20greenhouse%20edited.jpg)
                
## Installation instructions
## How to operate the product?

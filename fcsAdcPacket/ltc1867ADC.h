/*This source code has a set of support functions for data acquisition using LTC1867 16-bit ADC Chip 
 * https://www.analog.com/en/products/ltc1867.html
 * Adapted and written for Arduino to be implemented on Teensy 4.1 
 * Author: Pavan Vutukur <pavan.vutukur@oregonstate.edu>
 * Organization: Ocean Mixing Group / College of Earth, Ocean, and Atmoshperic Sciences (CEOAS)/ Oregon State University
 * Start Date: 29 Sep 2022
 */




#ifndef ltc1867ADC_h
#define ltc1867ADC_h
/*
The construct is just a simple way to prevent problems if ever the user adds 
the #include <ltc1867.h> twice on their code.
More info on the tutorial link below to create this library
https://www.teachmemicro.com/create-arduino-library/
*/
/*
Include Arduino and its SPI functions for communication with Teensy 4.1
*/

#include <Arduino.h>
#include <SPI.h>

void spiTransferWord(uint8_t csPin, uint16_t txBuffer, uint16_t *rxBuffer);

void ltc1867ADCSetup(uint8_t csPin);

#endif


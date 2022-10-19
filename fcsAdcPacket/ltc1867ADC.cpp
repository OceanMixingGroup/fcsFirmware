#include <ltc1867ADC.h>
#include <SPI.h>
void spiTransferWord(uint8_t csPin, uint16_t txBuffer, uint16_t *rxBuffer)
{ 
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));  //SPI transaction settings 
  delayMicroseconds(3);                       // Conv time of LTC1867 prior to beginning transaction
  digitalWrite(csPin, LOW);          // LTC1867 CS PIN set LOW to begin transaction
  *rxBuffer = SPI.transfer16(txBuffer);                   // Read & Write simultaneously 16 bits of data from LTC1867
 // *rxBuffer = (int16_t)(SPI.transfer16(txBuffer)) - 0x8000; //convert to int16
  delayMicroseconds(1);                       // T_acq of LTC1867 prior to ending transaction
  digitalWrite(csPin, HIGH);             // Pull LTC1867 CS high
  SPI.endTransaction();
}

void ltc1867ADCSetup(uint8_t csPin)
{
  pinMode(csPin, OUTPUT);    //LTC1867 CS PIN as Output
  digitalWrite(csPin, HIGH); //LTC1867 CS PIN set HIGH as default  
  SPI.begin();        //Begin SPI communication
}

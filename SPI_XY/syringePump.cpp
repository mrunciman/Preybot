#include "syringePump.h"


LinAxis::LinAxis(uint8_t CS_Pin, uint8_t LM_Pin)//:  settingsSPI(1000000, MSBFIRST, SPI_MODE0)
{
    selectPin = CS_Pin;
    pinMode(selectPin, OUTPUT);
    digitalWrite(selectPin, HIGH);
    limitPin = LM_Pin;
    pinMode(selectPin, INPUT_PULLUP);
    dataIn.fData = 0.0;
    dataOut.fData = 0.0;
}



void LinAxis::sendRecvFloat(int pinSS, dataFloat *outData, dataFloat *inData)
{
  // SPI.beginTransaction(settingsSPI);
  // take the select pin low to activate buffer
  digitalWrite(pinSS, LOW);
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO
  firstByte = SPI.transfer(firstOut);
  delayMicroseconds(microDelay);

  firstByte = SPI.transfer(outData->bData[0]);
  delayMicroseconds(microDelay);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->bData[i] = SPI.transfer(outData->bData[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData->bData[i] = SPI.transfer(lastOut);
      delayMicroseconds(microDelay);
    }
    delayMicroseconds(microDelay); // delay between transmissions
  }
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  // SPI.endTransaction();
}



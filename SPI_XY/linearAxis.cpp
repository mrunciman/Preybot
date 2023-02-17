#include "linearAxis.h"


LinAxis::LinAxis():  settingsSPI(500000, MSBFIRST, SPI_MODE3) 
{

}


void LinAxis::init(int CS_Pin, int LM_Pin)
{    
  selectPin = CS_Pin;
  limitPin = LM_Pin;
  pinMode(selectPin, OUTPUT);
  digitalWrite(selectPin, HIGH);
  pinMode(limitPin, INPUT_PULLUP);
  dataIn.fData = 0.0;
  dataOut.fData = 0.0;
}


void LinAxis::sendRecvFloat(dataFloat *outData, dataFloat *inData)
{
  SPI.beginTransaction(settingsSPI);
  // take the select pin low to activate buffer
  digitalWrite(selectPin, LOW);
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO

  // firstByte should be 0 sent by motor at end of SPI interrupt
  firstByte = SPI.transfer(firstOut); 
  delayMicroseconds(microDelay);

  // firstByte will now be 0 from start of SPI interrupt on motor
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
    }
    delayMicroseconds(microDelay); // delay between transmissions
  }
  // take the select pin high to de-select the chip:
  digitalWrite(selectPin, HIGH);
  // delayMicroseconds(microDelay);
  SPI.endTransaction();
  // inData->fData = 6.28;
}



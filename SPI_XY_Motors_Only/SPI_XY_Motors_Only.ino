///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
SPISettings settings0(500000, MSBFIRST, SPI_MODE3);  // At 16 = SPI Clock = 8MHz.

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 9;
const byte selectPinY = 8;
// const byte selectPinZ = 10;


////////////////////////////////////////////////////////
// uStepper setup

int microDelay = 50;

int posX = 0;
int posY = 0;

union dataFloat{
  float fData;
  uint8_t bData[4];
};

dataFloat posEncX;
dataFloat posEncY;
// dataFloat posEncZ;

dataFloat posToX;
dataFloat posToY;
// dataFloat posToZ;

dataFloat pressX;
dataFloat pressY;
// dataFloat pressZ;

byte firstByte = 0;
byte lastByte = 255;

char firstOut = '<';
char lastOut = '>';


////////////////////////////////////////////////////////
// Joystick setup

// Joystick should increment position, not map directly
// Joystick also has button
int joyPinX = A3;
int joyPinY = A4;
int joySwitch = 7;
int joyValueX = 0;
int joyValueY = 0;


void setup() {
  Serial.begin(115200);
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(selectPinX, OUTPUT);
  pinMode(selectPinY, OUTPUT);
  // pinMode(selectPinZ, OUTPUT);
  digitalWrite(selectPinX, HIGH);
  digitalWrite(selectPinY, HIGH);
  // digitalWrite(selectPinZ, HIGH);

  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joySwitch, INPUT_PULLUP);

  
  // initialize SPI:
  SPI.begin();
   
  posToX.fData = 50.0;
  posToY.fData = 51.0;
  // posToZ.fData = 52.0;
  posEncX.fData = 0.0;
  posEncY.fData = 0.0;
  // posEncZ.fData = 0.0;

  Serial.println("Start XY Stage");
}


/////////////////////////////////////////////////////////

void readJoystick(){
  joyValueX = analogRead(joyPinX);
  joyValueY = analogRead(joyPinY);
}


void mapJoystick(){
  int mapX = map(joyValueX, 0, 1023, -512, 512);
  int mapY = map(joyValueY, 0, 1023, -512, 512);

  posX = mapX;
  posY = mapY;

  //TODO: Map these to angles and increment current position
}




void stepExchange(int pinSS, dataFloat* outData, dataFloat* inData){
  // SPI.beginTransaction(settings0);
  // take the select pin low to activate buffer
  digitalWrite(pinSS, LOW);
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO
  firstByte = SPI.transfer(outData->bData[0]);
  delayMicroseconds(microDelay);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->bData[i] = SPI.transfer(outData->bData[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData->bData[i] = SPI.transfer(lastByte);
      delayMicroseconds(microDelay);
    }
    delayMicroseconds(microDelay); // delay between transmissions
  }
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  // SPI.endTransaction();
}


void stepExchange2(int pinSS, dataFloat* outData, dataFloat* inData){
  SPI.beginTransaction(settings0);
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
  SPI.endTransaction();
}


///////////////////////////////////////////////////////////


void loop() {

  // Update joyValX and joyValY
  // readJoystick();
  // // Map joystick to XY plane
  // mapJoystick();
//  Serial.println(digitalRead(joySwitch));
//  Serial.println(posX);
//  Serial.println(posY);
//  Serial.println();
  
  // Send desired positions to steppers
  posToX.fData = posToX.fData + 0.5;
  Serial.print("Desired X position: ");
  Serial.println(posToX.fData);
  stepExchange2(selectPinX, &posToX, &posEncX);
  Serial.print("Measured angle X: ");
  Serial.println(posEncX.fData);
 
  posToY.fData = posToY.fData + 0.5;
  Serial.print("Desired Y position: ");
  Serial.println(posToY.fData);
  stepExchange2(selectPinY, &posToY, &posEncY);
  Serial.print("Measured angle Y: ");   
  Serial.println(posEncY.fData);

  // stepExchange(selectPinZ, &posToZ, &posEncZ);
  // Serial.print("Z: ");   
  // Serial.println(posEncZ.fData);

 
  Serial.println();
  
  delay(1000);

}
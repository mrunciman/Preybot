/*


*/

///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
SPISettings settings0(1000000, MSBFIRST, SPI_MODE0);

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 9;
const byte selectPinY = 10;
const byte selectPinZ = 8;
const byte resetPin = 4;
int microDelay = 50;


////////////////////////////////////////////////////////
// uStepper setup

int posX = 0;
int posY = 0;

union dataFloat{
  float fData;
  uint8_t bData[4];
};

dataFloat posEncX;
dataFloat posEncY;
dataFloat posEncZ;

dataFloat posToX;
dataFloat posToY;
dataFloat posToZ;

dataFloat pressX;
dataFloat pressY;
dataFloat pressZ;

byte firstByte = 0;
byte lastByte = 0;


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
//  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(selectPinX, OUTPUT);
  pinMode(selectPinY, OUTPUT);
  pinMode(selectPinZ, OUTPUT);
  pinMode(resetPin, OUTPUT);
  digitalWrite(selectPinX, HIGH);
  digitalWrite(selectPinY, HIGH);
  digitalWrite(selectPinZ, HIGH);

  //digitalWrite(resetPin, HIGH);
  //digitalWrite(resetPin, LOW);
  //delay(100);
  //digitalWrite(resetPin, HIGH);

  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joySwitch, INPUT_PULLUP);

  
  // initialize SPI:
  SPI.begin();
//   SPCR = 0; 
//   SPCR |= (0<<SPIE)|(0<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
//   SPSR &= ~(0<<SPI2X0);
//   while(!(SPSR & (1<<SPIF))){;}
   

  posToX.fData = 50.0;
  posToY.fData = 51.0;
  posToZ.fData = 52.0;
  posEncX.fData = 0.0;
  posEncY.fData = 0.0;
  posEncZ.fData = 0.0;

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
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  // digitalWrite(SS, LOW);
  digitalWrite(pinSS, LOW);
//  inData->fData = 0.0;
  
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
  // digitalWrite(SS, HIGH);
  SPI.endTransaction();
}


void stepperTwoFloat(int pinSS, dataFloat* outData, dataFloat* inData, dataFloat* pressData){
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  // digitalWrite(SS, LOW);
  digitalWrite(pinSS, LOW);
//  inData->fData = 0.0;
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO
  firstByte = SPI.transfer(outData->bData[0]);
  delayMicroseconds(microDelay);
  for (int i = 0; i < 8; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->bData[i] = SPI.transfer(outData->bData[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData->bData[i] = SPI.transfer(outData->bData[0]);
      delayMicroseconds(microDelay);
    }
    delayMicroseconds(microDelay); // delay between transmissions
  }

  for (int i = 0; i < 8; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      pressData->bData[i] = SPI.transfer(outData->bData[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      pressData->bData[i] = SPI.transfer(lastByte);
      delayMicroseconds(microDelay);
    }
    delayMicroseconds(microDelay); // delay between transmissions
  }

  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  // digitalWrite(SS, HIGH);
  SPI.endTransaction();
}



///////////////////////////////////////////////////////////


void loop() {

  // Update joyValX and joyValY
  // readJoystick();
  // Map joystick to XY plane
  // mapJoystick();
//  Serial.println(digitalRead(joySwitch));
//  Serial.println(posX);
//  Serial.println(posY);
//  Serial.println();
  
  // Send desired positions to steppers
//  stepExchange(selectPinX, &posToX, &posEncX);
  stepExchange(selectPinX, &posToX, &posEncX);
  posToX.fData += 1;
  Serial.println(posToX.fData);

  Serial.print("X: ");
  Serial.println(posEncX.fData);

  Serial.print(posEncX.bData[0], DEC);
  Serial.print('\t');
  Serial.println(posEncX.bData[0], BIN);
  Serial.print(posEncX.bData[1], DEC);
  Serial.print('\t');
  Serial.println(posEncX.bData[1], BIN);
  Serial.print(posEncX.bData[2], DEC);
  Serial.print('\t');
  Serial.println(posEncX.bData[2], BIN);
  Serial.print(posEncX.bData[3], DEC);
  Serial.print('\t');
  Serial.println(posEncX.bData[3], BIN);
  
  Serial.println();

  stepExchange(selectPinY, &posToY, &posEncY);
//  stepExchange(selectPinY, &posToY, &posEncY);
  Serial.println(posToY.fData);

  Serial.print("Y: ");   
  Serial.println(posEncY.fData);

  Serial.println(posEncY.fData);
  Serial.print(posEncY.bData[0], DEC);
  Serial.print('\t');
  Serial.println(posEncY.bData[0], BIN);
  Serial.print(posEncY.bData[1], DEC);
  Serial.print('\t');
  Serial.println(posEncY.bData[1], BIN);
  Serial.print(posEncY.bData[2], DEC);
  Serial.print('\t');
  Serial.println(posEncY.bData[2], BIN);
  Serial.print(posEncY.bData[3], DEC);
  Serial.print('\t');
  Serial.println(posEncY.bData[3], BIN);
  
  Serial.println();

  
//  stepExchange(selectPinZ, &posToZ, &posEncZ);
//  Serial.print("Z: ");   
//  Serial.println(posEncZ.fData);
  
  delay(1000);

}

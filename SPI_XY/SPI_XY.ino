/*


*/

///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
SPISettings settings0(4000000, MSBFIRST, SPI_MODE0);  // At 16 = SPI Clock = 8MHz.

// set pin 10 as the slave select for the X axis, 20 for Y axis
const int p0SelectPin = 53;
const int p1SelectPin = 10;


////////////////////////////////////////////////////////
// uStepper setup

int posX = 0;
int posY = 0;
char charX = posX;
char charY = posY; // Ensures that messages are null terminated
int realX = 0;
byte realY = 0;

union pos_data{
  float posFloat;
  uint8_t posBytes[4];
};
pos_data posEncX;
pos_data posEncY;
pos_data posToX;
pos_data posToY;
byte lastByte = 0;


////////////////////////////////////////////////////////
// Joystick setup

// Joystick should increment position, not map directly
// Joystick also has button
int joyPinX = A3;
int joyPinY = A4;
int joyValueX = 0;
int joyValueY = 0;


void setup() {
  Serial.begin(115200);
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(p0SelectPin, OUTPUT);
  pinMode(p1SelectPin, OUTPUT);
  digitalWrite(p0SelectPin, HIGH);
  digitalWrite(p1SelectPin, HIGH);

  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);

  
  // initialize SPI:
  SPI.begin();
  // Set MOSI and SCK output, all others input
  // DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK);
  // Enable SPI, Master, set clock rate fck/4
//   SPCR = 0; 
//   SPCR |= (0<<SPIE)|(0<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
//  SPSR &= ~(0<<SPI2X0);
  // while(!(SPSR & (1<<SPIF))){;}
   

  posToX.posFloat = 25.0;
  posToY.posFloat = 50.0;
  posEncX.posFloat = 0.0;
  posEncY.posFloat = 0.0;
}


/////////////////////////////////////////////////////////

void readJoystick(){
  joyValueX = analogRead(joyPinX);
  joyValueY = analogRead(joyPinY);
}


void joyMap(){
  int mapX = map(joyValueX, 0, 1023, -512, 512);
  int mapY = map(joyValueX, 0, 1023, -512, 512);

  posX = joyValueX;
  posY = joyValueX;
}


void stepperWrite(int pinSS, pos_data* outData, pos_data* inData){
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  digitalWrite(pinSS, LOW);

  //Send first byte and discard last byte that was sent (lastByte)
  SPI.transfer(outData->posBytes[0]);
  delayMicroseconds (20);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->posBytes[i] = SPI.transfer(outData->posBytes[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData->posBytes[i] = SPI.transfer(lastByte);
    }
//    Serial.print(i);
//    Serial.println(inData->posBytes[i], DEC);
    delayMicroseconds (20);
  }
  
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  SPI.endTransaction;
}

/*
void stepperWrite(int pinSS, union pos_data* outData, union pos_data* inData){
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  digitalWrite(pinSS, LOW);

  //Send first byte and discard last byte that was sent (lastByte)
  SPI.transfer(outData.posBytes[0]);
  delayMicroseconds (20);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData.posBytes[i] = SPI.transfer(outData.posBytes[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData.posBytes[i] = SPI.transfer(lastByte);
    }
    Serial.print(i);
    Serial.println(inData.posBytes[i], DEC);
    delayMicroseconds (20);
  }
  
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  SPI.endTransaction;
}
*/


///////////////////////////////////////////////////////////


void loop() {

  // Update joyValX and joyValY
  readJoystick();
//  Serial.println(joyValueX);
  // Map joystick to XY plane
  joyMap();
  
  // Send desired positions to steppers
  // and read realX and realY from respective enclosures
  stepperWrite(p0SelectPin, &posToX, &posEncX);
  stepperWrite(p1SelectPin, &posToY, &posEncY);

  Serial.print("X: ");
  Serial.println(posEncX.posFloat);
  Serial.print("Y: ");
  Serial.println(posEncY.posFloat);
  Serial.println();
  
  delay(1000);

}

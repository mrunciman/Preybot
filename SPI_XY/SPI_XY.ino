/*

 The circuit:
  * All A pins  of AD5206 connected to +5V
  * All B pins of AD5206 connected to ground
  * An LED and a 220-ohm resisor in series connected from each W pin to ground
  * CS - to digital pin 10  (SS pin)
  * SDI - to digital pin 11 (MOSI pin)
  * CLK - to digital pin 13 (SCK pin)

 created 10 Aug 2010
 by Tom Igoe

 Thanks to Heather Dewey-Hagborg for the original tutorial, 2005

*/

///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
//#include <SPI1.h>
SPISettings settings0(4000000, MSBFIRST, SPI_MODE0);  // At 16 = SPI Clock = 8MHz.

// set pin 10 as the slave select for the X axis, 20 for Y axis
const int p0SelectPin = 10;
const int p1SelectPin = 53;


////////////////////////////////////////////////////////
// uStepper setup
int posX = 0;
int posY = 0;
char charX = posX;
char charY = posY; // Ensures that messages are null terminated
int realX = 0;
byte realY = 0;


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
  pinMode(p0SelectPin, OUTPUT);
  pinMode(p1SelectPin, OUTPUT);

  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);

  
  // initialize SPI:
  SPI.begin();
//  SPI1.begin();
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


void stepperWriteX(int x_pos) {
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  digitalWrite(p0SelectPin, LOW);
  // send two bytes via SPI:
  realX = SPI.transfer(x_pos);
  // take the select pin high to de-select the chip:
  digitalWrite(p0SelectPin, HIGH);
  SPI.endTransaction;
}


byte stepperWriteY(byte y_pos){
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  digitalWrite(p1SelectPin, LOW);
  // send two bytes via SPI:
  realY = SPI.transfer(y_pos);
  delayMicroseconds (20);
  // take the select pin high to de-select the chip:
  digitalWrite(p1SelectPin, HIGH);
  SPI.endTransaction;
  return(realY);
//  y_pos = y_pos +1;
}




///////////////////////////////////////////////////////////


void loop() {

  // Update joyValX and joyValY
  readJoystick();
//  Serial.println(joyValueX);
  // Map joystick to XY plane
  joyMap();
  // Send desired positions to steppers
  // and read realX and realY from respective enclosures
  charY = 50;
//  stepperWriteX(charX);
  realY = stepperWriteY(charY);
  Serial.println(realY, DEC);

}

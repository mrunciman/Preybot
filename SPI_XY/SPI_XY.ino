/*


*/


#include <SPI.h>
#include "trackball.h"

/////////////////////////////////////////////////////////
// Trackball setup

// int SPI_Clock_tBall = 500000;
trackball tBall;


//////////////////////////////////////////////////////////
// SPI setup for uStepperS peripherals
int SPI_Clock_uStep = 1000000;
SPISettings settings0(1000000, MSBFIRST, SPI_MODE3); // Using the hardcoded value made it work

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 10;
const byte selectPinY = 11;
const byte selectPinZ = 12;
const byte resetPin = 8;
int microDelay = 50;


////////////////////////////////////////////////////////
// uStepper data structures

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
  Serial.println("Start XY Stage");
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  // Set trackball pins - done in init function
  // pinMode(PIN_MOUSECAM_RESET_1, OUTPUT);
  // pinMode(PIN_MOUSECAM_RESET_2, OUTPUT);
  // pinMode(PIN_MOUSECAM_CS_1, OUTPUT);
  // pinMode(PIN_MOUSECAM_CS_2, OUTPUT);
  // Set uStepperS pins
  pinMode(selectPinX, OUTPUT);
  pinMode(selectPinY, OUTPUT);
  pinMode(selectPinZ, OUTPUT);
  pinMode(resetPin, OUTPUT);

  // Set CS pins high
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


  posToX.fData = 50.0;
  posToY.fData = 51.0;
  posToZ.fData = 52.0;
  posEncX.fData = 0.0;
  posEncY.fData = 0.0;
  posEncZ.fData = 0.0;

  
  // initialize SPI:
  SPI.begin();
 
  // Initialise trackball
  Serial.println("Initialise cameras...");
  if(tBall.mousecam_init(PIN_MOUSECAM_RESET_1, PIN_MOUSECAM_CS_1)==-1)
  {
    Serial.println("Mouse cam_1 failed to init");
    while(1);
  }  
  if(tBall.mousecam_init(PIN_MOUSECAM_RESET_2, PIN_MOUSECAM_CS_2)==-1)
  {
    Serial.println("Mouse cam_2 failed to init");
    while(1);
  }


}


/////////////////////////////////////////////////////////
// Functions

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
  SPI.endTransaction();
}


void updateTrackball(){
  tBall.mousecam_read_motion(&tBall.md_1, PIN_MOUSECAM_CS_1);
  tBall.mousecam_read_motion(&tBall.md_2, PIN_MOUSECAM_CS_2);  
  tBall.xTranslation();
  tBall.yTranslation();
  tBall.yawAngle();
}


///////////////////////////////////////////////////////////


void loop() {

  // Update joyValX and joyValY
  readJoystick();
  // Map joystick to XY plane
  mapJoystick();

  // Get values from trackball
  updateTrackball();
  Serial.print(tBall.x_mm);     Serial.print("\t");
  Serial.print(tBall.y_mm);     Serial.print("\t");
  Serial.println(tBall.yaw_deg);

  // Send desired positions to respective stepper motors
  posToX.fData = posToX.fData + 1;
  posToY.fData = posToY.fData + 1;
  posToZ.fData = posToZ.fData + 1;
  stepExchange(selectPinX, &posToX, &posEncX);
  stepExchange(selectPinY, &posToY, &posEncY);
  stepExchange(selectPinZ, &posToZ, &posEncZ);
  Serial.println(posEncX.fData);
  Serial.println(posEncY.fData);
  Serial.println(posEncZ.fData);
  Serial.println();
  
  delay(1000);

}

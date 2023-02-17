/*

Turn on
Reset motors
Initialise trackball
Home XY stage and wait
Respond to trackball and joystick

*/


#include <SPI.h>
#include "trackball.h"
#include "linearAxis.h"

unsigned long LOOP_FREQ = 50; // Hz
unsigned long LOOP_PERIOD_MICRO = round(1000000/LOOP_FREQ);

/////////////////////////////////////////////////////////
// Trackball setup

// instantiate trackball object "tBall"
// trackball tBall;


//////////////////////////////////////////////////////////
// Setup pins for uStepperS peripherals
int selectPinX = 9;
int selectPinY = 10;
int selectPinZ = 11;
int selectPinP = 12;
int resetPin1 = 8;
int resetPin2 = 7;
int limitPinX = 4;
int limitPinY = 5;

float PULLEY_RAD = 6.0; // mm

float angPosX = 0.0;
float angPosY = 0.0;
float posX = 0.0;
float posY = 0.0;
int FLOAT_LEN = 9;
int FLOAT_PREC = 2;
char posXStr[10];
char posYStr[10]; 

bool stageHomed = false;

unsigned long timeSinceExec = 0;
unsigned long timeNow;
unsigned long timeLastExecution;


////////////////////////////////////////////////////////
// uStepper data structures

// LinAxis axisList[2] = {LinAxis(selectPinX, limitPinX), LinAxis(selectPinY, limitPinY)};
LinAxis axisList[2] = {LinAxis(), LinAxis()};


////////////////////////////////////////////////////////
// Joystick setup

// Joystick should increment position, not map directly
// Joystick also has button
int joyPinX = A3;
int joyPinY = A4;
int joySwitch = 8;
int joyValueX = 0;
int joyValueY = 0;
bool joyPressed = false;
int joyX = 0;
int joyY = 0;
float deltaJoyX = 0.0;
float deltaJoyY = 0.0;


////////////////////////////////////////////////////////
// Messages
char data[54]; // Char array to write stepNo, pressure and time into
char endByte = 'E';
unsigned long writeTime;


////////////////////////////////////////////////////////
// Setup

void setup() {

  Serial.begin(115200);
  axisList[0].init(selectPinX, limitPinX);
  axisList[1].init(selectPinY, limitPinY);
  Serial.println("Start XY Stage");
  Serial.println(axisList[0].dataIn.fData);
  Serial.println(axisList[1].dataIn.fData);
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  // initialize SPI:
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  pinMode(selectPinZ, OUTPUT);
  pinMode(selectPinP, OUTPUT);
  digitalWrite(selectPinZ, HIGH);
  digitalWrite(selectPinP, HIGH);

  // Set uStepperS reset pin
  pinMode(resetPin1, OUTPUT);
  pinMode(resetPin2, OUTPUT);

  // Reset uSteppers 
  digitalWrite(resetPin1, LOW);
  digitalWrite(resetPin2, LOW);
  delay(100);
  digitalWrite(resetPin1, HIGH);
  digitalWrite(resetPin2, HIGH);

  // Setup Joystick
  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joySwitch, INPUT_PULLUP);

  delay(1000);

  // Initialise trackball cameras
  // tBall.init();

  // Initial encoder reading for position calcs
  // intialPosition();

  // Home stage by hitting X then Y limit switches
  // stageHomed = homeXYStage();

}


/////////////////////////////////////////////////////////
// Functions

// JoyStick //
// Read value of joystick potentiometers
void readJoystick(){
  joyValueX = analogRead(joyPinX);
  joyValueY = analogRead(joyPinY);
  joyPressed = digitalRead(joySwitch)==HIGH;
}


// Map to XY coordinate system
void mapJoystick(){
  int mapX = map(joyValueX, 0, 1023, -512, 512);
  int mapY = map(joyValueY, 0, 1023, -512, 512);

  joyX = mapX;
  joyY = mapY;
  // Convert to 
  deltaJoyX = posX*2;
  deltaJoyY = posY*2;
}


// Trackball //
// void readTrackball(){
//   tBall.mousecam_read_motion(&tBall.md_1, PIN_MOUSECAM_CS_1);
//   tBall.mousecam_read_motion(&tBall.md_2, PIN_MOUSECAM_CS_2);  
//   tBall.xTranslation();
//   tBall.yTranslation();
//   tBall.yawAngle();
// }


// void transformtBall(){
//   tBall.t_dx = tBall.x_mm;
//   tBall.t_dy = tBall.y_mm;
// }


// uSteppers / XY Stage //

void intialPosition(){
  // Read encoder value from each motor. 
  for(auto &item : axisList){
    // For each motor, SPI transfer desired and true angular positions
    item.sendRecvFloat(&item.dataOut, &item.dataIn);
    item.initEncoder = item.dataIn.fData;
  }
}


bool homeXYStage(){
  bool xHomed = false;
  bool yHomed = false;
  Serial.println("Homing X axis...");
  xHomed = homeX();
  Serial.println("Homing Y axis...");
  yHomed = homeY();
  return true; // For test purposes return true
}


// Home X by setting constant dx
bool homeX(){
  float dx = -1.0;
  float dy = 0.0;
  calcAngles(dx, dy); // sets dAngle1 dAngle2
  // read current position of motor, to then decrement it
  while(digitalRead(axisList[0].limitPin) == LOW){  // CHANGE TO HIGH FOR WHEN USING INTERNAL PULLUPS
    moveMotors();
  }
  axisList[0].homeEncoder = axisList[0].dataIn.fData;
  axisList[0].homeOffset = axisList[0].homeEncoder - axisList[0].initEncoder;
  Serial.println(axisList[0].homeOffset);
  dx = 5.0;
  dy = 0.0;
  calcAngles(dx, dy); // sets dAngle1 dAngle2
  moveMotors();
  return true;
}


// Home Y with constant dy
bool homeY(){
  float dx = 0.0;
  float dy = -1.0;
  calcAngles(dx, dy); // sets dAngle1 dAngle2
  // read current position of motor, to then decrement it
  while(digitalRead(axisList[1].limitPin) == LOW){ // CHANGE TO HIGH FOR WHEN USING INTERNAL PULLUPS
    moveMotors();
  }  
  axisList[1].homeEncoder = axisList[1].dataIn.fData;
  axisList[1].homeOffset = axisList[1].homeEncoder - axisList[1].initEncoder;
  Serial.println(axisList[0].homeOffset);
  dx = 0.0;
  dy = 5.0;
  calcAngles(dx, dy); // sets dAngle1 dAngle2
  moveMotors();
  return true;
}


// based on desired change in X and Y positions, find angular position change
void calcAngles(float dx, float dy){
  float delta1;
  float delta2;
  float dAngle1;
  float dAngle2;

  delta1 = dx + dy;
  delta2 = dx - dy;

  dAngle1 = delta1/PULLEY_RAD;
  dAngle2 = delta2/PULLEY_RAD;

  // Serial.println("Angles");
  // Serial.println(dAngle1);
  // Serial.println(dAngle2);

  // Set the changes in angle
  axisList[0].dataOut.fData = dAngle1;
  axisList[1].dataOut.fData = dAngle2;
}


// Calculate XY position of stage from motor angles
void calcPosition(float angle1, float angle2){
  posX = (angle1 - axisList[0].homeOffset)*PULLEY_RAD;  // homeOffsest values unintitialised
  posY = (angle2 - axisList[1].homeOffset)*PULLEY_RAD;
  // Serial.println(posX);
  // Serial.println(posY);
}


void setPosition(float desX, float desY){
  float dx;
  float dy;
  // Find differences
  dx = desX - posX;
  dy = desY - posY;
  //Calculate and set change in angles
  calcAngles(dx, dy);
}


// Move each motor individually
void moveMotors(){
  // Iterate through list of uStepper objects
  for(auto &item : axisList){ 
    // For each motor, SPI transfer desired and true angular positions
    item.sendRecvFloat(&item.dataOut, &item.dataIn);
  }
}


// Send values to serial
void writeSerial(){
  // Write angle data to control computer.
  writeTime = millis();
  // Save float values in char arrays as Arduino can't send them as is
  dtostrf(posX, FLOAT_LEN, FLOAT_PREC, posXStr);
  dtostrf(posY, FLOAT_LEN, FLOAT_PREC, posYStr);
  data[0] ='\0';
  // Send 4 encoder values, three pressure values, time and end byte
  sprintf(data, "%s,%s,%lu,%c\n", posXStr, posYStr, writeTime, endByte);
  Serial.write(data);
}










///////////////////////////////////////////////////////////


void loop() {

  // Check if sampling period has been reached
  timeNow = micros();
  timeSinceExec = timeNow - timeLastExecution;
  if (timeSinceExec >= LOOP_PERIOD_MICRO){
    timeLastExecution = micros();  
  

    // // Update joyValX and joyValY
    // readJoystick();
    // // Map joystick to XY plane
    // mapJoystick();

    // // Get values from trackball
    // readTrackball();
    // Serial.print(tBall.x_mm); Serial.print("\t"); Serial.print(tBall.y_mm); Serial.print("\t"); Serial.println(tBall.yaw_deg);

    // Calc then send desired positions to respective stepper motors
    // calcAngles(tBall.t_dx, tBall.t_dy);
    calcAngles(1, 0);
    // axisList[0].dataOut.fData = 3.14;
    // axisList[1].dataOut.fData = 1.57;
    Serial.println();
    Serial.println(axisList[0].dataOut.fData);
    Serial.println(axisList[1].dataOut.fData);
    moveMotors();

    // Calculate true position based on encoder values
    angPosX = axisList[0].dataIn.fData;
    angPosY = axisList[1].dataIn.fData;
    Serial.println(angPosX);
    Serial.println(angPosY);
    // calcPosition(angPosX, angPosY);

    // Send out over serial
    writeSerial();

    
  }
}

// const PROGMEM uint16_t SineLookup_5bits[32]
// {
//   500, 500, 500, 500, 500, 500, 500, 500,
//   500, 500, 500, 500, 500, 500, 500, 500,
//   500, 500, 500, 500, 500, 500, 500, 500,
//   500, 500, 500, 500, 500, 500, 500, 500
// };
// pgm_read_word(&(SineLookup_5bits[i]))





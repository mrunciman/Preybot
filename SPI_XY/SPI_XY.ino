/*


*/


#include <SPI.h>
#include "trackball.h"
#include "syringePump.h"

/////////////////////////////////////////////////////////
// Trackball setup

// int SPI_Clock_tBall = 500000;
trackball tBall;


//////////////////////////////////////////////////////////
// Setup pins for uStepperS peripherals
const byte selectPinX = 9;
const byte selectPinY = 10;
const byte resetPin = 8;
int microDelay = 50;

float PULLEY_RAD = 6.0; // mm

float dAngle1;
float dAngle2;

////////////////////////////////////////////////////////
// uStepper data structures

LinAxis axisList[2] = {LinAxis(selectPinX), LinAxis(selectPinY)};

// union dataFloat{
//   float fData;
//   uint8_t bData[4];
// };

// dataFloat posEncX;
// dataFloat posEncY;

// dataFloat posToX;
// dataFloat posToY;

// byte firstByte = 0;
// byte lastByte = 255;

// char firstOut = '<';
// char lastOut = '>';


////////////////////////////////////////////////////////
// Joystick setup

// Joystick should increment position, not map directly
// Joystick also has button
int joyPinX = A3;
int joyPinY = A4;
int joySwitch = 7;
int joyValueX = 0;
int joyValueY = 0;
int posX = 0;
int posY = 0;


////////////////////////////////////////////////////////
// Messages
char data[54]; // Char array to write stepNo, pressure and time into
char endByte = 'E';
unsigned long writeTime;


////////////////////////////////////////////////////////
// Setup
void setup() {
  Serial.begin(115200);
  Serial.println("Start XY Stage");
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  // initialize SPI:
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  // Set uStepperS pins
  // pinMode(selectPinX, OUTPUT);
  // pinMode(selectPinY, OUTPUT);
  pinMode(resetPin, OUTPUT);

  // Set CS pins high
  // digitalWrite(selectPinX, HIGH);
  // digitalWrite(selectPinY, HIGH);

  // Reset uSteppers 
  digitalWrite(resetPin, LOW);
  delay(10);
  digitalWrite(resetPin, HIGH);

  // Setup Joystick
  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joySwitch, INPUT_PULLUP);

  // Initialise unions 
  // posToX.fData = 0.0;
  // posToY.fData = 0.0;
  // posEncX.fData = 0.0;
  // posEncY.fData = 0.0;
  

  // Initialise trackball
  Serial.println("Initialise cameras...");
  Serial.print("Camera_1 ");
  if(tBall.mousecam_init(PIN_MOUSECAM_RESET_1, PIN_MOUSECAM_CS_1)==-1)
  {
    Serial.println("Mouse cam_1 failed to init");
    while(tBall.mousecam_init(PIN_MOUSECAM_RESET_1, PIN_MOUSECAM_CS_1)==-1){
      Serial.print("Camera_1");
      delay(500);
    }
  }  
  Serial.print("Camera_2 ");
  if(tBall.mousecam_init(PIN_MOUSECAM_RESET_2, PIN_MOUSECAM_CS_2)==-1)
  {
    Serial.println("Mouse cam_2 failed to init");
    while(tBall.mousecam_init(PIN_MOUSECAM_RESET_2, PIN_MOUSECAM_CS_2)==-1){
      Serial.print("Camera_2");
      delay(500);
    }
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




// void stepExchange2(int pinSS, union dataFloat *outData, union dataFloat *inData){
//   // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
//   // take the select pin low to activate buffer
//   digitalWrite(pinSS, LOW);
  
//   //Send first byte and discard last byte that was sent (lastByte)
//   // 'transfer' firstly sends data on MOSI, then waits and receives from MISO
//   firstByte = SPI.transfer(firstOut);
//   delayMicroseconds(microDelay);

//   firstByte = SPI.transfer(outData->bData[0]);
//   delayMicroseconds(microDelay);
//   for (int i = 0; i < 4; i++){
//     if (i < 3){
//       // Send bytes 2 - 4 and receive bytes 1 -3
//       inData->bData[i] = SPI.transfer(outData->bData[i+1]);
//     }
//     else if (i == 3){
//       // Send placeholder last byte and receive byte 4
//       inData->bData[i] = SPI.transfer(lastOut);
//       delayMicroseconds(microDelay);
//     }
//     delayMicroseconds(microDelay); // delay between transmissions
//   }
//   // take the select pin high to de-select the chip:
//   digitalWrite(pinSS, HIGH);
//   // SPI.endTransaction();
// }


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
  // readJoystick();
  // Map joystick to XY plane
  // mapJoystick();

  // Get values from trackball
  updateTrackball();
  Serial.print(tBall.x_mm);     Serial.print("\t");
  Serial.print(tBall.y_mm);     Serial.print("\t");
  Serial.println(tBall.yaw_deg);

  // Send desired positions to respective stepper motors
  // posToX.fData = tBall.x_mm;
  // posToY.fData = tBall.y_mm;
  // stepExchange2(selectPinX, &posToX, &posEncX);
  // stepExchange2(selectPinY, &posToY, &posEncY);
  // Serial.println(posEncX.fData);
  // Serial.println(posEncY.fData);
  Serial.println();
  
  delay(50);

}



// Turn on, home, and wait
// Respond to trackball and joystick


// Home stage
// In loop:
// Read trackball
// Calculate change in position (transform trackball to change in mm)
// Move XY stage / Send values 


// Functions
// Set sampling interval
// 

void homeXYStage(){
  bool xHomed = false;
  bool yHomed = false;
  // xHomed = homeX();
  // yHomed = homeY();
}


bool homeAxis(int limitPin){
  pinMode(limitPin, INPUT_PULLUP);
  // read current position of motor, to then decrement it
  while(digitalRead(limitPin)==LOW){
    
  }
}

bool homeY(){

}

void writeSerial(){
  // Write angle and pressure data to control computer.
  writeTime = millis();
  data[0] ='\0';
  // Send 4 encoder values, three pressure values, time and end byte
  sprintf(data, "%f,%f,%lu,%c\n", axisList[0].angleIn, axisList[1].angleIn, writeTime, endByte);
  Serial.write(data);
}



int setState(LinAxis& axisObject){
  if (axisObject.calibrated == false){
    axisObject.motorState = CALIBRATING;
  }
  else if (axisObject.calibrated){//some other condition
    axisObject.motorState = ACTIVE;
  }
  return axisObject.motorState;
}



void calcAngles(float dx, float dy){
  float delta1;
  float delta2;

  delta1 = dx + dy;
  delta2 = dx - dy;

  dAngle1 = delta1/PULLEY_RAD;
  dAngle2 = delta2/PULLEY_RAD;
}

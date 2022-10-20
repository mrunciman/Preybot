#include <SPI.h>
#include <math.h>

// ADNS3080 code adapted from:
// Original Code by: Simon Winder
//                   https://github.com/impressivemachines/Arduino
//                   https://www.youtube.com/user/robotbugs
// small changes by: Jan Neumann aka. Neumi
//                   https://github.com/Neumi
//                   https://www.youtube.com/user/NeumiElektronik
// All code is published unter MIT license. Feel free to use!


// these pins may be different on different boards
// this is for the uno
#define PIN_MISO      50
#define PIN_MOSI      51
#define PIN_SCK       52

#define PIN_MOUSECAM_RESET_1     3
#define PIN_MOUSECAM_CS_1        2
#define PIN_MOUSECAM_RESET_2     5
#define PIN_MOUSECAM_CS_2        4

#define ADNS3080_PIXELS_X       30
#define ADNS3080_PIXELS_Y       30

#define ADNS3080_PRODUCT_ID                       0x00
#define ADNS3080_REVISION_ID                      0x01
#define ADNS3080_MOTION                           0x02
#define ADNS3080_DELTA_X                          0x03
#define ADNS3080_DELTA_Y                          0x04
#define ADNS3080_SQUAL                            0x05
#define ADNS3080_PIXEL_SUM                        0x06
#define ADNS3080_MAXIMUM_PIXEL                    0x07
#define ADNS3080_CONFIGURATION_BITS               0x0a
#define ADNS3080_EXTENDED_CONFIG                  0x0b
#define ADNS3080_DATA_OUT_LOWER                   0x0c
#define ADNS3080_DATA_OUT_UPPER                   0x0d
#define ADNS3080_SHUTTER_LOWER                    0x0e
#define ADNS3080_SHUTTER_UPPER                    0x0f
#define ADNS3080_FRAME_PERIOD_LOWER               0x10
#define ADNS3080_FRAME_PERIOD_UPPER               0x11
#define ADNS3080_MOTION_CLEAR                     0x12
#define ADNS3080_FRAME_CAPTURE                    0x13
#define ADNS3080_SROM_ENABLE                      0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER     0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER     0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER     0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER     0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER          0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER          0x1e
#define ADNS3080_SROM_ID                          0x1f
#define ADNS3080_OBSERVATION                      0x3d
#define ADNS3080_INVERSE_PRODUCT_ID               0x3f
#define ADNS3080_PIXEL_BURST                      0x40
#define ADNS3080_MOTION_BURST                     0x50
#define ADNS3080_SROM_LOAD                        0x60

#define ADNS3080_PRODUCT_ID_VAL                   0x17


// //................................... Trackball Parameters ...................................

float x_mm    = 0;
float y_mm    = 0;
float yaw_deg = 0;

float ball_diameter = 50; // mm
// arclength = theta(rad)*radius; theta(rad) = arclength/radius = 2*arclength/ball_diameter; theta(rad)*180/pi = 180*2*arclength/(ball_diameter*pi)
float arc2deg       = 360/(ball_diameter*3.1415);

float cali_x1_pos 	      = 0.0995; // units are mm/sensor
float cali_x1_neg 	      = 0.1005;
float cali_x2_pos 	      = 0.0966;
float cali_x2_neg 	      = 0.0961;

float cali_y1_pos 	      = 0.1005;
float cali_y1_neg 	      = 0.0984;
float cali_y2_pos 	      = 0.0969;
float cali_y2_neg 	      = 0.0966;

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};

// int SPI_Clock = 500000; // For trackball
SPISettings settings_tBall(500000, MSBFIRST, SPI_MODE3);


//...............................uStepperS Stuff................................


//////////////////////////////////////////////////////////
// SPI setup for uStepperS peripherals
// int SPI_Clock_uStep = 1000000;
SPISettings settings0(1000000, MSBFIRST, SPI_MODE3); // Using the hardcoded value made it work

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 8;
const byte selectPinY = 9;
// const byte resetPin = 11;
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

dataFloat posToX;
dataFloat posToY;

dataFloat pressX;
dataFloat pressY;

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




//................................... Setup ...................................

void setup() 
{
  
  //  Serial.begin(9600);
  //  Serial.begin(19200);
  // Serial.begin(38400);
  //  Serial.begin(57600);
  Serial.begin(115200);
  //  Serial.begin(230400);

  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);

  pinMode(selectPinX, OUTPUT);
  pinMode(selectPinY, OUTPUT);
  // pinMode(resetPin, OUTPUT);

  // Set CS pins high
  digitalWrite(selectPinX, HIGH);
  digitalWrite(selectPinY, HIGH);


  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joySwitch, INPUT_PULLUP);

 

  SPI.begin();
  // SPI.setClockDivider(SPI_CLOCK_DIV32);
  // SPI.setDataMode(SPI_MODE3);
  // SPI.setBitOrder(MSBFIRST);
  
  // if(mousecam_init(PIN_MOUSECAM_RESET_1, PIN_MOUSECAM_CS_1)==-1)
  // {
  //   Serial.println("Mouse cam_1 failed to init");
  //   while(1);
  // }  
  // if(mousecam_init(PIN_MOUSECAM_RESET_2, PIN_MOUSECAM_CS_2)==-1)
  // {
  //   Serial.println("Mouse cam_2 failed to init");
  //   while(1);

  posToX.fData = 50.0;
  posToY.fData = 51.0;
  posEncX.fData = 0.0;
  posEncY.fData = 0.0;
  Serial.println("Start XY Stage");
  // }


}

/////////////////////////////////////////////////////////
// Functions
//................................... Functions ...................................

float nfmod(float a,float b)
{
    return a - b * floor(a / b);
}

void mousecam_reset(int resetPin)
{
  digitalWrite(resetPin,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(resetPin,LOW);
  delay(35); // 35ms from reset to functional
}

int mousecam_init(int resetPin, int pinSS)
{
  pinMode(resetPin,OUTPUT);
  pinMode(pinSS,OUTPUT); //PIN_MOUSECAM_CS_1
  digitalWrite(pinSS,HIGH); //PIN_MOUSECAM_CS_1
  mousecam_reset(resetPin); //PIN_MOUSECAM_RESET_1 PIN_MOUSECAM_RESET_2
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID, pinSS);
  Serial.println(pid);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19, pinSS);

  return 0;
}

void mousecam_write_reg(int reg, int val, int pinSS)
{
  // SPI.beginTransaction(settings_tBall);
  digitalWrite(pinSS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(pinSS,HIGH);
  delayMicroseconds(50);
  // SPI.endTransaction();
}

int mousecam_read_reg(int reg, int pinSS)
{
  // SPI.beginTransaction(settings_tBall);
  digitalWrite(pinSS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(pinSS,HIGH); 
  delayMicroseconds(1);
  // SPI.endTransaction();
  return ret;
}


void mousecam_read_motion(struct MD *p, int pinSS)
{
  // SPI.beginTransaction(settings_tBall);
  digitalWrite(pinSS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(pinSS,HIGH); 
  delayMicroseconds(5);
  // SPI.endTransaction();
}



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

/*
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
*/


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





//................................... Loop ...................................


void loop() 
{

  // // read the dx dy data for each sensor
  // MD md_1;
  // mousecam_read_motion(&md_1, PIN_MOUSECAM_CS_1);

  // MD md_2;
  // mousecam_read_motion(&md_2, PIN_MOUSECAM_CS_2);  

  // // write data
  // Serial.print(1);    Serial.print("\t");
  
  // Serial.print((int)md_1.dx);     Serial.print("\t");
  // Serial.print((int)md_1.dy);     Serial.print("\t");

  // Serial.print((int)md_2.dx);     Serial.print("\t");
  // Serial.print((int)md_2.dy);     Serial.print("\t");



  // // x translation mm
  // if ((int)md_1.dy >= 0){
  //   x_mm      = x_mm + ((int)md_1.dy * cali_y1_pos); // x translation
  // }
  // else {
  //   x_mm      = x_mm + ((int)md_1.dy * cali_y1_neg); // x translation
  // }
  
  // // y translation mm
  // if ((int)md_2.dy >= 0){
  //   y_mm      = y_mm + ((int)md_2.dy * cali_y2_pos); // y translation
  // }
  // else{
  //   y_mm      = y_mm + ((int)md_2.dy * cali_y2_neg); // y translation
  // }

  // // yaw rotation deg
  // if ( (int)md_1.dx >= 0 & (int)md_2.dx >= 0 ){
  //   yaw_deg   = nfmod(yaw_deg + 0.5*( ((int)md_1.dx * cali_x1_pos * arc2deg) + ((int)md_2.dx * cali_x2_pos * arc2deg) ), 360.);
  // }
  // else if( (int)md_1.dx >= 0 & (int)md_2.dx < 0 ){
  //   yaw_deg   = nfmod(yaw_deg + 0.5*( ((int)md_1.dx * cali_x1_pos * arc2deg) + ((int)md_2.dx * cali_x2_neg * arc2deg) ), 360.);
  // }
  // else if( (int)md_1.dx < 0 & (int)md_2.dx >= 0 ){
  //   yaw_deg   = nfmod(yaw_deg + 0.5*( ((int)md_1.dx * cali_x1_neg * arc2deg) + ((int)md_2.dx * cali_x2_pos * arc2deg) ), 360.);
  // }
  // else if ( (int)md_1.dx < 0 & (int)md_2.dx < 0 ){
  //   yaw_deg   = nfmod(yaw_deg + 0.5*( ((int)md_1.dx * cali_x1_neg * arc2deg) + ((int)md_2.dx * cali_x2_neg * arc2deg) ), 360.);
  // }


  // Serial.print(x_mm);     Serial.print("\t");
  // Serial.print(y_mm);     Serial.print("\t");
  // Serial.println(yaw_deg);



  // // Update joyValX and joyValY
  // readJoystick();
  // // Map joystick to XY plane
  // mapJoystick();

  // // Send desired positions to respective stepper motors
  posToX.fData = posToX.fData + 1.5;
  Serial.print("Desired X position: ");
  Serial.println(posToX.fData);
  stepExchange2(selectPinX, &posToX, &posEncX);
  Serial.print("Measured angle X: ");
  Serial.println(posEncX.fData);
 
  posToY.fData = posToY.fData + 1.5;
  Serial.print("Desired Y position: ");
  Serial.println(posToY.fData);
  stepExchange2(selectPinY, &posToY, &posEncY);
  Serial.print("Measured angle Y: ");   
  Serial.println(posEncY.fData);
  Serial.println();
  delay(5);


}

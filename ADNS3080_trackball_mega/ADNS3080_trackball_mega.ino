// ADNS3080 code adapted from:
// Original Code by: Simon Winder
//                   https://github.com/impressivemachines/Arduino
//                   https://www.youtube.com/user/robotbugs
// small changes by: Jan Neumann aka. Neumi
//                   https://github.com/Neumi
//                   https://www.youtube.com/user/NeumiElektronik
// All code is published unter MIT license. Feel free to use!

#include <SPI.h>
#include <math.h>
#include "trackball.h"


int SPI_Clock_tBall = 500000;

trackball tBall(SPI_Clock_tBall);

////////////////////////////////////////////////////////////////////

void setup() 
{
  
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  
  SPI.begin();

  Serial.begin(115200);

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

////////////////////////////////////////////////////////////////////////////

void loop() 
{

  // read the dx dy data for each sensor

  tBall.mousecam_read_motion(&tBall.md_1, PIN_MOUSECAM_CS_1);
  tBall.mousecam_read_motion(&tBall.md_2, PIN_MOUSECAM_CS_2);  
  tBall.xTranslation();
  tBall.yTranslation();
  tBall.yawAngle();
  

  // Print data
  Serial.print(1);    Serial.print("\t");
  
  Serial.print((int)tBall.md_1.dx);     Serial.print("\t");
  Serial.print((int)tBall.md_1.dy);     Serial.print("\t");

  Serial.print((int)tBall.md_2.dx);     Serial.print("\t");
  Serial.print((int)tBall.md_2.dy);     Serial.print("\t");


  Serial.print(tBall.x_mm);     Serial.print("\t");
  Serial.print(tBall.y_mm);     Serial.print("\t");
  Serial.println(tBall.yaw_deg);


}

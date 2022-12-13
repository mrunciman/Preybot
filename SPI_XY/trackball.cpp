#include <SPI.h>
#include "trackball.h"


///////////////////////////////////////////////////////////////
// Class definition
trackball::trackball()//   : settings_tBall(500000, MSBFIRST, SPI_MODE3)
{
  x_mm = 0.0;
  y_mm = 0.0;
  yaw_deg =0.0;
}

//////////////////////////////////////////////////////////////
// Class methods
float trackball::nfmod(float a, float b)
{
    return a - b * floor(a / b);
}

void trackball::mousecam_reset(int resetPin)
{
  digitalWrite(resetPin, HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(resetPin, LOW);
  delay(35); // 35ms from reset to functional
}

int trackball::mousecam_init(int resetPin, int pinSS)
{
  pinMode(resetPin, OUTPUT);
  pinMode(pinSS, OUTPUT); //PIN_MOUSECAM_CS_1
  digitalWrite(pinSS, HIGH); //PIN_MOUSECAM_CS_1
  mousecam_reset(resetPin); //PIN_MOUSECAM_RESET_1 PIN_MOUSECAM_RESET_2
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID, pinSS);
  Serial.print(" ID: ");
  Serial.println(pid);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19, pinSS);

  return 0;
}

void trackball::mousecam_write_reg(int reg, int val, int pinSS)
{
  // SPI.beginTransaction(settings_tBall);
  digitalWrite(pinSS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(pinSS, HIGH);
  delayMicroseconds(50);
  // SPI.endTransaction();
}

int trackball::mousecam_read_reg(int reg, int pinSS)
{
  // Serial.print("Register: ");
  // Serial.println(reg);
  // SPI.beginTransaction(settings_tBall);
  digitalWrite(pinSS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(pinSS, HIGH); 
  delayMicroseconds(1);
  // SPI.endTransaction();
  return ret;
}

void trackball::mousecam_read_motion(struct MD *p, int pinSS)
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


float trackball::xTranslation()
{
  // x translation mm
  if (int(md_1.dy) >= 0){
    x_mm = x_mm + (int(md_1.dy) * _cali_y1_pos); // x translation
  }
  else {
    x_mm = x_mm + (int(md_1.dy) * _cali_y1_neg); // x translation
  }
  return x_mm;
}


float trackball::yTranslation()
{
  // y translation mm
  if (int(md_2.dy) >= 0){
    y_mm = y_mm + (int(md_2.dy) * _cali_y2_pos); // y translation
  }
  else{
    y_mm = y_mm + (int(md_2.dy) * _cali_y2_neg); // y translation
  }
  return y_mm;
}


float trackball::yawAngle()
{
  // yaw rotation deg
  if (int(md_1.dx) >= 0 & int(md_2.dx) >= 0 ){
    yaw_deg = nfmod(yaw_deg + 0.5* ((int(md_1.dx) * _cali_x1_pos * _ARC2DEG) + (int(md_2.dx) * _cali_x2_pos * _ARC2DEG) ), 360.);
  }
  else if(int(md_1.dx) >= 0 & int(md_2.dx) < 0 ){
    yaw_deg = nfmod(yaw_deg + 0.5* ((int(md_1.dx) * _cali_x1_pos * _ARC2DEG) + (int(md_2.dx) * _cali_x2_neg * _ARC2DEG) ), 360.);
  }
  else if(int(md_1.dx) < 0 & int(md_2.dx) >= 0 ){
    yaw_deg = nfmod(yaw_deg + 0.5* ((int(md_1.dx) * _cali_x1_neg * _ARC2DEG) + (int(md_2.dx) * _cali_x2_pos * _ARC2DEG) ), 360.);
  }
  else if (int(md_1.dx) < 0 & int(md_2.dx) < 0 ){
    yaw_deg = nfmod(yaw_deg + 0.5* ((int(md_1.dx) * _cali_x1_neg * _ARC2DEG) + (int(md_2.dx) * _cali_x2_neg * _ARC2DEG) ), 360.);
  }
  return yaw_deg;
}

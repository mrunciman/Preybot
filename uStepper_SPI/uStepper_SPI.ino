
#include <uStepperS.h>
// include the SPI library:
#include <SPI.h>
//#include "pins_arduino.h"

byte posFromMegaLSB = 0;
byte posFromMegaMSB = 0;
float encAngle = 0.0;
byte encAngleLSBs = 0;
byte encAngleMSBs = 0;


union pos_from_mega{
  uint8_t posBytes[4];
  float posFloat;
}

pos_from_mega posFromMega;

volatile boolean process_it = false;

uStepperS stepper;

int SS_Pin = 7;


void setup(void)
{
  Serial.begin(115200);
  stepper.setup(CLOSEDLOOP,200);     //Initiate the stepper object to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper 
  
  // For the closed loop position control the acceleration and velocity parameters define the response of the control:
  stepper.setMaxAcceleration(2000);     //use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxVelocity(800);          //Max velocity of 800 fullsteps/s
  
  stepper.checkOrientation(5.0);       //Check orientation of motor connector with +/- 30 microsteps movement
  stepper.setControlThreshold(15);    //Adjust the control threshold - here set to 15 microsteps before making corrective action

  stepper.moveSteps(51200);                 //Turn shaft 51200 steps, counterClockWise (equal to one revolution with the TMC native 1/256 microstepping)


  ///////////////////////////////////////////
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  pinMode(SS_Pin, INPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
//  SPCR = (0<<CPOL)|(0<<CPHA); // SPI on
//  SPCR = (1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(1<<SPR0); // SPI on
  
  // turn on interrupts
  //  SPCR |= _BV(SPIE);
  // now turn on interrupts
  SPI.attachInterrupt();

}


// SPI interrupt routine
ISR (SPI_STC_vect){
  //Read PSI buffer
  posFromMegaMSB = SPDR;
  //  Send to Controller
  SPDR = encAngle;
  process_it = true;
}




void loop(void)
{
//  Serial.println(stepper.encoder.getAngleMoved());    //Print out the current angle of the motor shaft.
  if (digitalRead(SS_Pin) == HIGH){
//    Serial.println(c, DEC);
//    Serial.println(encAngle);
    encAngle = byte(stepper.encoder.getAngleMoved());
    posFromMega.posFloat = stepper.encoder.getAngleMoved();
  }
  
    if (process_it){
      process_it = false;
    }  // end of flag set
}

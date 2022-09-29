
#include <uStepperS.h>
uStepperS stepper;

// include the SPI library:
#include <SPI.h>

int SS_Pin = 3;

union pos_data{
  volatile float posFloat;
  volatile uint8_t posBytes[4];
};

pos_data posEncoder;
pos_data posFromMega;

volatile int posIndex = 0;
volatile byte lastByte = 0;


volatile boolean process_it = false;


void setup(void)
{
  Serial.begin(115200);
  stepper.setup(CLOSEDLOOP,200);     //Initiate the stepper object to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper 
  // Initialise data structures
  posEncoder.posFloat = 655.4; //stepper.encoder.getAngleMoved();
  Serial.println(posEncoder.posFloat);
  Serial.println(posEncoder.posBytes[0], DEC);
  Serial.println(posEncoder.posBytes[1], DEC);
  Serial.println(posEncoder.posBytes[2], DEC);
  Serial.println(posEncoder.posBytes[3], DEC);
  posFromMega.posFloat = 0.0;

  
  // For the closed loop position control the acceleration and velocity parameters define the response of the control:
  stepper.setMaxAcceleration(2000);     //use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxVelocity(800);          //Max velocity of 800 fullsteps/s
  
  stepper.checkOrientation(5.0);       //Check orientation of motor connector with +/- 30 microsteps movement
  stepper.setControlThreshold(15);    //Adjust the control threshold - here set to 15 microsteps before making corrective action

  stepper.moveSteps(51200);                 //Turn shaft 51200 steps, counterClockWise (equal to one revolution with the TMC native 1/256 microstepping)


  ///////////////////////////////////////////
  // have to send on controller in, *peripheral out*
  pinMode(MISO, OUTPUT);
  pinMode(SS_Pin, INPUT);
  
  // turn on SPI in peripheral mode
  SPCR |= bit(SPE);
  
  // turn on interrupts
  SPCR |= bit(SPIE);
  Serial.println("Here");
}


// SPI interrupt routine
ISR (SPI_STC_vect){

  if (digitalRead(SS_Pin) == LOW){
  
    // Check index
    if (posIndex >= 4){
      lastByte = SPDR;
      posIndex = 0;
      process_it = true;
    }
    else{
      //Read byte from SPI buffer
      posFromMega.posBytes[posIndex] = SPDR;
      // Write encoder value to buffer
      SPDR = posEncoder.posBytes[posIndex];
      posIndex++;
    }
  }

}// end of SPI interrupt routine


void loop(void)
{
//  if (digitalRead(SS_Pin) == HIGH){
//    posEncoder.posFloat = stepper.encoder.getAngleMoved();
//  }

  // If end byte was received, process data
  if (process_it){
    Serial.println(posFromMega.posFloat);
    //Serial.println(posEncoder.posFloat);
    // Truncate received data up to pos
    posEncoder.posFloat = stepper.encoder.getAngleMoved();
    process_it = false;
  }
}

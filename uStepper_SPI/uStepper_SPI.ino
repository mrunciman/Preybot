
#include <uStepperS.h>
uStepperS stepper;

const uint8_t SS_Pin = 3;

// char data[20];

union dataFloat{
  float fData;
  uint8_t bData[4];
};

float deltaAngle = 0.0;

float angPos = 0.0;

volatile dataFloat posEncoder;
volatile dataFloat posFromMega;

volatile int posIndex = 0;
volatile uint8_t lastByte = 0;

volatile bool process_it = false;

volatile bool startMessage = false; 
volatile bool endMessage = false; 

volatile int prevSelectState = 1;

int microDelay = 5;


void setup(void)
{
  noInterrupts();
  Serial.begin(115200);
  stepper.setup(CLOSEDLOOP,200);     //Initiate the stepper object to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper 
  // Initialise data structures
  posEncoder.fData = 0.0;// stepper.encoder.getAngleMoved();
  posFromMega.fData = 0.0;
  Serial.println("Start");


  
  // For the closed loop position control the acceleration and velocity parameters define the response of the control:
  stepper.setMaxAcceleration(2000);   //use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxVelocity(800);        //Max velocity of 800 fullsteps/s
  
  stepper.checkOrientation(5.0);      //Check orientation of motor connector with +/- 30 microsteps movement
  stepper.setControlThreshold(15);    //Adjust the control threshold - here set to 15 microsteps before making corrective action


  ///////////////////////////////////////////
  // have to send on controller in, peripheral out
  pinMode(MISO0, OUTPUT);
  pinMode(SS0, INPUT);
  // pinMode(SCK0, INPUT);
  pinMode(SS_Pin, INPUT);
  
  // turn on SPI in peripheral mode, but with no interrupt
  SPCR0 = 0;
  SPCR0 |= (1<<SPIE0)|(1<<SPE0)|(0<<DORD0)|(0<<MSTR0)|(1<<CPOL0)|(1<<CPHA0)|(0<<SPR01)|(1<<SPR00); // SPI_MODE 3
  SPSR0 &= ~(0<<SPI2X0);
  interrupts();
}


// // SPI interrupt routine
// ISR (SPI_STC_vect){
//   if (digitalRead(SS_Pin) == LOW){
//     if (startMessage == false){
//       char c = SPDR0;
//       if (c == '<'){
//         // Serial.println("Start message");
//         startMessage = true;
//       }
//       SPDR0 = 0;
//     }
//     else{ // Have received start byte, so store received data
//       // Check index
//       if (posIndex >= 4){
//         lastByte = SPDR0;
//         if (lastByte == '>'){
//           endMessage = true;
//         }
//         SPDR0 = 0;
//         posIndex = 0;
//         process_it = true;
//       }
//       else if(endMessage == false){ 
//         // Prevent extra bytes being appended if Controller sends
//         // too many.
//         //Read byte from SPI buffer
//         posFromMega.bData[posIndex] = SPDR0;
        
//         // Write encoder value to buffer
//         SPDR0 = posEncoder.bData[posIndex];
//         posIndex++;
//       }
//       else{
//         lastByte = SPDR0;
//         SPDR0 = 0;
//       }
//     }
//   }
// }// end of SPI interrupt routine

// SPI interrupt routine
ISR (SPI_STC_vect){
  if (digitalRead(SS_Pin) == LOW){
    if (startMessage == false){
      char c = SPDR0;
      if (c == '<'){
        startMessage = true;
        posIndex = 0;
      }
      SPDR0 = 0;
    }
    else if(endMessage == false){ // Have received start byte, so store received data
      // Check index
      if(posIndex < 4){ 
        // Prevent extra bytes being appended if Controller sends
        // too many.
        //Read byte from SPI buffer
        posFromMega.bData[posIndex] = SPDR0;
        delayMicroseconds(microDelay);
        // Write encoder value to buffer
        SPDR0 = posEncoder.bData[posIndex];
        // Serial.println(posEncoder.bData[posIndex]);
        // posIndex++;
      }
      else if(posIndex == 4){
        char d = SPDR0;
        if (d == '>'){
          endMessage = true;
        }
        SPDR0 = 0;
        // posIndex = 0;
        // process_it = true;
      }
      else{
        lastByte = SPDR0;
        SPDR0 = 0;
      }
      posIndex++;
    }
  }
}// end of SPI interrupt routine


void loop(void)
{

  // Motor chip select has been deselected
  // if (digitalRead(SS_Pin)==HIGH){
  //   // prevSelectState = 0;
  //   startMessage = false;
  //   endMessage = false;
  //   // posIndex = 0;
  // }

 
  // If end byte was received, process data
  // if (process_it){

  posEncoder.fData = stepper.encoder.getAngleMoved();
  // Serial.println(posEncoder.bData[2], DEC);

  // If both start and end messages received correctly,
  // move to position received from controller
  if (startMessage && endMessage){
    deltaAngle = posFromMega.fData;
    angPos = posEncoder.fData + deltaAngle;
    stepper.moveToAngle(angPos);

    Serial.print("Change in angle: ");  
    Serial.println(deltaAngle);
    Serial.print("From encoder: ");
    Serial.println(posEncoder.fData);
    Serial.println();
    // Clear message flags so that any one message
    // will only move the motor once.
    startMessage = false;
    endMessage = false;
  }
  // process_it = false;
  // }
}

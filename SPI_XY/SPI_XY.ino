/*


*/

///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
SPISettings settings0(4000000, MSBFIRST, SPI_MODE0);  // At 16 = SPI Clock = 8MHz.

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 10;
const byte selectPinY = 9;
const byte selectPinZ = 8;


////////////////////////////////////////////////////////
// uStepper setup

int posX = 0;
int posY = 0;

union pos_data{
  float posFloat;
  uint8_t posBytes[4];
};
volatile pos_data posEncX;
volatile pos_data posEncY;
volatile pos_data posToX;
volatile pos_data posToY;
volatile pos_data posToZ;
volatile pos_data posEncZ;
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
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(selectPinX, OUTPUT);
  pinMode(selectPinY, OUTPUT);
  pinMode(selectPinZ, OUTPUT);
  digitalWrite(selectPinX, HIGH);
  digitalWrite(selectPinY, HIGH);
  digitalWrite(selectPinZ, HIGH);

  pinMode(joyPinX, INPUT);
  pinMode(joyPinY, INPUT);
  pinMode(joySwitch, INPUT_PULLUP);

  
  // initialize SPI:
  SPI.begin();
//   SPCR = 0; 
//   SPCR |= (0<<SPIE)|(0<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
//   SPSR &= ~(0<<SPI2X0);
//   while(!(SPSR & (1<<SPIF))){;}
   

  posToX.posFloat = 50.0;
  posToY.posFloat = 51.0;
  posToZ.posFloat = 52.0;
  posEncX.posFloat = 0.0;
  posEncY.posFloat = 0.0;
  posEncZ.posFloat = 0.0;

  Serial.println("Start XY Stage");
}


/////////////////////////////////////////////////////////

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


void stepperWrite(int pinSS, pos_data* outData, pos_data* inData){
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  digitalWrite(SS, LOW);
  digitalWrite(pinSS, LOW);
  inData->posFloat = 0.0;
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO
  firstByte = SPI.transfer(outData->posBytes[0]);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->posBytes[i] = SPI.transfer(outData->posBytes[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData->posBytes[i] = SPI.transfer(lastByte);
    }
    delayMicroseconds(20); // delay between transmissions
  }
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  digitalWrite(SS, HIGH);
  SPI.endTransaction;
}

/*
void stepperWrite(int pinSS, union pos_data* outData, union pos_data* inData){
  SPI.beginTransaction(settings0);
  // take the select pin low to select the chip:
  digitalWrite(pinSS, LOW);

  //Send first byte and discard last byte that was sent (lastByte)
  SPI.transfer(outData.posBytes[0]);
  delayMicroseconds (20);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData.posBytes[i] = SPI.transfer(outData.posBytes[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData.posBytes[i] = SPI.transfer(lastByte);
    }
    Serial.print(i);
    Serial.println(inData.posBytes[i], DEC);
    delayMicroseconds (20);
  }
  
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  SPI.endTransaction;
}
*/


///////////////////////////////////////////////////////////


void loop() {

  // Update joyValX and joyValY
  readJoystick();
  // Map joystick to XY plane
  mapJoystick();
//  Serial.println(digitalRead(joySwitch));
//  Serial.println(posX);
//  Serial.println(posY);
//  Serial.println();
  
  // Send desired positions to steppers
  stepperWrite(selectPinX, &posToX, &posEncX);
  Serial.print("X: ");
  Serial.println(posEncX.posFloat);
 
  stepperWrite(selectPinY, &posToY, &posEncY);
  Serial.print("Y: ");   
  Serial.println(posEncY.posFloat);

  stepperWrite(selectPinZ, &posToZ, &posEncZ);
  Serial.print("Z: ");   
  Serial.println(posEncZ.posFloat);

  Serial.println(posEncZ.posFloat);
  Serial.print(posEncZ.posBytes[0], DEC);
  Serial.print('\t');
  Serial.println(posEncZ.posBytes[0], BIN);
  Serial.print(posEncZ.posBytes[1], DEC);
  Serial.print('\t');
  Serial.println(posEncZ.posBytes[1], BIN);
  Serial.print(posEncZ.posBytes[2], DEC);
  Serial.print('\t');
  Serial.println(posEncZ.posBytes[2], BIN);
  Serial.print(posEncZ.posBytes[3], DEC);
  Serial.print('\t');
  Serial.println(posEncZ.posBytes[3], BIN);
  
  Serial.println();
  
  delay(1000);

}

#include <SPI.h>

typedef union {
  float fData;
  uint8_t bData[4];
}dataFloat;

class LinAxis{

    #define ACTIVE 0
    #define CALIBRATING 1


///////////////////////////////////////////////////////////////////////////
public:
    // Constructor
    LinAxis(); //uint8_t CS_Pin, uint8_t LM_Pin

    int selectPin;
    int limitPin;

    /////////////////////////////////////////////////////////////////////////
    // Variables for pressure and position data

    // union dataFloat{
    //   float fData;
    //   uint8_t bData[4];
    // };

    dataFloat dataIn;
    dataFloat dataOut;

    // float angleIn;
    float initEncoder = 0.0;
    float homeEncoder = 0.0;
    float homeOffset = 0.0;

    int stepCount = 0;
    int motorState = 0;
    int prevMotorState = 0;

    /////////////////////////////////////////////////////////////////////
    // State setting variables
    bool calibrated = false;
    bool active = false;
    //Use start and end bytes to set state on pumps/peripherals
    char firstOut = '<';
    char lastOut = '>';

    byte firstByte = 0;
    byte lastByte = 255;

    SPISettings settingsSPI;

    int microDelay = 25;

    ////////////////////////////////////////////////////////////////////////////
    // Functions
    void init(int CS_Pin, int LM_Pin);
    void sendRecvFloat(dataFloat *outData, dataFloat *inData);

};
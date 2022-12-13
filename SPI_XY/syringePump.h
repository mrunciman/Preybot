#include <SPI.h>


class LinAxis{

    #define ACTIVE 0
    #define CALIBRATING 1

    int microDelay = 50;

///////////////////////////////////////////////////////////////////////////
public:
    LinAxis(uint8_t CS_Pin, uint8_t LM_Pin);
    uint8_t selectPin;
    uint8_t limitPin;

    /////////////////////////////////////////////////////////////////////////
    // Variables for pressure and position data

    union dataFloat{
        float fData;
        uint8_t bData[4];
    };

    dataFloat dataIn;
    dataFloat dataOut;

    float angleIn;
    float initEncoder;
    float homeEncoder;
    float homeOffset;

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

    ////////////////////////////////////////////////////////////////////////////
    // Functions
    void sendRecvFloat(int pinSS, dataFloat *outData, dataFloat *inData);

};
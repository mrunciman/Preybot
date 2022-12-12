
#include <SPI.h>

// these pins may be different on different boards
// this is for the uno


class trackball{
  // SPI pins on the MEGA
  // #define PIN_MISO      50
  // #define PIN_MOSI      51
  // #define PIN_SCK       52

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
 
  public:
    trackball();

    // const int SPI_Clock = 500000;
     SPISettings settings_tBall;

    // SPISettings settings_tBall(SPI_Clock, MSBFIRST, SPI_MODE3);  

    struct MD{
      byte motion;
      char dx, dy;
      byte squal;
      word shutter;
      byte max_pix;
    };

    float nfmod(float a,float b);

    void mousecam_reset(int resetPin);

    int mousecam_init(int resetPin, int pinSS);
    
    void mousecam_write_reg(int reg, int val, int pinSS);
    
    int mousecam_read_reg(int reg, int pinSS);
    
    void mousecam_read_motion(struct MD *p, int pinSS);

    float xTranslation();

    float yTranslation();

    float yawAngle();

    // Variables
    struct MD md_1;
    struct MD md_2;

    float x_mm;
    float y_mm;
    float yaw_deg;

  private:
    float _BALL_DIAMETER = 50; // mm
    // arclength = theta(rad)*radius; theta(rad) = arclength/radius = 2*arclength/BALL_DIAMETER; theta(rad)*180/pi = 180*2*arclength/(BALL_DIAMETER*pi)
    float _ARC2DEG       = 360/(_BALL_DIAMETER*3.1415);

    float _cali_x1_pos 	      = 0.0995; // units are mm/sensor
    float _cali_x1_neg 	      = 0.1005;
    float _cali_x2_pos 	      = 0.0966;
    float _cali_x2_neg 	      = 0.0961;

    float _cali_y1_pos 	      = 0.1005;
    float _cali_y1_neg 	      = 0.0984;
    float _cali_y2_pos 	      = 0.0969;
    float _cali_y2_neg 	      = 0.0966;
};

#ifndef HMC5883L_h
#define HMC5883L_h
#include <Wire.h>
#include "Arduino.h"
#include "Vector.h"

#define HMC5883L_DEVICE 0x1E      //device address

#define HMC5883L_CONF_REG_A 0x00
#define HMC5883L_CONF_REG_B 0x01

#define HMC5883L_MODE 0x02

#define HMC5883L_OUT_X_MSB 0x03
#define HMC5883L_OUT_X_LSB 0x04

#define HMC5883L_OUT_Z_MSB 0x05
#define HMC5883L_OUT_Z_LSB 0x06

#define HMC5883L_OUT_Y_MSB 0x07
#define HMC5883L_OUT_Y_LSB 0x08

#define HMC5883L_TO_READ 6

#define HMC5883L_STATUS 0x09
#define HMC5883L_ID_REG_A 0x0A
#define HMC5883L_ID_REG_B 0x0B
#define HMC5883L_ID_REG_C 0x0C



typedef enum
  {
    HMC5883L_IDLE          = 0b10,
    HMC5883L_SINGLE        = 0b01,
    HMC5883L_CONTINUOUS     = 0b00
  } hmc5883l_mode_t;

typedef enum
  {
   HMC5883L_SAMPLES_8     = 0b11,
   HMC5883L_SAMPLES_4     = 0b10,
   HMC5883L_SAMPLES_2     = 0b01,
    HMC5883L_SAMPLES_1     = 0b00
    } hmc5883l_samples_t;

typedef enum
  {
    HMC5883L_DATARATE_75HZ       = 0b110,
    HMC5883L_DATARATE_30HZ       = 0b101,
    HMC5883L_DATARATE_15HZ       = 0b100,
    HMC5883L_DATARATE_7_5HZ      = 0b011,
    HMC5883L_DATARATE_3HZ        = 0b010,
    HMC5883L_DATARATE_1_5HZ      = 0b001,
    HMC5883L_DATARATE_0_75_HZ    = 0b000
  } hmc5883l_dataRate_t;

typedef enum
  {
    HMC5883L_RANGE_8_1GA     = 0b111,
    HMC5883L_RANGE_5_6GA     = 0b110,
    HMC5883L_RANGE_4_7GA     = 0b101,
    HMC5883L_RANGE_4GA       = 0b100,
    HMC5883L_RANGE_2_5GA     = 0b011,
    HMC5883L_RANGE_1_9GA     = 0b010,
    HMC5883L_RANGE_1_3GA     = 0b001,
    HMC5883L_RANGE_0_88GA    = 0b000
  } hmc5883l_range_t;


class HMC5883L
{
 public:
  HMC5883L();
  bool init();

  void calibrate();
  

  void setRange(hmc5883l_range_t range);
  void setMeasurementMode(hmc5883l_mode_t mode);
  void setDataRate(hmc5883l_dataRate_t dataRate);
  void setSamples(hmc5883l_samples_t samples);

  hmc5883l_range_t getRange();
  hmc5883l_mode_t getMeasurementMode();
  hmc5883l_dataRate_t getDataRate();
  hmc5883l_samples_t getSamples();

  void setOffset(int xoffset, int yoffset);
  
  Vector readCompass();
  Vector readRaw();
  
  void printAllRegister();
  void print_byte(byte val);
  void printCalibrationValues(int samples);
  void writeTo(byte address, byte val);
  void readFrom(byte address, int num, byte _buff[]);
 private:
  // this is an array with 6 bytes (3 ints!!)
  // this allows us to store the information from compass readings
  // since each reading comes in as 2 bytes
  byte _buff[6];
  float mgPerDigit;
  double x_offset, y_offset;

  
};

#endif


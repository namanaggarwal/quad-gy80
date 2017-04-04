#ifndef BMP085
#define BMP085
#include <Wire.h>
#include "Arduino.h"

#define BMP085_DEVICE 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3

/* Calibration Data */
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)


#define BMP085_CONTROL           0xF4

#define BMP085_TEMPERATURE_READ  0x2E
#define BMP085_TEMPERATUREDATA   0xF6

#define BMP085_PRESSURE_READ     0x34
#define BMP085_PRESSUREDATA      0xF6

class BMP085 {
 public:
  BMP085();
  bool init(uint8_t mode = BMP085_ULTRAHIGHRES);
  float readTemperature();
  int32_t readPressure();
  int32_t readSeaLevelPressure(float altitude_meters = 0);
  float readAltitude(float selevelPressure = 101325);
  uint16_t readRawTemp();
  uint32_t readRawPressure();

 private:
  int32_t computeB5(int32_t UT);
  int16_t readRegister16(byte address);
  void writeRegister(byte data, byte address);

  uint8_t oversample;

  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;
};

#endif

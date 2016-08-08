#include "BMP085.h"

BMP085::BMP085() {}

bool BMP085::init(uint8_t mode) {
  // set device mode
  if (mode > BMP085_ULTRAHIGHRES) mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;
  //start communication
  Wire.begin();

  // check to make sure we have a BMP085 on board
  if(readRegister8(0xD0) != 0x55) return false;

  // get calibration data
  ac1 = readRegister16(BMP085_CAL_AC1);
  ac2 = readRegister16(BMP085_CAL_AC2);
  ac3 = readRegister16(BMP085_CAL_AC3);
  ac4 = readRegister16(BMP085_CAL_AC4);
  ac5 = readRegister16(BMP085_CAL_AC5);
  ac6 = readRegister16(BMP085_CAL_AC6);

  b1 = readRegister16(BMP085_CAL_B1);
  b2 = readRegister16(BMP085_CAL_B2);

  mb = readRegister16(BMP085_CAL_MB);
  mc = readRegister16(BMP085_CAL_MC);
  md = readRegister16(BMP085_CAL_MD);

  return true;
}

int BMP085::computeB5(int32_t UT) {
  // see datasheet for more
  int32_t X1 = ((UT - ac6) * ((int32_t)ac5 >> 15));
  int32_t X2 = +  ((int32_t)mc << 11)/(X1 + md);
  return X1 + X2;
}


//switched write parameters
float BMP085::readTemperature() {
  writeRegister8(BMP085_TEMPERATURE_READ, BMP085_CONTROL);
  uint16_t data = readRegister16(BMP085_TEMPERATUREDATA);
  return data;
}

int32_t BMP085::readPressure() {
  writeRegister(BMP085_PRESSURE_READ, BMP085_CONTROL);
  uint16_t data = readRegister16(BPM085_PRESSUREDATA);
  byte data2 = readRegister8(

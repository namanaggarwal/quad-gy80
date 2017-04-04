#ifndef NAVI_h
#define NAVI_h

#include "ADXL345.h"
#include "L3G400D.h"
#include "HMC5843.h"
#include "Kalman.h"


class Navi
{
 public:
  Navi();
  void init();
  void UpdateState();

 private:
  ADXL345 accel;
  L3G4200D gyro;
  HMC5883L compass;
  BMP085 alti;

  float state[7];


#endif

#ifndef NAVI_h
#define NAVI_h

#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"
//#include "Kalman.h"
//#include "Arduino.h"

class Navi
{
 public:
  Navi(ADXL345* accelerometer, L3G4200D* gyroscope, HMC5883L* magnetometer);
  bool init();
  void UpdateState();
  void printSensorInfo();
 private:
  ADXL345* accel;
  L3G4200D* gyro;
  HMC5883L* compass;
  //BMP085 alti;

  Vector a;
  Vector g;
  Vector c;
  Vector i;
  
  float state[7];
};

#endif

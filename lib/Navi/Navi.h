#ifndef NAVI_h
#define NAVI_h

#include "matrix.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "Kalman.h"


class Navi
{
 public:
  Navi();
  bool init();
  bool UpdateState();
  Matrix getState();
  Kalman getKalman();
  void printSensorInfo();
  
 private:
  ADXL345 accel;
  L3G4200D gyro;
  Kalman myKalman;
  //mmHMC5883L* compass;
  //BMP085 alti;
  
  float state[7];
  Matrix angle = Matrix(3,1);
  //Matrix linear(1,3);
};

#endif

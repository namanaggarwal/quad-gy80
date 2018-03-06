#ifndef L3G4200D_h
#define L3G4200D_h

#include <Wire.h>
#include "Arduino.h"
#include "Sensor.h"

#define L3G4200D_DEVICE 105
#define L3G4200D_BYTES_READ 6

#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_CTRL_REG2 0x21
#define L3G4200D_CTRL_REG3 0x22
#define L3G4200D_CTRL_REG4 0x23
#define L3G4200D_CTRL_REG5 0x24

#define L3G4200D_OUT_X_L 0x28
#define L3G4200D_OUT_X_H 0x29
#define L3G4200D_OUT_Y_L 0x2A
#define L3G4200D_OUT_Y_H 0x2B
#define L3G4200D_OUT_Z_L 0x2C
#define L3G4200D_OUT_Z_H 0x2D

#define ALPHA_G 0.3


class L3G4200D : public Sensor
{
 public:
  L3G4200D();
  
  void Update();
  Matrix getAngularVelocity();
  Matrix getAngularVelocity2x1();
  Matrix getAngularAcceleration();
  Matrix getAngularAcceleration2x1();
  Matrix getAngularPosition();
  Matrix getAngularPosition2x1();
  Matrix getKalmanInput();
  Matrix getKalmanInput4x1();
  
  
  
  bool init(double xoffset=0, double yoffset=0, double zoffset=0);
  
  // IO
  void writeTo(byte address, byte val);
  void readFrom(byte address, int num, byte buff_[]);
  
  // Calibration
  void printCalibrationValues(int samples);
  
  // DEBUG
  void printAllRegister();
  void print_byte(byte val);
  
  
 private:
  byte buff_[6];
  double xg;
  double yg;
  double zg;
  
  unsigned long int last_recorded_time;

  double _xoffset;
  double _yoffset;
  double _zoffset;
  
  Matrix data_ang_position = Matrix(3,1);
  Matrix data_ang_velocity = Matrix(3,1);
  Matrix data_ang_acceleration = Matrix(3,1);

};


#endif

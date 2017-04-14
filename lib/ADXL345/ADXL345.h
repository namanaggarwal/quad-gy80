#ifndef ADXL345_h
#define ADXL345_h

#include "Vector.h"
#include <Wire.h>
#include "Arduino.h"

#define ADXL345_DEVICE 0x53    //device address
#define ADXL345_TO_READ 6
#define ADXL345_POWER_CTRL 0x2d
#define ADXL345_DATAX0 0x32
#define ADXL345_DATA_FORMAT 0x31

#define ADXL345_OFFX 0x1E  // address for x offset
#define ADXL345_OFFY 0x1F  // address for y offset
#define ADXL345_OFFZ 0x20  // address for z offset

#define ALPHA 0.5

#define ADXL345_GRAVITY_EARTH 9.80665f


class ADXL345 : public Sensor
{
 public:
  ADXL345();
  void init(char xoff=0, char yoff=0, char zoff=0);
  void writeTo(byte address, byte val);
  Vector Update();
  Vector readNormalize(float gravityFactor = ADXL345_GRAVITY_EARTH);
  void printAllRegister();
  void print_byte(byte val);
  void printCalibrationValues(int samples);
  void readFrom(byte address, int num, byte _buff[]);
  void setSoftwareOffset(double x, double y, double z);

 private:
  byte _buff[6];
  double _xoffset, _yoffset, _zoffset;

};


#endif



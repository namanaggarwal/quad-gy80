#ifndef ADXL345_h
#define ADXL345_h

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

struct AccelRaw  //create 'vector' for raw data
{
  int x;
  int y;
  int z;
};

struct AccelG
{
  double x;
  double y;
  double z;
};

struct AccelRotation
{
  double pitch;
  double roll;
};

class ADXL345
{
 public:
  ADXL345();
  void init(char xoff=0, char yoff=0, char zoff=0);
  void writeTo(byte address, byte val);
  AccelRaw readAccel();
  AccelG readAccelG();
  void printAllRegister();
  void print_byte(byte_val);
  void printCalibrationValues(int samples);
  void readFrom(byte address, byte val);
  AccelRotation readPitchRoll();
  void softoff(double x, double y, double z);

 private:
  byte_buff[6];

  double xg, yg, zg;
  double _xoffset, _yoffset, _zoffset;

};


#endif



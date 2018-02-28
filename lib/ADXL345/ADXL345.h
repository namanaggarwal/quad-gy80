#ifndef ADXL345_h
#define ADXL345_h

#include <Wire.h>
#include "Arduino.h"
#include "Sensor.h"

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
    
    void Update();
    Matrix getAngularPosition();
    Matrix getAngularVelocity();
    
    void init(char xoff=0, char yoff=0, char zoff=0);
    
    // IO
    void writeTo(byte address, byte val);
    void readFrom(byte address, int num, byte _buff[]);
    
    // Calibration
    void printCalibrationValues(int samples);
    void setSoftwareOffset(double x, double y, double z);
    
    // DEBUG
    void printAllRegister();
    void print_byte(byte val);
    
  private:
    byte _buff[6];
    double _xoffset, _yoffset, _zoffset;
    
    Matrix data_ang_position(1,3);
    Matrix data_lin_acceleration(1, 3);
  

};


#endif



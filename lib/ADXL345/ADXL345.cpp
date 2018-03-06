#include "ADXL345.h"


ADXL345::ADXL345() {}

void ADXL345::Update()
{
  readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 16 bit resolution, ie 2 bytes. Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  
  //convert voltage readings to G's
  double fX, fY, fZ;
  fX = ((((int)_buff[1]) << 8) | _buff[0]) * 0.00390625;
  fY = ((((int)_buff[3]) << 8) | _buff[2]) * 0.00390625;
  fZ = ((((int)_buff[5]) << 8) | _buff[4]) * 0.00390625;
  
  data_lin_acceleration(0) = fX;
  data_lin_acceleration(1) = fY;
  data_lin_acceleration(2) = fZ;
  
  data_ang_position(0) = (atan2(fX, sqrt(fY*fY+fZ*fZ))*180) / PI; //pitch
  data_ang_position(1) = (atan2(fY, sqrt(fX*fX+fZ*fZ))*180) / PI; //roll
  data_ang_position(2) = (atan2(fZ, sqrt(fX*fX+fY*fY))*180) / PI; //yaw
}

Matrix ADXL345::getAngularPosition()
{
  return data_ang_position;
}

Matrix ADXL345::getLinearAcceleration()
{
  return data_lin_acceleration;
}

Matrix ADXL345::getAngularPosition2x1()
{
  Matrix r = Matrix(2,1);
  r(0) = data_ang_position(0);
  r(1) = data_ang_position(1);
  return r;
}

bool ADXL345::init(char x_offset, char y_offset, char z_offset)
{
  Wire.begin();
  writeTo(ADXL345_POWER_CTRL, 8);

  writeTo(ADXL345_DATA_FORMAT, 0x0B);

  writeTo(ADXL345_OFFX, x_offset);
  writeTo(ADXL345_OFFY, y_offset);
  writeTo(ADXL345_OFFZ, z_offset);

  return true;
}

// IO
void ADXL345::writeTo(byte address, byte val)
{
  Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

void ADXL345::readFrom(byte address, int num, byte _buff[])
{
  Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
  Wire.write(address); // sends address to read from
  Wire.endTransmission(); // end transmission

  Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
  Wire.requestFrom(ADXL345_DEVICE, num); // request 6 bytes from device Registers: DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1

  int i = 0;
  while(Wire.available()) // device may send less than requested (abnormal)
    {
      _buff[i] = Wire.read(); // receive a byte
      i++;
    }

  Wire.endTransmission(); // end trasmission
}
  
// Calibration  
void ADXL345::setSoftwareOffset(double x, double y, double z)
{
  _xoffset = x;
  _yoffset = y;
  _zoffset = z;
}
  
void ADXL345::printCalibrationValues(int samples)
{
  //double x,y,z;
  double xt,yt,zt;
  xt = 0;
  yt = 0;
  zt = 0;

  Serial.print("Calibration in: 3");
  delay(1000);
  Serial.print(" 2");
  delay(1000);
  Serial.println(" 1");
  delay(1000);

  for(int i=0; i<samples; i++)
    {
      //Vector accel = readNormalize();
      //xt += accel.x;
      //yt += accel.y;
      //zt += accel.z;
      //delay(100);
    }

  Serial.println("Accel Offset (mg): ");
  Serial.print("X: ");
  Serial.print(xt/float(samples)*1000,5);
  Serial.print(" Y: ");
  Serial.print(yt/float(samples)*1000,5);
  Serial.print(" Z: ");
  Serial.println( zt/float(samples)*1000,5);

  delay(2000);
}

// DEBUG  
void ADXL345::printAllRegister()
{
  byte _b;
  Serial.print("0x00: ");
  readFrom(0x00, 1, &_b);
  print_byte(_b);
  Serial.println("");
  int i;
  for (i=29;i<=57;i++)
    {
      Serial.print("0x");
      Serial.print(i, HEX);
      Serial.print(": ");
      readFrom(i, 1, &_b);
      print_byte(_b);
      Serial.println("");
    }
}

void ADXL345::print_byte(byte val)
{
  int i;
  Serial.print("B");
  for(i=7; i>=0; i--){
    Serial.print(val >> i & 1, BIN);
  }
}
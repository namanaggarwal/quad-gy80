#include "Arduino.h"
#include "L3G4200D.h"
 
L3G4200D::L3G4200D()
{
  xg = yg = zg = 0;
 
}
 
void L3G4200D::init(double xoffset, double yoffset, double zoffset)
{
  _xoffset = xoffset;
  _yoffset = yoffset;
  _zoffset = zoffset;
   
  writeTo(L3G4200D_CTRL_REG1, 0x0F);
  writeTo(L3G4200D_CTRL_REG4, 0x80); //Dont override values
  writeTo(L3G4200D_CTRL_REG5, 0x80); 
}
 
void L3G4200D::printCalibrationValues(int samples)
{
   
}
 
Matrix L3G4200D::Update() 
{
  // gyroscope Update function will always return an
  // array of size 6. Reading the gyroscope lets us
  // acquire the velocity of it's angular rotation in
  // quids. 
  readFrom(L3G4200D_OUT_X_L | 0x80, L3G4200D_BYTES_READ, buff_);
  Matrix raw = Matrix(1,3);
  
  raw(0) = ((((int)buff_[1]) << 8) | buff_[0]);
  raw(1) = (((int)buff_[3]) << 8) | buff_[2];
  raw(2) = (((int)buff_[5]) << 8) | buff_[4];

  
  return raw;
}
 
// Writes val to address register on device
void L3G4200D::writeTo(byte address, byte val) 
{
  Wire.beginTransmission(L3G4200D_DEVICE); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}
 
// Reads num bytes starting from address register on device in to buff_ array
void L3G4200D::readFrom(byte address, int num, byte buff_[])
{
  Wire.beginTransmission(L3G4200D_DEVICE); // start transmission to device
  Wire.write(address); // sends address to read from
  Wire.endTransmission(); // end transmission
 
  Wire.beginTransmission(L3G4200D_DEVICE); // start transmission to device
  Wire.requestFrom(L3G4200D_DEVICE, num); // request 6 bytes from device Registers: DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1
   
  int i = 0;
  while(Wire.available()) // device may send less than requested (abnormal)
  {
    buff_[i] = Wire.read(); // receive a byte
    i++;
  }
  if(i != num)
  {
 
  }
  Wire.endTransmission(); // end transmission
}
 
void L3G4200D::printAllRegister() 
{
    byte _b;
    Serial.print("0x00: ");
        readFrom(0x0F, 1, &_b);
    print_byte(_b);
    Serial.println("");
    int i;
    for (i=32;i<=56;i++)
        {
        Serial.print("0x");
        Serial.print(i, HEX);
        Serial.print(": ");
        readFrom(i, 1, &_b);
        print_byte(_b);
        Serial.println("");    
    }
}
 
void L3G4200D::print_byte(byte val)
{
    int i;
    Serial.print("B");
    for(i=7; i>=0; i--){
        Serial.print(val >> i & 1, BIN);
    }
}

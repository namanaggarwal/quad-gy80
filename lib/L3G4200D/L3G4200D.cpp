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
    Vector dps = readNormalize();
    xt += dps.x;
    yt += dps.y;
    zt += dps.z;
    delay(100);
  }
   
  Serial.println("Gyro Offset (dps): ");
  Serial.print("X: ");
  Serial.print(xt/float(samples),5);
  Serial.print(" Y: ");
  Serial.print(yt/float(samples),5);
  Serial.print(" Z: ");
  Serial.println( zt/float(samples),5);
   
  delay(2000);
}
 
Vector L3G4200D::readNormalize()
{
  //deafult 250dps = 8.75 mdps/digit
   
  Vector raw;
  raw = readRaw();
   
  double fXg, fYg, fZg;
  fXg = raw.x * 0.00875 + _xoffset;
  fYg = raw.y * 0.00875 + _yoffset;
  fZg = raw.z * 0.00875 + _zoffset;
 
  Vector dps;
   
  dps.x = fXg * ALPHA_G + (xg * (1.0-ALPHA_G));
  xg = dps.x;
   
  dps.y = fYg * ALPHA_G + (yg * (1.0-ALPHA_G));
  yg = dps.y;
   
  dps.z = fZg * ALPHA_G + (zg * (1.0-ALPHA_G));
  zg = dps.z;
   
  return dps;
   
   
}
 
void L3G4200D::Update(double* target, int size) 
{
  // gyroscope Update function will always return an
  // array of size 6. Reading the gyroscope lets us
  // acquire the velocity of it's angular rotation in
  // quids. 
  readFrom(L3G4200D_OUT_X_L | 0x80, L3G4200D_BYTES_READ, buff_); 
  
  Vector dps;
  dps.x = ((((int)buff_[1]) << 8) | buff_[0])
  dps.y = (((int)buff_[3]) << 8) | buff_[2];
  dps.z = (((int)buff_[5]) << 8) | buff_[4];

  
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

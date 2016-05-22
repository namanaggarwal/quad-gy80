#include "Arduino.h"
#include "HMC5883L.h"

HMC5883L::HMC5883L()
{
  xg = yg = zg = 0;
}

void HMC5883L::init(char x_offset, char y_offset, char z_offset)
{
  //writeTo(HMC5884L_MODE, 0) this is the default value anyway
  //HMC has no offset as far as I can tell
  //this may just be an empty function
  
  mgPerDigit = 0.92f;
}

void HMC5883L::calibrate()
{
  int minX, minY, maxX, maxY, offX, offY;
  minX = minY = maxX = maxY = 0;
  offX = offY = 0;
  for(int i = 0; i < 1000; i++) // take 1000 samples
    {
      MagnetoRaw raw = readCompass();

      if (raw.x < minX) minX = raw.x;
      if (raw.y < minY) minY = raw.y;
      if (raw.x > maxX) maxX = raw.x;
      if (raw.y > maxY) maxY = raw.y;
    }
  
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;

  Serial.print("Calculated X Offset: ");
  Serial.println(offX);
  Serial.print("Calculated Y Offset: ");
  Serial.println(offY);
}

void HMC5883L::setOffset(int xoffset, int yoffset)
{
  x_offset = xoffset;
  y_offset = yoffset;
}

void HMC5883L::writeTo(byte address, byte val)
{
  Wire.beginTransmission(HMC5883L_DEVICE);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

MagnetoRaw HMC5883L::readCompass()
{
  //read compass data from hmc5883l
  readFrom(HMC5883L_OUT_X_MSB, HMC5883L_TO_READ, _buff);

  MagnetoRaw raw;
  raw.x = (((int)_buff[1]) << 8) | _buff[0];                                      raw.y = (((int)_buff[3]) << 8) | _buff[2];
  raw.z = (((int)_buff[5]) << 8) | _buff[4];

  raw.x = raw.x - x_offset;
  raw.y = raw.y - y_offset;

  return raw;
}

void HMC5883L::readFrom(byte address, int num, byte _buff[])
{
  Wire.beginTransmission(HMC5883L_DEVICE); // start transmission to device
  Wire.write(address); // send address we want to read from
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_DEVICE); // start second transmission

  //we can get the data from sequential registers if we request more bytes
  //than 1
  Wire.requestFrom(HMC5883L_DEVICE, num); //request num bytes from registers

  int i = 0;
  while(Wire.available())
    {
      _buff[i] = Wire.read();
      i++;
    }

  Wire.endTransmission(); //end the transmission
}

MagnetoG HMC5883L::readCompassG()
{
  MagnetoRaw raw;
  raw = readCompass();
  
  double fXg, fYg, fZg;
  fXg = (raw.x) * mgPerDigit;
  fYg = (raw.y) * mgPerDigit;
  fZg = raw.z * mgPerDigit;
  //mg per digit = .9.92f

  MagnetoG res;

  res.x = fXg * .1 + (xg * .9);
  xg = res.x;
  res.y = fYg * .1 + (yg * .9);
  yg = res.y;
  res.z = fZg * .1 + (zg * .9);
  zg = res.z;

  return res;
}

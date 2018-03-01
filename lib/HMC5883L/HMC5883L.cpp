#include "HMC5883L.h"

HMC5883L::HMC5883L() {}

void HMC5883L::Update() {


  readFrom(HMC5883L_OUT_X_MSB, HMC5883L_TO_READ, _buff);

  Matrix raw = Matrix(1,3);

  raw(0) = ((int)_buff[1]) << 8 | _buff[0];
  raw(1) = ((int)_buff[3]) << 8 | _buff[2];
  raw(2) = ((int)_buff[5]) << 8 | _buff[4];

  return raw;
}

Matrix getHeading() {
  return data_heading;
}

bool HMC5883L::init() {
  readFrom(HMC5883L_ID_REG_A,3, _buff);
  
  if(_buff[0] != 0x48 ||
     _buff[1] != 0x34 ||
     _buff[2] != 0x33) {
    return false;
  }
  
  setRange(HMC5883L_RANGE_1_3GA);
  setMeasurementMode(HMC5883L_CONTINUOUS);
  setDataRate(HMC5883L_DATARATE_15HZ);
  setSamples(HMC5883L_SAMPLES_1);

  mgPerDigit = 0.92f;
  
  return true;
}


// IO
void HMC5883L::writeTo(byte address, byte val) {
  Wire.beginTransmission(HMC5883L_DEVICE);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

void HMC5883L::readFrom(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(HMC5883L_DEVICE); // start transmission to device
  Wire.write(address); // send address we want to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_DEVICE); // start second transmission

  Wire.requestFrom(HMC5883L_DEVICE, num); //request num bytes from registers

  int i = 0;
  while(Wire.available())
    {
      _buff[i] = Wire.read();
      i++;
    }
  Wire.endTransmission(); //end the transmission
}


// Calibration
void HMC5883L::calibrate() {

}

void HMC5883L::setOffset(int xoffset, int yoffset) {
  x_offset = xoffset;
  y_offset = yoffset;
}


// Getters and Setters
void HMC5883L::setRange(hmc5883l_range_t range) {
  
    switch(range)
      {
      case HMC5883L_RANGE_0_88GA:
	mgPerDigit = 0.073f;
	break;

      case HMC5883L_RANGE_1_3GA:
	mgPerDigit = 0.92f;
	break;

      case HMC5883L_RANGE_1_9GA:
	mgPerDigit = 1.22f;
	break;

      case HMC5883L_RANGE_2_5GA:
	mgPerDigit = 1.52f;
	break;

      case HMC5883L_RANGE_4GA:
	mgPerDigit = 2.27f;
	break;

      case HMC5883L_RANGE_4_7GA:
	mgPerDigit = 2.56f;
	break;

      case HMC5883L_RANGE_5_6GA:
	mgPerDigit = 3.03f;
	break;

      case HMC5883L_RANGE_8_1GA:
	mgPerDigit = 4.35f;
	break;

      default:
	break;
      }


    writeTo(HMC5883L_CONF_REG_B, range << 5);
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode) {
  byte value;

  readFrom(HMC5883L_MODE, 1, &value);
  //  value = _buff[1]
  value &= 0b11111100;
  value |= mode;
  
  writeTo(HMC5883L_MODE, value);
}

void  HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate) {
  byte value;

  readFrom(HMC5883L_CONF_REG_A, 1, &value);
  //  value = _buff[1];
  value &= 0b11100011;
  value |= (dataRate << 2);

  writeTo(HMC5883L_CONF_REG_A, value);
}

void HMC5883L::setSamples(hmc5883l_samples_t samples) {
  byte value;

  readFrom(HMC5883L_CONF_REG_A, 1, &value);
  value &= 0b10011111;
  value |= (samples << 5);

  writeTo(HMC5883L_CONF_REG_A, value);
  
}

hmc5883l_range_t HMC5883L::getRange() {
  byte value;
  
  readFrom(HMC5883L_CONF_REG_B, 1, &value);
  //  value  = _buff[1];
  return (hmc5883l_range_t)(value >> 5);
}

hmc5883l_mode_t HMC5883L::getMeasurementMode() {
  byte value;
  readFrom(HMC5883L_MODE, 1, &value);
  //  value = _buff[1];
  value &= 0b00000011;
  
  return (hmc5883l_mode_t)value;
}

hmc5883l_dataRate_t HMC5883L::getDataRate() {
  byte value;

  readFrom(HMC5883L_CONF_REG_A, 1, &value);
  //  value = _buff[1];
  value &= 0b00011100;
  value >>= 2;

  return (hmc5883l_dataRate_t)value;
  
}

hmc5883l_samples_t HMC5883L::getSamples() {
  byte value;

  readFrom(HMC5883L_CONF_REG_A, 1, &value);
  value &= 0b01100000;
  value >>= 5;

  return (hmc5883l_samples_t)value;
}
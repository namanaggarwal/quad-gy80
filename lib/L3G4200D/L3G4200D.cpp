#include "Arduino.h"
#include "L3G4200D.h"
 
L3G4200D::L3G4200D()
{
  xg = yg = zg = 0;
}

void L3G4200D::Update() 
{
  // gyroscope Update function will always return an
  // array of size 6. Reading the gyroscope lets us
  // acquire the velocity of it's angular rotation in
  // quids. 
  readFrom(L3G4200D_OUT_X_L | 0x80, L3G4200D_BYTES_READ, buff_);
  unsigned long int current_time = micros()/1000000;
  float delta_time = (current_time - last_recorded_time);
  
  float prev_data_x = data_ang_velocity(0);
  float prev_data_y = data_ang_velocity(1);
  float prev_data_z = data_ang_velocity(2);
  
  // angular velocity
  data_ang_velocity(0) = ((((int)buff_[1]) << 8) | buff_[0]) * 0.00875 + _xoffset;
  data_ang_velocity(1) = ((((int)buff_[3]) << 8) | buff_[2]) * 0.00875 + _yoffset;
  data_ang_velocity(2) = ((((int)buff_[5]) << 8) | buff_[4]) * 0.00875 + _zoffset;

  // angular position (integratal)
  data_ang_position(0) += data_ang_velocity(0)*delta_time;
  data_ang_position(1) += data_ang_velocity(1)*delta_time;
  data_ang_position(2) += data_ang_velocity(2)*delta_time;
  
  // angular acceleration (derivative)
  data_ang_acceleration(0) = (data_ang_velocity(0) - prev_data_x)/delta_time;
  data_ang_acceleration(1) = (data_ang_velocity(1) - prev_data_y)/delta_time;
  data_ang_acceleration(2) = (data_ang_velocity(2) - prev_data_z)/delta_time;
  
  // update time
  last_recorded_time = current_time;
}

Matrix L3G4200D::getAngularAcceleration() {
  return data_ang_acceleration;
}

Matrix L3G4200D::getAngularVelocity() {
  return data_ang_velocity;
}

Matrix L3G4200D::getAngularPosition() {
  return data_ang_position;
}

Matrix L3G4200D::getKalmanInput() {
  Matrix r = Matrix(6,1);
  r(0) = data_ang_velocity(0);
  r(1) = data_ang_velocity(1);
  r(2) = data_ang_velocity(2);
  r(3) = data_ang_position(0);
  r(4) = data_ang_position(1);
  r(5) = data_ang_position(2);
  return r;
}

bool L3G4200D::init(double xoffset, double yoffset, double zoffset)
{
  _xoffset = xoffset;
  _yoffset = yoffset;
  _zoffset = zoffset;
   
  writeTo(L3G4200D_CTRL_REG1, 0x0F);
  writeTo(L3G4200D_CTRL_REG4, 0x80); //Dont override values
  writeTo(L3G4200D_CTRL_REG5, 0x80); 
  last_recorded_time = micros()/1000000;
  return true;
}

// Callibration
void L3G4200D::printCalibrationValues(int samples)
{
   
}
 
// IO
void L3G4200D::writeTo(byte address, byte val) 
{
  Wire.beginTransmission(L3G4200D_DEVICE); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}
 
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
 
// DEBUG
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

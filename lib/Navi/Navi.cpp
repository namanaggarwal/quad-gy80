#include "Navi.h"

Navi::Navi(ADXL345* accelerometer, L3G4200D* gyroscope, HMC5883L* magnetometer)
{
  //Tell Navi what sensors to use
  accel = accelerometer;
  gyro = gyroscope;
  compass = magnetometer;
  //altimeter and kalman filter go here
}

bool Navi::init() {
  Serial.println("Navi initializing...");
  accel->init(0,0,0);
  gyro->init(0,0,0);
  compass->init(0,0,0);

  return true;
}

void Navi::UpdateState() {
  // logic for updating state here
  //
  // get next state based on previous state
  Serial.println("Updating State");
  a = accel.Update();
  g = gyro.Update();
  c = compass.Update();
}

void Navi::printSensorInfo() {
    
  Serial.print("Accel Vals: ");
  Serial.print(a(0));
  Serial.print(" | ");
  Serial.print(a(1));
  Serial.print(" | ");
  Serial.print(a(2));
  Serial.print("\t");
  
  Serial.print("Gyro Vals: ");
  Serial.print(g(0));
  Serial.print(" | ");
  Serial.print(g(1));
  Serial.print(" | ");
  Serial.print(g(2));
  Serial.print("\t");
  
  Serial.print("Comp Vals: ");
  Serial.print(c(0));
  Serial.print(" | ");
  Serial.print(c(1));
  Serial.print(" | ");
  Serial.print(c(2));
  Serial.println();
}
  


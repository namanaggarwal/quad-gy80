#include "Navi.h"


Navi::Navi(ADXL345* accelerometer, L3G4200D* gyroscope, HMC5883L* magnetometer)
{
  //Tell Navi what sensors to use
  accel = accelerometer;
  gyro = gyroscope;
  compass = magnetometer;
}



bool Navi::init() {
  Serial.println("Navi initializing...");
  accel->init(0,0,0);

  return true;
}



void Navi::UpdateState() {
  // logic for updating state here
  //
  // get next state based on previous state
  Serial.println("Updating State");
}

void Navi::printSensorInfo() {
  /*  //test function to show proper sensor behavior
  Serial.print("Accelerometer Reading: ");
  Serial.println("-------------------------");
  a = accel.readRaw();
  Serial.print("X:");
  Serial.print(a.x);
  Serial.print("\tY:");
  Serial.println(a.y);
  Serial.print("\tZ:");
  Serial.print(a.z);

  Serial.print("Gyroscope Reading: ");
  Serial.println("-------------------------");
  a = gyro.readRaw();
  Serial.print("X:");
  Serial.print(a.x);
  Serial.print("\tY:");
  Serial.println(a.y);
  Serial.print("\tZ:");
  Serial.print(a.z);

  Serial.print("Compass Reading: ");
  Serial.println("-------------------------");
  a = compass.readRaw();
  Serial.print("X:");
  Serial.print(a.x);
  Serial.print("\tY:");
  Serial.println(a.y);
  Serial.print("\tZ:");
  Serial.print(a.z); */
}
  


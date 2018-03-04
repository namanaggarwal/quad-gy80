#include "Navi.h"

Navi::Navi()//, HMC5883L* magnetometer)
{

  
}

bool Navi::init() {
  // Bind sensors to kalman
  //Kalman myKalman2(accel, GPS);
  Serial.println("Navi initializing...");
  Wire.begin();
  
  Serial.println("Accelerometer Initializing...");
  accel.init(0,0,0);
  Serial.println("Gyroscope Initializing...");
  gyro.init(0,0,0);
  //compass.init(0,0,0);
  //alti.init(0);
  Serial.print("Preparing Kalman Filter... ");
  if(myKalman.init(&accel, &gyro) == 0) {
    Serial.print("!!! --- Kalman Filter did not initialize properly --- !!!");
    while(1) {}
  }
  else Serial.println("Complete!");
  return true;
}

bool Navi::UpdateState() {
  myKalman.PredictState(); 
  myKalman.getState().printMatrix();
  return true;
}

Matrix Navi::getState() {
  return angle;
}

Kalman Navi::getKalman() {
  return myKalman;
}

void Navi::printSensorInfo() {
    
  // Serial.print("Accel Vals: ");
  // Serial.print(a(0));
  // Serial.print(" | ");
  // Serial.print(a(1));
  // Serial.print(" | ");
  // Serial.print(a(2));
  // Serial.print("\t");
  
  // Serial.print("Gyro Vals: ");
  // Serial.print(g(0));
  // Serial.print(" | ");
  // Serial.print(g(1));
  // Serial.print(" | ");
  // Serial.print(g(2));
  // Serial.print("\t");
  
  // Serial.print("Comp Vals: ");
  // Serial.print(c(0));
  // Serial.print(" | ");
  // Serial.print(c(1));
  // Serial.print(" | ");
  // Serial.print(c(2));
  // Serial.println();
}
  


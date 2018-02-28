#include "Navi.h"

Navi::Navi(ADXL345* accelerometer, L3G4200D* gyroscope)//, HMC5883L* magnetometer)
{
  // Bind sensors to Navi
  accel = accelerometer;
  gyro = gyroscope;
  //compass = magnetometer;
  
  // Bind sensors to kalman
  Kalman myKalman(accel, gyro);
  //Kalman myKalman2(accel, GPS);
  
}

bool Navi::init() {
  Serial.println("Navi initializing...");
  accel->init(0,0,0);
  gyro->init(0,0,0);
  //compass->init();

  return true;
}

void Navi::UpdateState() {
  // Compute Time Delta
  
  // Get our actual state
  // Call Kalman PredictNextState
  myKalman.PredictNextState(dt);
  //myKalman2.PredictNextState(dt);
  // Get our new state
  angle = myKalman.getState();
  //linear = myKalman2.getState();
  
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
  


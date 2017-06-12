#include <Wire.h>
#include <TimerOne.h>
 
#include "matrix.h"

#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"


ADXL345 accel;
Matrix a = Matrix(1,3);

L3G4200D gyro;
Matrix g = Matrix(1,3);

HMC5883L compass;
Matrix c = Matrix(1,3);

//commit test
void setup()
{

  // Initialize Serial Communication on Arduino
  Serial.begin(9600);
  Serial.println("Starting Up.");
  Wire.begin();

  accel.init();
  gyro.init();
  compass.init();
   
}
 
void loop()
{
  a = accel.Update();
  g = gyro.Update();
  c = compass.Update();
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
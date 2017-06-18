#include "Navi.h"

 
ADXL345 accel;
L3G4200D gyro;
HMC5883L compass;

Navi navi(&accel, &gyro, &compass);

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
  navi.UpdateState();
  navi.printSensorInfo();
  
  
  
}
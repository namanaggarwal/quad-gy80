#include <Wire.h>
#include <TimerOne.h>
 
#include "Navi.h"


ADXL345 adxl345;
L3G4200D l3g4200d;
HMC5883L hmc5883l;
Navi navi(&adxl345, &l3g4200d, &hmc5883l);

void setup()
{

  // Initialize Serial Communication on Arduino
  Serial.begin(9600);
  Serial.println("Starting Up.");
  Wire.begin();

  // Initialize Sensors
  if(navi.init() == 0) {
    Serial.print("Navi could not initialize!!");
  }
  else Serial.print("Navi Initialized");
   
}
 
void loop()
{
  //navi.printSensorInfo();
  navi.UpdateState();
  delay(500);
  
}
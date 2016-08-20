
/* Unit test for compass */

#include "HMC5883L.h"
#include <Wire.h>


HMC5883L compass;

void initHMC() {
  Serial.println("Compass Ready.");
  if(!compass.init()) {
    Serial.println("Compass not ready!");
    while(1);
  }
  compass.calibrate();
  while(1){}
  //compass.setOffset(20, -6);
  //compass.setRange(HMC5883L_RANGE_1_3GA);
  //compass.setMeasurementMode(HMC5883L_CONTINUOUS);
  //compass.setDataRate(HMC5883L_DATARATE_30HZ);
  //compass.setSamples(HMC5883L_SAMPLES_8);
}

void setup(){

  Serial.begin(9600);

  Serial.print("\nReady");
  
  Wire.begin();
  initHMC();
  
}



void loop(){
  
  //Compass
  Vector norm;
  norm  = compass.readNormalize();

  //No Tilt compensated heading:
  float heading1 = atan2(norm.x, norm.y);
  //convert to degrees
  float deg = (heading1 *  180)/M_PI;  
 
  
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  // Lexington, KY -South

  float declinationAngle = (-5.0 + (36.0/60.0)) / (180 / M_PI );
  
 
  deg += declinationAngle;

  
  Serial.println(deg);
  

}
  





  







#include "ADXL345.h"

ADXL345 accel;



void setup(){

  Serial.begin(9600);

  Serial.print("\nReady");
  Wire.begin();

  accel.printCalibrationValues(40);



}


void loop(){





}
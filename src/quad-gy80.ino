#include <Wire.h>
#include <TimerOne.h>
 
#include "ADXL345.h"
#include "L3G4200D.h"
//Gyro sampling rate in micro seconds
#define SampleRate 10000

ADXL345 accel;
L3G4200D gyro;

bool readGyro;
unsigned long pT; //previous time

Vector a;
Vector g;
Vector c;

void setup()
{
   
  Serial.begin(9600);
   
  Serial.println("Ready.");
  Wire.begin();
   
  
  accel.init(0,0,0);
  gyro.init(0,0,0);
  //accel.printCalibrationValues(40);
  //gyro.printCalibrationValues(40);
 
    
  pT = 0;
  readGyro = false;
      
  Timer1.initialize(SampleRate);
  Timer1.attachInterrupt( timerIsr );

  Serial.print("Printing Gryo values");
}
 
void timerIsr()
{
  readGyro = true;
}
 
void loop()
{
   
  //TEST LOOP
  a = accel.readNormalize();
  g = gyro.readNormalize();
  /*
  Serial.print("X: ");
  Serial.print(a.x);
  Serial.print("\tY: ");
  Serial.print(a.y);
  Serial.print("\tZ: ");
  Serial.println(a.z);
  */
  Serial.print("X: ");
  Serial.print(g.x);
  Serial.print("\tY: ");
  Serial.print(g.y);
  Serial.print("\tZ: ");
  Serial.println(g.z);
 

}
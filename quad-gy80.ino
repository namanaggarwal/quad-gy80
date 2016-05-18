#include "ADXL345.h"
#include "L3G4200D.h"
#include <Wire.h>
#include <Servo.h>


#define ESC_A 10
#define ESC_B 9
#define ESC_C 6
#define ESC_D 5

#define ESC_MIN 600  //needs testing
#define ESC_MAX 1200
#define ESC_ARM_DELAY 7000

Servo a,b,c,d;




ADXL345 accel;
L3G4200D gyro;
inline void arm() {
  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);

  delay(1000);


}

inline void initESCs() {

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);

  delay(1000);

  arm();

}

inline void initADXL() {

  Serial.println("Accelerometer Ready.");
  accel.init(0,0,0);
  accel.setSoftwareOffset(0,0,0);
  accel.printCalibrationValues(40);
}


inline void initL3G() {
  Serial.println("Gyro Ready.");
   //  gyro.init()
   gyro.printCalibrationValues(40); // X: -30.34639 Y: 4.40818 Z: 8.29726



}



void setup(){

  Serial.begin(9600);

  Serial.print("\nReady");
  Wire.begin();
  initADXL();
  initL3G();
  initESCs();
  initESCs();



}


void loop(){





}

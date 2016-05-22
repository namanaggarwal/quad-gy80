#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"

#include <Wire.h>
#include <Servo.h>
#include <TimerOne.h>

#define ESC_A 10
#define ESC_B 9
#define ESC_C 6
#define ESC_D 5

#define ESC_MIN 600  //needs testing
#define ESC_MAX 1200
#define ESC_ARM_DELAY 7000

#define SampleRate 10000
Servo a,b,c,d;


ADXL345 accel;
L3G4200D gyro;
HMC5883L compass;

bool readGyro;
double angleF_roll;
double angleF_pitch;
double angleF_yaw;
float headingDegrees;
unsigned long pT;
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

inline void initHMC() {
  Serial.println("Compass Ready.");

}


//void DEBUG() {
//  Serial.print("Filtered Roll: ");
//  Serial.println(angleF_roll);

//  Serial.print("Filtered Pitch: ");
//  Serial.print(angleF_pitch);

//  Serial.print("Filtered Yaw: ");
//  Serial.print(angleF_yaw);
//}


void setup(){

  Serial.begin(9600);

  Serial.print("\nReady");

  pT = 0;
  readGyro = false;
  angleF_roll = angleF_pitch = angleF_yaw = headingDegrees = 0;
  
  Wire.begin();
  initADXL();
  initL3G();
  initHMC();
  
  //initESCs();
  //initESCs();
  Timer1.initialize(SampleRate);
  Timer1.attachInterrupt( timerIsr );


}

void timerIsr()
{
  readGyro = true;
}


void loop(){
  //Accel
  AccelRotation accelRot;
  accelRot = accel.readPitchRoll(); 

  
  //Gyro
  if(readGyro)
    {
      readGyro = false;
      unsigned long cT = micros();
      
      GyroDPS gDPS;
      gDPS = gyro.readGyroDPS();
      unsigned long dT = cT - pT;
      pT = cT;

      // Filtered pitch and roll from accel and gyro
      angleF_roll = 0.95*(angleF_roll + gDPS.x*(dT/1000000.0)) +0.05*accelRot.roll;
      angleF_pitch = 0.95*(angleF_pitch - gDPS.y*(dT/1000000.0)) +0.05*accelRot.pitch;
      angleF_yaw = 0.95*(angleF_yaw - gDPS.y*(dT/1000000.0)) +0.05*headingDegrees;

      Serial.print("Roll: ");
      Serial.print(angleF_roll);
      Serial.print("\tPitch: ");
      Serial.print(angleF_pitch);
      Serial.print("\tYaw: ");
      Serial.print(angleF_yaw);
      Serial.print("\tHeading: ");
      Serial.println(headingDegrees);
    }

  //Compass
  MagnetoG cDPS;
  cDPS = compass.readCompassG();

  float heading = atan2(cDPS.y, cDPS.x);
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  //Lexington, KY -South

  float declinationAngle = (-5.0  + (36.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  if (heading < 0)
    {
      heading += 2 * PI;
    }

  if (heading > 2 * PI)
    {
      heading -= 2 * PI;
    }

  // Convert to doegrees
  headingDegrees = heading * 180/M_PI;
}
  





  






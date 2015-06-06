// nav_control.ino


#include <Wire.h>
#include "PID_v1.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "TimerOne.h"
#include "HMC5883L.h"



// accel
ADXL345 accel;

// variables for roll and pitch calculated
// from accel readings

float pitch, roll;

//offset variables that are written to offset registers
//*note that these are in incrememnets of 15.6 mG
uint8_t xofs = 6; 
uint8_t yofs = -2;
uint8_t zofs = 5;

unsigned long time2 = 0;
unsigned long pT;

// compass
HMC5883L compass;
float heading1;
float heading2;



// gyro
L3G4200D gyro;

#define SampleRate 10000
bool readGyro;
double xi,yi,zi;



//fused results:
float angleF_roll, angleF_pitch, angleF_yaw;


void setup() {

	//open a serial connection at 9600 baud
	Serial.begin(9600);
	//start wire library instance
  Wire.begin();

	// setup accelerometer
	if (!accel.begin())
  {
   	delay(500);
  }
  	
  // set range of accel, this needs to tested  
  accel.setRange(ADXL345_RANGE_16G);

   //set offsets:
  accel.setAccelOFS(xofs, yofs, zofs);

  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);


  compass.setOffset(109, -185); 


	// setup gyroscope
  gyro.init(0,0,0);
  //set previous time to 0
  pT = 0;
  xi = yi = zi = 0;
  readGyro = false;
  //set interrupt to trigger every SampleRate 
  Timer1.initialize(SampleRate);
  Timer1.attachInterrupt( timerIsr ); 	


}

void timerIsr()
{
  readGyro = true;
}

float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}

float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll 

  float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
}

void loop() {

	// Read normalized values
	Vector norm = accel.readNormalize();

	// Low pass filter to smooth out data
	Vector filtered = accel.lowPassFilter(norm, .5);

  // Read compass 
  Vector comp_norm = compass.readNormalize();


  Vector mag = compass.readNormalize();
  Vector acc = accel.readScaled();

  // Calculate headings
  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, acc);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // For Lexington / Kentucky declination angle is -5'34W 
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-5.0 + (34.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI; 


	//Calculate pitch & roll -- filtered
	pitch = -(-(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI);
	roll = -((atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI);


  if(readGyro)
  {
    readGyro = false;
    unsigned long cT = micros();
     
    GyroDPS gDPS;
    gDPS = gyro.readGyroDPS();
    
    //calculate change in time for integration
    unsigned long dT = cT - pT;
    pT = cT;
    
    //integrate angular velocity to get angular position
    // new pos = old pos + velocity * time
    // time = current time - previous time
    // we put this back into regular seconds to get a pos that makes sense

    xi = xi + gDPS.x*(dT/1000000.0);
    yi = yi + gDPS.y*(dT/1000000.0);
    zi = zi + gDPS.z*(dT/1000000.0);

    angleF_roll = 0.95*(angleF_roll + gDPS.x*(dT/1000000.0)) +0.05*roll;
    angleF_pitch = 0.95*(angleF_pitch - gDPS.y*(dT/1000000.0)) +0.05*pitch;
    angleF_yaw = heading2; //gDPS.z needs to be incorporated into the measurement


  }

	if (millis() - time2 > 2500){
		time2 = millis();
		Serial.print("\nA-Roll: ");
		Serial.print(roll);
		Serial.print("\tG-Roll: ");
		Serial.print(xi);
    Serial.print("\tF-Roll: ");
    Serial.print(angleF_roll);

		Serial.print("\nA-Pitch: ");
		Serial.print(pitch);
		Serial.print("\tG-Roll: ");
		Serial.print(yi);
    Serial.print("\tF-Pitch: ");
    Serial.print(angleF_pitch);

    Serial.print("\nG-Yaw: ");
    Serial.print(zi);
    Serial.print("\tC-Yaw: ");
    Serial.print(heading2);
    Serial.print("\tF-Yaw: ");
    Serial.print(angleF_yaw);
	}
}



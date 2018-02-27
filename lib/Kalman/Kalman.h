// Apply this Kalman filter on one axis at a time!


#ifndef KALMAN_H
#define KALMAN_H

//#include "Arduino.h"
#include "Sensor.h"

class Kalman
{
  public:
    Kalman(Sensor* sensor1, Sensor* sensor2);
    float PredictState(float dt);
    
  private:
  Matrix PCM = 
    
    
    
  

};

#endif

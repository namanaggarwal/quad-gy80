// Apply this Kalman filter on one axis at a time!


#ifndef KALMAN_H
#define KALMAN_H

//#include "Arduino.h"
#include "Sensor.h"

class Kalman
{
  public:
    Kalman(Sensor* sensor1, Sensor* sensor2);
    void PredictState(float dt);
    Matrix getState();
    
  private:
    void Kalman::calcStateMatrix();
    unsigned long int dt;
    unsigned long int pt = 0;
    Matrix data = Matrix(1,3); // return data
    Matrix Xp = Matrix(1,6);   // Current State
    Matrix PCM = Matrix(6,6);  // Process Covariance Matrix
    Matrix Pkp = Matrix(6,6);
    Matrix KG = Matrix(6,6);
    Matrix Yk = Matrix(6,1);
    
};

#endif

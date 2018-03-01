// Apply this Kalman filter on one axis at a time!


#ifndef KALMAN_H
#define KALMAN_H

//#include "Arduino.h"
#include "Sensor.h"

class Kalman
{
  public:
    Kalman(L3G4200D* gyroscope, ADXL345* accelerometer);
    void PredictState();
    Matrix getState();
    
  private:
    void Kalman::calcStateMatrix();
    void Kalman::calcProcessControlMatrix();
    void Kalman::calcKalmanGain();
    void Kalman::calcNewMeasurement();
    void Kalman::calcCurrentState();
    void Kalman::calcNewProcessControlMatrix();

    void getdt();
    void create_Xk_matrix();
    
    unsigned long int dt = 0;
    unsigned long int pt = 0;
    
    L3G4200D* gyro;
    ADXL345* accel;

    Matrix Xk = Matrix(1,6);  // Current State
    Matrix Pk = Matrix(6,6);  // Process Covariance Matrix
    Matrix K = Matrix(6,6);   // Kalman Gain
    Matrix Yk = Matrix(6,1);  // Observed State
    
    // User provided
    Matrix A = Matrix(6, 6);  // State Transition Model
    //Matrix B = Matrix(3,6);   // Control-Input Model
    //Matrix C = Matrix(6,6);   
    Matrix H = Matrix(6,6);
    

    // Noise covariance Matrices - User Tuned
    //Matrix Q = Matrix(6,6);   // Covariance of process noise
    //Matrix R = Matrix (6,6);   // Covariance of the observation noise

    
};

#endif

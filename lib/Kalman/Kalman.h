// Apply this Kalman filter on one axis at a time!


#ifndef KALMAN_H
#define KALMAN_H

#include "Arduino.h"
#include "ADXL345.h"
#include "L3G4200D.h"


class Kalman
{
  public:
    Kalman();
    bool init(ADXL345* accelerometer, L3G4200D* gyroscope);
    bool PredictState();
    Matrix getState();
    void init();
    
  private:
    void calcStateMatrix();
    void calcProcessControlMatrix();
    void calcKalmanGain();
    void calcNewMeasurement();
    void calcCurrentState();
    void calcNewProcessControlMatrix();

    void getdt();

    float dt = 0.0;
    float pt = 0.0;
    
    L3G4200D* gyro;
    ADXL345* accel;

    Matrix Xk = Matrix(6,1);  // Current State
    Matrix Pk = Matrix(6,6);  // Process Covariance Matrix
    Matrix K = Matrix(6,6);   // Kalman Gain
    Matrix Yk = Matrix(6,1);  // Observed State
    
    // User provided
    Matrix A = Matrix(6, 6);  // State Transition Model
    //Matrix B = Matrix(3,6);   // Control-Input Model
    //Matrix C = Matrix(6,6);   // duplicate of H
    Matrix H = Matrix(6,6);
    

    // Noise covariance Matrices - User Tuned
    //Matrix Q = Matrix(6,6);   // Covariance of process noise
    //Matrix R = Matrix (6,6);   // Covariance of the observation noise

    
};

#endif

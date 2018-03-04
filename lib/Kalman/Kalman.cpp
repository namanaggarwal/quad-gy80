/*************************************************************************
* 
*  This Kalman filter predicts the next state whenever PredictNextState is
*  is called. It keeps track of its own process covariance matrix and its
*  process and observational error. The class binds to two sensor objects
*  and reports the predicted state for 3 axes. Two instances of this class
*  are needed to cover the six axis required for movement in a 3-D space.
*   
*************************************************************************/

#include "Kalman.h"
#include "ADXL345.h"
#include "L3G4200D.h"
Kalman::Kalman() {
  //Arduino why
}

bool Kalman::init(ADXL345* accelerometer, L3G4200D* gyroscope) {
    gyro->Update(); 
    Xk = gyro->getKalmanInput(); // we should have a 1x6 matrix here
    //Pk is initialized correctly already: https://en.wikipedia.org/wiki/Kalman_filter#Example_application,_technical

    // Initialize our observation model/Identity matrix
    double H_arr[36] = {1, 0, 0, 0, 0, 0,
                       0, 1, 0, 0, 0, 0,
                       0, 0, 1, 0, 0, 0,
                       0, 0, 0, 1, 0, 0,
                       0, 0, 0, 0, 1, 0,
                       0, 0, 0, 0, 0, 1};
    H = Matrix(6,6, H_arr);    
    return true;
}

bool Kalman::PredictState() {
    Serial.print("Step 1: Calculating State Matrix");
    calcStateMatrix();
    return true;

    // calcProcessControlMatrix();
    // calcKalmanGain();  
    // calcNewMeasurement();
    // calcCurrentState();
    // calcNewProcessControlMatrix();

    
}

void Kalman::calcStateMatrix() {
    // Step 1. Calculate the State Matrix
    //    X_kp = AX_k-1 + Bu_k + w_k
    // Initialize A and B - needs to be done every loop in order to update
    // the time-dependent values.
    // Note: in this step, we mathematically determine where our device has moved
    // based on the data we had previously
    getdt();
    double A_arr[36] = {1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0,
                        dt, 0, 0, 1, 0, 0,
                        0, dt, 0, 0, 1, 0,
                        0, 0, dt, 0, 0, 1};
    A = Matrix(6,6, A_arr);

    double B_arr[18] = {.5*dt*dt, 0, 0,
                        0, .5*dt*dt, 0,
                        0,  0, .5*dt*dt,
                        dt, 0, 0,
                        0,  dt, 0, 
                        0,  0, dt};

    Matrix B = Matrix(6, 3, B_arr);
    
    // poll our gyro for the new angular acceleration values
    Matrix u = gyro->getAngularAcceleration();
    
    // plug & chug
    Xk = A.multiply(Xk).add(B.multiply(u));
}

void Kalman::calcProcessControlMatrix() {
  // Step 2. Calculate the Predicted Process Control Matrix
  //    P_kp = AP_k-1 * A^T + Q_k
  // Note: the first run-through of this function yields the same value since Pk = 0;
  // Note: Q is the process noise variance for the gyro bias/acceleromoter
  Pk = A.multiply(Pk).multiply(A.transpose());
}

void Kalman::calcKalmanGain() {
  // Step 3. Calculate the Kalman Gain (K)
  //    K = (Pk*H^T)/(H*Pk*H^T + Rk)
  // Note: R is the measurement noise variance - variance of the measurement noise!
  K = Pk.multiply(H.transpose()).divide(
    H.multiply(Pk).multiply(H.transpose()));
}

void Kalman::calcNewMeasurement() {
  // Step 4. Update the Observed State
  //    Yk = H * Yk + Zk
  // Note: here we update our current measurements from our sensors.
  gyro->Update();
  accel->Update();
  Matrix Ykm = gyro->getAngularVelocity().concatenate(accel->getAngularPosition(), 0);
  Yk = H.multiply(Ykm);
}

void Kalman::calcCurrentState() {
  // Step 5. Calculate the Current State:
  //    Xk = Xk + K[Yk - HXk]
  // Note: larger/smaller K values place more/less emphasis on our newest estimation.
  Xk = Xk.add(K.multiply(Yk.subtract(H.multiply(Xk))));
}

void Kalman::calcNewProcessControlMatrix() {
  // Step 6. Calculate the New Process Control Matrix
  //    Pk = (I - KH)*Pk
  Pk = H.subtract(K.multiply(H)).multiply(Pk);
}

Matrix Kalman::getState() {
    return Xk;
}

void Kalman::getdt() {
  // process our change in time
  unsigned long int ct = micros();
  dt = (ct - pt)/1000000;
  pt = ct/1000000;
  
}


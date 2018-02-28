/*************************************************************************
* 
*  This Kalman filter predicts the next state whenever PredictNextState is
*  is called. It keeps track of its own process covariance matrix and its
*  process and observational error. The class binds to two sensor objects
*  and reports the predicted state for 3 axis. Two instances of this class
*  are needed to cover the six axis required for movement in a 3-D space.
*   
*************************************************************************/

#include "Kalman.h"



Kalman::Kalman(L3G4200D* gyro, ADXL345* accel) {
    // setup initial conditions
    gyro->Update();
    accel->Update();
    Xp = gyro->getAngularVelocity().concatenate(accel->getAngularPosition(), 0) // we should have a 1x6 matrix here
}

Kalman::getState() {
    return data;
}

Matrix Kalman::PredictState() {
    // get new dt
    getdt()
    
    // initial input is used to calculate the current state
    // the current predicted state with:
    //      X_kp = AX_k-1 + Bu_k + w_k
    calcStateMatrix();
    
    // Calculate process control matrix:
    //      P_k = AP_k-1 * A^T + Q_k
    calcProcessControlMatrix();
    
    // These are used to calculate the Kalman Gain (KG)
    //      K = (P_kp * H) / (H * P_kp * H^T +R)
    //      X_k = X_kp  + K( Y - H * X_kp ) <- updated state
    //  * H is used to transform the values so the
    //  multiplaction/divison can work properly
    calcKalmanGain();
    
    calcNewMeasurement();
    // Current becomes the previous state and 
    // the currently predicted state
    //      P_k = (I - KH) * P_kh
    //      X_p
    calcCurrentState();
    
    //state time is given by parameter
    delta_time = dt;
    
    //return current state
}

Kalman::getdt() {
  unsigned long int ct = micros();
  dt = ct - pt;
  pt = ct;
  
}

void Kalman::calcStateMatrix() {
    //X_kp = AX_k-1 + Bu_k + w_k
    Matrix A = Matrix(6,6, [1, 0, 0, (dt/(10**6)), 0, 0,
                           0, 1, 0, 0, (dt/(10**6)), 0,
                           0, 0, 1, 0, 0, (dt/(10**6)),
                           0, 0, 0, 1, 0, 0,
                           0, 0, 0, 0, 1, 0,
                           0, 0, 0, 0, 0, 1]);
      
    Matrix B = Matrix(6, 3, [.5*(dt/(10**6))*(dt/(10**6)), 0, 0,
                              0, .5*(dt/(10**6))*(dt/(10**6)), 0,
                              0,  0, .5*(dt/(10**6))*(dt/(10**6)),
                              (dt/(10**6)), 0, 0,
                              0,  (dt/(10**6)), 0, 
                              0,  0, (dt/(10**6))]);
    
    Matrix u = gyro->getAngularAcceleration();
    
    Xkp = A*Xp + B*u;
    Pkp = calcProcessControlMatrix(A);
}

Matrix Kalman::calcProcessControlMatrix(Matrix A) {
  // P_kp = AP_k-1 * A^T + Q_k
  Pkp = A.multiply(PCM).multiply(A.transpose());
  return Pkp
}

void Kalman::calcKalmanGain() {
  KG = Pkp.multiply(H.transpose()).divide(
    H.multiply(Pkp).multiply(H.transpose()));
}

void Kalman::calcNewMeasurement() {
  Matrix Ykm = accel->getAngularPosition();
  Yk = H.Multiply(Ykm);
}

void Kalman::calcCurrenState() {
  Xkp = X_kp.add(KG.multiply(Yk.subtract(H.multiply(X_kp))));
  currState = Xk;
}

void Kalman::updateFilter() {
  Matrix Pk = H.subtract(KG.multiply(H)).multiply(Pkp);
  X_k_1 = Xk;
  PCM = Pk;
}


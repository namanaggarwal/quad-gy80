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

  

}







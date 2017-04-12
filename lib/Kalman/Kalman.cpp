#include "Kalman.h"

Kalman::Kalman(T* sensor_data) {
  // filter needs to acquire data at every iteration in order
  // to properly begin and update. This function takes in
  // sensor data and packages it as a vector matrix. A Kalman filter
  // will take in varying dimensions of data.

  // the parameter in this function should point to a vector that supplies
  // input data
  sensor_source = sensor_data;
  // input data will determine the size of A, B, C, and various H's

  // size of the data is required to construct a Matrix of this data


  
  
  

}


Kalman::PredictState(int time) {
  // initial input is used to calculate the current state
  // the current predicted  state with:
  //         X_kp = AX_k-1 + Bu_k + w_k
  // We also calculate the process control matrix:
  //         P_k = AP_k-1 * A^T + Q_k
  //
  // These matrices are used to calculate the Kalman Gain (KG)
  //         K = (P_kp * H) / (H * P_kp * H^T + R)
  //         X_k = X_kp + K( Y - H * X_kp ) <- updated state
  //         * H is used to transform the values so the
  //         multiplication/division can work properly
  //
  // Current becomes the previous state and is outputted as
  // the currently predicted state
  //         P_k = (I - KH) * P_kh
  //         X_p
  //
  // Process repeats


  //state time is given by parameter
  time_cycle = time;


}

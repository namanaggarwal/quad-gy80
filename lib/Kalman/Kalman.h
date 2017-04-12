#include "Arduino.h"
#include "Matrix.h"

template <typename T> class Kalman
{
 public:
  Kalman(T *sensordata); //data can be of any type here
  Matrix PredictState();
 private:
  Matrix currState;
  Matrix prevState;
};

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

const Matrix Ident2 = [1, 0,
		       0, 1];

const Matrix Ident3 = [1, 0, 0,
		       0, 1, 0,
		       0, 0, 1];

const Matrix Ident4 = [1, 0, 0, 0,
		       0, 1, 0, 0,
		       0, 0, 1, 0,
		       0, 0, 0, 1];

const Matrix Ident5 = [1, 0, 0, 0, 0,
		       0, 1, 0, 0, 0,
		       0, 0, 1, 0, 0,
		       0, 0, 0, 1, 0,
		       0, 0, 0, 0, 1];

const Matrix Ident6 = [1, 0, 0, 0, 0, 0,
		       0, 1, 0, 0, 0, 0,
		       0, 0, 1, 0, 0, 0,
		       0, 0, 0, 1, 0, 0,
		       0, 0, 0, 0, 1, 0,
		       0, 0, 0, 0, 0, 1];

const Matrix Ident7 = [1, 0, 0, 0, 0, 0, 0,
		       0, 1, 0, 0, 0, 0, 0,
		       0, 0, 1, 0, 0, 0, 0,
		       0, 0, 0, 1, 0, 0, 0,
		       0, 0, 0, 0, 1, 0, 0,
		       0, 0, 0, 0, 0, 1, 0,
		       0, 0, 0, 0, 0, 0, 1];


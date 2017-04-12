#include "Arduino.h"
#include "Matrix.h"

template <typename T> class Kalman
{
 public:
  // regular functions
  Kalman(T *sensor_data); //data can be of any type here
  Matrix PredictState(int time);
 private:
  // helper functions
  Matrix calcStateMatrix();
  Matrix calcProcessControlMatrix();
  Matrix calcKalmanGain();
  Matrix calcNewMeasurement();
  Matrix getPrediction();
  Matrix setPrevState();
  Matrix setCurrentState();

  // variables required between states
  Matrix currState;
  Matrix prevState;

  // data source for filter
  T* sensor_source;

  // time between cycles (defaults to 1??)
  int time_cycle = 1;

  // Standard Identity Matrices
  const static Matrix Ident2 = Matrix(2,2,[1, 0, 0, 1]);

  const static Matrix Ident3 = Matrix(3,3,[1, 0, 0, 0, 1, 0, 0, 0, 1]);

  const static Matrix Ident4 =
    Matrix(4,4,[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);

  const static Matrix Ident5 = Matrix(5,5,[1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1]);

  const static Matrix Ident6 = Matrix(6,6,[1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1]);

  const static Matrix Ident7 = Matrix(7,7,[1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1]);


};






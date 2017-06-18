#ifndef KALMAN_H
#define KALMAN_H

//#include "Arduino.h"
#include "Sensor.h"

class Kalman
{
 public:
  // regular functions
  Kalman(Sensor* sen_a, Sensor* sen_b); 
  Matrix PredictState(int time);
  
 private:
  // helper functions
  void calcStateMatrix();
  void calcProcessControlMatrix();
  void calcKalmanGain();
  void calcNewMeasurement();
  void calcCurrentState();
  void updatePCM();
  
  
  void getPrediction();
  void setPrevState();
  void setCurrentState();

  // variables required between states
  double arrStandard61[6] = {0,0,0,0,0,0};
  double arrStandard66[36] = {0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0};
                            
  Matrix PCM = Matrix(6,6,arrStandard66);
  Matrix A = Matrix(6,6,arrStandard66);
  Matrix B = Matrix(6,3);
  Matrix X_kp = Matrix(6,1,arrStandard61);
  Matrix X_k_1 = Matrix(6,1,arrStandard61);
  Matrix Pkp = Matrix(6,6,arrStandard66);
  Matrix KG = Matrix(6,6,arrStandard66);
  Matrix Yk = Matrix(6,1, arrStandard61);
  Matrix Xk = Matrix(6,1, arrStandard61);
  Matrix H = Matrix(6,6, arrStandard66);
  Matrix R = Matrix(6,1, arrStandard66);
  Matrix currState = Matrix(6,1);
  // Process and Observational Error -- values inserted are test values
  double pep[3] = {2, 2, 2};
  double pev[3] = {2, 2, 2};
  double oep[3] = {2, 2, 2};
  double oev[3] = {2, 2, 2};
  
  Sensor* sensor_a;
  Sensor* sensor_b;
  

  // time between cycles (defaults to 1??)
  double time_cycle = 1;

  // Standard Identity Matrices

};
 #endif

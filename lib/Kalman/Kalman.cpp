#include "Kalman.h"


Kalman::Kalman(Sensor* sen_a, Sensor* sen_b) {
  // filter needs to acquire data at every iteration in order
  // to properly begin and update. This function takes in
  // sensor data and packages it as a vector matrix. A Kalman filter
  // will take in varying dimensions of data.
  
  sensor_a = sen_a;
  sensor_b = sen_b;
  

  // Generate Xkp
  double arrXkp[6] = {0,0,0,0,0,0};
  X_kp = Matrix(6, 1, arrXkp);
  
  // Generate A
  //A -> 6x6
  double arrA[36] = {1, 0, 0, time_cycle, 0, 0, 
                    0, 1, 0, 0, time_cycle, 0,  
                    0, 0, 1, 0, 0, time_cycle,  
                    0, 0, 0, 1, 0, 0,  
                    0, 0, 0, 0, 1, 0,  
                    0, 0, 0, 0, 0, 1 }; 
  A = Matrix(6,6,arrA);
  
  // Generate B
  // B -> 6x3
  double arrB[18] = {.5*time_cycle*time_cycle, 0, 0,
                     0, .5*time_cycle*time_cycle, 0,
                     0, 0, .5*time_cycle*time_cycle,
                     time_cycle, 0, 0,
                     0, time_cycle, 0, 
                     0, 0, time_cycle };
  B = Matrix(6,3, arrB);
  // Generate H
  double arrH[36] = {1, 0, 0, 0, 0, 0, 
                     0, 1, 0, 0, 0, 0,  
                     0, 0, 1, 0, 0, 0,  
                     0, 0, 0, 1, 0, 0,  
                     0, 0, 0, 0, 1, 0,  
                     0, 0, 0, 0, 0, 1 }; 
  H = Matrix(6,6,arrH);
  
  // Generate R
  double arrR[36] = {oep[0]*oep[0],0,0,0,0,0,
                     0,oep[1]*oep[1],0,0,0,0,
                     0,0,oep[2]*oep[2],0,0,0,
                     0,0,0,oev[0]*oev[0],0,0,
                     0,0,0,0,oev[1]*oev[1],0,
                     0,0,0,0,0,oev[2]*oev[2]};
  R = Matrix(6,6,arrR);
  
  // Generate PCM now
  double arrPCM[36] = {pep[0]*pep[0], pep[0]*pep[1], pep[0]*pep[2], pep[0]*pev[0], pep[0]*pev[1], pep[0]*pev[2],
                pep[1]*pep[0], pep[1]*pep[1], pep[1]*pep[2], pep[1]*pev[0], pep[1]*pev[1], pep[1]*pev[2],
                pep[2]*pep[0], pep[2]*pep[1], pep[2]*pep[2], pep[2]*pev[0], pep[2]*pev[1], pep[2]*pev[2],
                pev[0]*pep[0], pev[0]*pep[1], pev[0]*pep[2], pev[0]*pev[0], pev[0]*pev[1], pev[0]*pev[2], 
                pev[1]*pep[0], pev[1]*pep[1], pev[1]*pep[2], pev[1]*pev[0], pev[1]*pev[1], pev[1]*pev[2],
                pev[2]*pep[0], pev[2]*pep[1], pev[2]*pep[2], pev[2]*pev[0], pev[2]*pev[1], pev[2]*pev[2]};
  PCM = Matrix(6,6, arrPCM);
  
  // Generate First Step outside iteration loop
  //X_k_1 -> 6x1
  //double arrX_k_1[6];
  //sensor_a->Update(arrX_k_1, 6);
  Matrix X_k_1 = Matrix(6, 1) = sensor_a->Update();
  currState = X_k_1;
}

Matrix Kalman::PredictState(int time) {
  // initial input is used to calculate the current state
  // the current predicted  state with:
  //         X_kp = AX_k-1 + Bu_k + w_k
  calcStateMatrix();
  // We also calculate the process control matrix:
  //         P_k = AP_k-1 * A^T + Q_k
  calcProcessControlMatrix();
  // These matrices are used to calculate the Kalman Gain (KG)
  //         K = (P_kp * H) / (H * P_kp * H^T + R)
  //         X_k = X_kp + K( Y - H * X_kp ) <- updated state
  //         * H is used to transform the values so the
  //         multiplication/division can work properly
  calcKalmanGain();
  calcNewMeasurement();
  // Current becomes the previous state and is outputted as
  // the currently predicted state
  //         P_k = (I - KH) * P_kh
  //         X_p
  calcCurrentState();
  //state time is given by parameter
  time_cycle = time;
}

void Kalman::calcStateMatrix() {
  // X_kp = A*X_k_1 + B*u_k + w_k
  
  double arrU_k[3];
  sensor_b->Update(arrU_k, 3);              
  //U_k -> 3x1
  Matrix U_k = Matrix(3,1, arrU_k);
  
  //w_k -> 6x1 noise/error/uncertainty
  
  //X_kp -> 6x1 Matrix
  X_kp = A.multiply(X_k_1);
  X_kp = X_kp.add(B.multiply(U_k)); //add w_k later
  
}

void Kalman::calcProcessControlMatrix() {
    // P_kp = AP_k-1 * A^T + Q_k
    Pkp = A.multiply(PCM).multiply(A.transpose()); //add Q in later
}

void Kalman::calcKalmanGain() {
  //K = (P_kp * H) / (H * P_kp * H^T + R)
  KG = Pkp.multiply(H.transpose()).divide(
    H.multiply(Pkp).multiply(H.transpose())+R);
}

void Kalman::calcNewMeasurement() {
  //Y_k = C*Y_km + Zk
  //retrieve new measurement from sensor_a:
  double arrYkm[6];
  sensor_a->Update(arrYkm, 6);
  Matrix Ykm = Matrix(6, 1, arrYkm);
  // H is already the Identity Matrix
  Yk = H.multiply(Ykm); //add Zk later for noise
  
}

void Kalman::calcCurrentState() {
  //Xk = Xkp + KG[Yk - H*Xkp]
  Xk = X_kp.add(KG.multiply(Yk.subtract(H.multiply(X_kp))));
  currState = Xk;
}

void Kalman::updateFilter() {
  //Pk = (I - KH)*Pkp
  // H = I in this icase
  Pk = H.subtract(KG.multiply(H)).multiply(Pkp);
  X_k_1 = Xk;
  PCM = Pk;
}

Matrix Kalman::getCurrState() {
  return currState;
}




#include <Arduino.h>

#include "Navi.h"
#include "L3G4200D.h"
#include "ADXL345.h"
#include "Kalman.h"
#include "matrix.h"

// Navi myNavi;
// Matrix myAngle;

// void setup() {
//     Serial.begin(9600);
//     Serial.println("Starting");
//     myNavi.init();
//     delay(500);
// }

// void loop() { 
//     Serial.println("Updating State...");
//     myNavi.UpdateState();
//     // myAngle = myNavi.getState();
//     // Serial.println("HEY!: Your current angle:");
//     // myAngle.printMatrix();
//     while(1) {}
// }

ADXL345 accel;
L3G4200D gyro;

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
Matrix Q = Matrix(6,6);   // Covariance of process noise
Matrix R = Matrix (6,6);   // Covariance of the observation noise

float dt = 0;
float pt = 0;
int iter = 0;
void getdt() {
  // process our change in time
  unsigned long int ct = micros();
  dt = (ct - pt)/1000000;
  pt = ct/1000000;
  
}

void setup() {
    Serial.begin(9600);
    Serial.println("--- Starting ---");
    Wire.begin();
    accel.init(0,0,0);
    gyro.init(0,0,0);
    
    double H_arr[36] = {1, 0, 0, 0, 0, 0,
                       0, 1, 0, 0, 0, 0,
                       0, 0, 1, 0, 0, 0,
                       0, 0, 0, 1, 0, 0,
                       0, 0, 0, 0, 1, 0,
                       0, 0, 0, 0, 0, 1};
    H = Matrix(6,6, H_arr);    

    double Q_arr[36] = {.05f, 0, 0, 0, 0, 0,
                       0, .05f, 0, 0, 0, 0,
                       0, 0, .05f, 0, 0, 0,
                       0, 0, 0, .05f, 0, 0,
                       0, 0, 0, 0, .05f, 0,
                       0, 0, 0, 0, 0, .05f};
    Q = Matrix(6,6, Q_arr);

    double R_arr[36] = {.03f, 0, 0, 0, 0, 0,
                       0, .03f, 0, 0, 0, 0,
                       0, 0, .03f, 0, 0, 0,
                       0, 0, 0, .03f, 0, 0,
                       0, 0, 0, 0, .03f, 0,
                       0, 0, 0, 0, 0, .03f};
    R = Matrix(6,6, R_arr);

    gyro.Update();
    gyro.Update();
    gyro.Update();
    gyro.Update();

    accel.Update();
    accel.Update(); 
    accel.Update();
    accel.Update();


    Xk = gyro.getKalmanInput();    
}

void loop() {
    Serial.println("GO");
    
    getdt();
    double A_arr[36] = {1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0,
                        dt, 0, 0, 1, 0, 0,
                        0, dt, 0, 0, 1, 0,
                        0, 0, dt, 0, 0, 1};
    A = Matrix(6,6, A_arr);
    // Step 1. Calculate State Matrix
    Xk = A.multiply(Xk); //need to add B in
    //Xk.printMatrix();


    // Step 2. Calculate Process Control Matrix
    Pk = A.multiply(Pk).multiply(A.transpose()).add(Q);
    //Pk.printMatrix();

    // // Step 3. Calculate the Kalman Gain
    K = Pk.multiply(H.transpose()).divide(
        H.multiply(Pk).multiply(H.transpose()).add(R));
    
    //K.printMatrix();

    // // Step 4. Update the Observed State
    accel.Update();
    gyro.Update();
    Matrix observed = Matrix(6,1);
    observed(0) = gyro.getAngularVelocity()(0);
    observed(1) = gyro.getAngularVelocity()(1);
    observed(2) = gyro.getAngularVelocity()(2);
    observed(3) = accel.getAngularPosition()(0);
    observed(4) = accel.getAngularPosition()(1);
    observed(5) = accel.getAngularPosition()(2);

    // H.printMatrix();
    // observed.printMatrix();
    Yk = H.multiply(observed);

    //Yk.printMatrix();
    // // Step 5. Calculate the Current State:
    Xk = Xk.add(K.multiply(Yk.subtract(H.multiply(Xk))));
    //Xk.printMatrix();


    // // Step 6. Calculate the new Process Control Matrix
    Pk = H.subtract(K.multiply(H)).multiply(Pk);
    //Pk = Pk.subtract(K.multiply)
    //Pk.printMatrix();

    Serial.print("Pitch:\t");
    Serial.println(Xk(3));
    Serial.print("Roll:\t");
    Serial.println(Xk(4));
    Serial.print("Yaw\t");
    Serial.println(Xk(5));
    delay(400);


    if(iter == 5){
        while(1){}
    }
    iter++;
    
}
#include <Arduino.h>

#include "L3G4200D.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "matrix.h"


ADXL345 accel;
L3G4200D gyro;
HMC5883L compass;

Matrix Xk = Matrix(4,1);  // Current State
Matrix Pk = Matrix(4,4);  // Process Covariance Matrix
Matrix K = Matrix(4,4);   // Kalman Gain
Matrix Yk = Matrix(4,1);  // Observed State


Matrix A = Matrix(4,4);  // State Transition Model
Matrix H = Matrix(4,4);

// Noise covariance Matrices - User Tuned
Matrix Q = Matrix(4,4);   // Covariance of process noise
Matrix R = Matrix (4,4);   // Covariance of the observation noise

// Matrices for complimentary filter
Matrix comp_angles = Matrix(2,1);
double CFA_arr[4] = {.98f, 0,
                    0, .98f,};

Matrix CFA = Matrix(2,2, CFA_arr);

double CFB_arr[4] = {.02f, 0,
                    0, .02f};

Matrix CFB = Matrix(2,2, CFB_arr);


// Timer
unsigned long pt = 0;

// Testing
int iter = 0;


void setup() {
    Serial.begin(9600);
    Serial.println("--- Starting ---");
    Wire.begin();
    accel.init(0,0,0);
    gyro.init(0,0,0);
    compass.init();
    
    double H_arr[16] = {1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1};
    H = Matrix(4, 4, H_arr);    

    // theory: 1st/2nd rows are gyro bias, 3rd/4th are accel noise variance
    double Q_arr[16] = {.05f, 0, 0, 0,
                        0, .05f, 0, 0,
                        0, 0, .05f, 0,
                        0, 0, 0, .05f};
    Q = Matrix(4, 4, Q_arr);

    double R_arr[16] = {.03f, 0, 0, 0,
                        0, .03f, 0, 0,
                        0, 0, .03f, 0,
                        0, 0, 0, .03f};
    R = Matrix(4, 4, R_arr);

    accel.Update();
    gyro.Update();
    accel.Update();
    gyro.Update();
}

void loop() {
    Serial.print("Iteration ");
    Serial.println(iter);
    unsigned long ct = micros();
    float dt = (ct - pt);
    dt = dt/1000000;
    pt = ct;

    double A_arr[16] = {1, 0, 0, 0,
                        0, 1, 0, 0,
                        dt, 0, 1, 0,
                        0, dt, 0, 1};
    A = Matrix(4, 4, A_arr);

    // double B_arr[8] = {dt, 0,
    //                     0, dt,
    //                     .5*dt*dt, 0,
    //                     0, .5*dt*dt};
    // Matrix B = Matrix(4, 2, B_arr);
    // Step 1. Calculate the Predicted State Matrix 
    // 4x1 = (4x4 * 4x1)
    Serial.println("State Transition Matrix:");
    Xk = A.multiply(Xk); //.add(B.multiply(gyro.getAngularAcceleration())); 
    A.printMatrix();

    Serial.println("Current State:");
    Xk.printMatrix();
    // Step 2. Calculate the Predicted Process Control Matrix
    // 4x4 = (4x4 * 4x4) *(4x4 + 4x4)
    Serial.println("Process Covariance Matrix:");
    Pk = A.multiply(Pk).multiply(A.transpose()).add(Q);
    Pk.printMatrix();

    // Step 3. Calculate the Kalman Gain
    // 4x4 = (4x4*4x4)/((4x4 * 4x4) + 4x4)
    K = Pk.multiply(H.transpose()).divide(
         H.multiply(Pk).multiply(H.transpose()).add(R));
    Serial.println("Kalman Gain:");
    K.printMatrix();
    
    
    // Step 4. Update the Observed State
    
    accel.Update();
    gyro.Update();
    // compass.Update();

    

    Matrix observed = Matrix(4,1);
    observed(0) = gyro.getAngularVelocity()(0);
    observed(1) = gyro.getAngularVelocity()(1);
    observed(2) = accel.getAngularPosition()(0);
    observed(3) = accel.getAngularPosition()(1);
    Yk = H.multiply(observed);
    Serial.println("Observed State");
    Yk.printMatrix();
    Serial.println("Comparison:");
    // gyro.getAngularVelocity().printMatrix();
    // accel.getAngularPosition().printMatrix();
    // 4x1  = 4x4 * 4x1
    
    // try refactoring sensors
    // Step 5. Calculate the Current State:
    //4x1 = 4x1 + (4x4 * 4x1 - (4x4 * 4x1))
    Xk = Xk.add(K.multiply(Yk.subtract(H.multiply(Xk))));
    Serial.println("Updated State:");
    Xk.printMatrix();

    // Step 6. Calculate the new Process Control Matrix
    // 4x4 = 4x4 - (4x4*4x4*4x4)
    Pk = H.subtract(K.multiply(H)).multiply(Pk);
    Serial.println("Updated Process Covariance Matrix:");
    Pk.printMatrix();


    Serial.print("Kalman:\tPitch: ");
    Serial.print(Xk(2));
    Serial.print("\tRoll: ");
    Serial.println(Xk(3));

    // Serial.print("\tGyro:\tPitch: ");
    // Serial.print(gyro.getAngularPosition()(0));
    // Serial.print("\tRoll: ");
    // Serial.print(gyro.getAngularPosition()(1));

    // Serial.print("\tAccel:\tPitch: ");
    // Serial.print(accel.getAngularPosition()(0));   
    // Serial.print("\tRoll: ");
    // Serial.println(accel.getAngularPosition()(1)); 

    // Serial.print("\tCmpss:\tHdng: ");
    // Serial.println(compass.getHeading());
    // delay(200);

    // Testing
    if(iter == 1){
        while(1){}
    }
    iter++;
    
}
#include <Arduino.h>

#include "L3G4200D.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "Kalman.h"
#include "matrix.h"


ADXL345 accel;
L3G4200D gyro;
HMC5883L compass;

Matrix Xk = Matrix(4,1);  // Current State
Matrix Pk = Matrix(4,4);  // Process Covariance Matrix
Matrix K = Matrix(4,4);   // Kalman Gain
Matrix Yk = Matrix(4,1);  // Observed State

// User provided
Matrix A = Matrix(4, 4);  // State Transition Model
//Matrix B = Matrix(3,6);   // Control-Input Model
//Matrix C = Matrix(6,6);   // duplicate of H
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


unsigned long pt = 0;
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

    gyro.Update();
    gyro.Update();
    gyro.Update();
    gyro.Update();

    accel.Update();
    accel.Update(); 
    accel.Update();
    accel.Update();


    Xk = gyro.getKalmanInput4x1();    
}

void loop() {
    //Serial.println("---------------------------------------");
    
    unsigned long ct = micros();
    float dt = (ct - pt);
    dt = dt/1000000;
    //Serial.println(dt,4);
    pt = ct;

    double A_arr[16] = {1, 0, 0, 0,
                        0, 1, 0, 0,
                        dt, 0, 1, 0,
                        0, dt, 0, 1};
    A = Matrix(4, 4, A_arr);

    double B_arr[8] = {dt, 0,
                        0, dt,
                        0, 0, dt,
                        .5*dt*dt, 0,
                        0, .5*dt*dt};
    Matrix B = Matrix(4, 2, B_arr);
    // Step 1. Calculate the Predicted State Matrix 
    Xk = A.multiply(Xk).add(B.multiply(gyro.getAngularAcceleration2x1())); 
    //Xk.printMatrix();


    // Step 2. Calculate the Predicted Process Control Matrix
    Pk = A.multiply(Pk).multiply(A.transpose()).add(Q);
    //Pk.printMatrix();

    // // Step 3. Calculate the Kalman Gain
    K = Pk.multiply(H.transpose()).divide(
        H.multiply(Pk).multiply(H.transpose()).add(R));
    
    //K.printMatrix();

    // // // Step 4. Update the Observed State
    accel.Update();
    gyro.Update();
    compass.Update();

    Matrix observed = Matrix(4,1);
    observed(0) = gyro.getAngularVelocity()(0);
    observed(1) = gyro.getAngularVelocity()(1);
    observed(2) = accel.getAngularPosition()(0);
    observed(3) = accel.getAngularPosition()(1);
  

    // H.printMatrix();
    // observed.printMatrix();
    Yk = H.multiply(observed);

    //Yk.printMatrix();
    // // Step 5. Calculate the Current State:
    Xk = Xk.add(K.multiply(Yk.subtract(H.multiply(Xk))));
    //Xk.printMatrix();

    // Step 6. Calculate the new Process Control Matrix
    Pk = H.subtract(K.multiply(H)).multiply(Pk);
    //Pk = Pk.subtract(K.multiply)
    //Pk.printMatrix();

    // Complimentary Filter Comparison:
    double comp_dt_arr[4] = {dt, 0.0f,
                            0.0f, dt};
                            
    Matrix comp_dt = Matrix(2,2, comp_dt_arr);
    //Serial.print("Calculating Comp Filter");
    //gyro.getAngularVelocity().printMatrix();
    // Matrix dps = comp_dt.multiply(gyro.getAngularVelocity());

    // Matrix gps_angles = CFA.multiply(comp_angles.add(dps));//.add(CFB.multiply(accel.getAngularPosition()));
    // comp_angles = gps_angles.add(CFB.multiply(accel.getAngularPosition()));
    comp_angles = CFA.multiply(comp_angles.add(comp_dt.multiply(gyro.getAngularVelocity2x1()))).add(CFB.multiply(accel.getAngularPosition2x1()));



    Serial.print("Kalman:\tPitch: ");
    Serial.print(Xk(0));
    Serial.print("\tRoll: ");
    Serial.print(Xk(1));


    Serial.print("\tCompli:\tPitch: ");
    Serial.print(comp_angles(0));
    Serial.print("\tRoll: ");
    Serial.print(comp_angles(1));


    Serial.print("\tGyro:\tPitch: ");
    Serial.print(gyro.getAngularPosition()(0));
    Serial.print("\tRoll: ");
    Serial.print(gyro.getAngularPosition()(1));


    Serial.print("\tAccel:\tPitch: ");
    Serial.print(accel.getAngularPosition()(0));   
    Serial.print("\tRoll: ");
    Serial.print(accel.getAngularPosition()(1)); 


    Serial.print("\tCmpss:\tHdng: ");
    Serial.print(compass.getHeading());
    delay(200);




    // // Testing
    // if(iter == 5){
    //     while(1){}
    // }
    // iter++;
    
}
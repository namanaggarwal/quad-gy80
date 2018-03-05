#include "Arduino.h"
#include "matrix.h"

class PID {
    public:
        PID();
        Update();
    
    private:
    
        Matrix* input;
        Matrix* output;
        Matrix* setpoint;
    
        // Tuning Parameters
        Matrix Kp = Matrix(1,1);   // Proportional Gain
        Matrix Ki = Matrix(1,1);   // Integral Gain
        Matrix Kd = Matrix(1,1);   // Derivative Gain
        
        Matrix error = Matrix(1,6);
        Matrix prev_error = Matrix(1,6);
        Matrix integral = Matrix(1,6);
        Matrix derivative = Matrix(1,6);
        
        // Timer Variables
        float dt;
        float pt;
        
        
}
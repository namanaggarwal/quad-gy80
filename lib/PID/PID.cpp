#include "PID.h"

PID::PID(Matrix* Input, Matrix* Output, Matrix* SetPoint, Kp, Ki, Kd, Direction) {
    input = Input;
    output = Output;
    setpoint = SetPoint;
}


PID::Update() {
    // adapted from https://en.wikipedia.org/wiki/PID_controller#Pseudocode
    error = *setpoint - *input;
    integral+=integral+error*dt;
    derivative = (error - prev_error)/dt;
    output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
}

PID::setKp(float newKp) {
    Kp(0) = newKp;
}

PID::setKi(float newKi) {
    Ki(0) = newKi;
}

PID::setKd(float newKd) {
    Kd(0) = newKd;
}
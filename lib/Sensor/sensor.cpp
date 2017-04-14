#include "sensor.h"

void Sensor::Update(double* target, int size){
  //return double* must be in form [Xx, Xy, Xz, Vx, Vy, Vz]
    for(int i = 0; i < size; i++) {
      target[i] = 1;
    }
  }

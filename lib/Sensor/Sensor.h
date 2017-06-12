#ifndef SENSOR_H
#define SENSOR_H

#include "matrix.h"
//sensor is parent class of other sensors

class Sensor {
  public:
    virtual Matrix Update();

};

#endif

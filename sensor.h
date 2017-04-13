#ifndef SENSOR_H
#define SENSOR_H

//sensor is parent class of other sensors

class Sensor {
  public:
    virtual void Update(double* target, int size);

};

#endif

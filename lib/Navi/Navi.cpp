#include "Navi.h"


Navi::Navi();
{
  accel.init();
  gryo.init();
  compass.init();
  alti.init();
}

void Navi::init() {
  Serial.println("Navi initializing...");
  // start Kalman filtering
  
}

void Navi::UpdateState() {
  // logic for updating state here
  //
  // get next state based on previous state

}


  


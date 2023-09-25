#include "Arduino.h"
#include "conversion.h"
#include "updt.h"

float conversion = 10 ^ (-7);  // [N/mm^3]
float alpha = 1; // learning rate
float eta = 1; // nudge factor
int tasktype = 0; //0 for motion tasks, 1 for force tasks (curretly does nothing)


float updt::updaterule_binary(float free, float clamp) {
  float dpow = (pow((clamp), 2) - pow((free), 2));
  int sgn = 0;
  if (dpow>0){
    sgn = 1;
  }
  else if(dpow<0){
    sgn = -1;
  }
  float dLayer = 0.5*sgn;
  return dLayer;
}


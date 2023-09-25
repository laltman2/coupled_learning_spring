#ifndef updt_h
#define updt_h

#include <Arduino.h>

class updt{
  public:
  float updaterule_binary(float free, float clamp);

  private:
  float conversion = 10 ^ (-7);  // [N/mm^3]
  float alpha = 1; // learning rate
  float eta = 1; // nudge factor
  int tasktype = 0; //0 for motion tasks, 1 for force tasks (curretly does nothing)

};

#endif
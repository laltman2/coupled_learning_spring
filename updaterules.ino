// Updating ---------------------------------
float conversion = 10 ^ (-7);  // [N/mm^3]
float alpha = 1; // learning rate
float eta = 1; // nudge factor
int tasktype = 0; //0 for motion tasks, 1 for force tasks (curretly does nothing)

// coupled learning update rules
// float updaterule_motion(float free, float clamp){

//   return deltaK;
// }

// float updaterule_force(float free, float clamp){

//   return deltaK;
// }

float updaterule(float free, float clamp) {
  float dpow = (pow((clamp), 2) - pow((free), 2));
  float deltaK = -alpha * eta / 2. * conversion * dpow;
  float dLayer = stif2layer(deltaK);
  return dLayer;
}

float updaterule_binary(float free, float clamp) {
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

void update(float goalLayer) {
  setpoint = lookup_table(goalLayer);
}
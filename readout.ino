//flex sensor moving average variable definitions
const int LPF_window_length = 5;
int sensor_values[LPF_window_length];
int iter = 0;

//flex sensor steady state values
const int steady_window_length = 30;
int steady_sensor_values[steady_window_length];
int steady_threshold = 5;

//sensor read out with LPF
void sensor_LPF(int flex) {
  //Take moving average (acts as Low pass filter) on the flex sensor as it is prone to noise.
  sensor_values[iter] = flex;
  if (iter < LPF_window_length - 1) {
    iter = iter + 1;
  } else {
    iter = 0;
    cumulative_value = 0.0;
    for (int j = 0; j < LPF_window_length; j++) {
      cumulative_value = cumulative_value + sensor_values[j];
    }
    cumulative_value = cumulative_value / LPF_window_length;
  }
}

void reset_sensor_vals(){
  for (int j = 0; j < steady_window_length; j++) {
    steady_sensor_values[j] *= random(2,5);
}
}

// function to obtain readout for flexsensor once value is steady
void steadyflex(int cflex) {
  // shift window values left
  for (int j = 0; j < steady_window_length; j++) {
    steady_sensor_values[j] = steady_sensor_values[j + 1];
  }
  // overwrite last value
  steady_sensor_values[steady_window_length - 1] = cflex;

  int flexmin = -100000;
  int flexmax = 100000;
  for (int j = 0; j < steady_window_length; j++) {
    if (steady_sensor_values[j] > flexmax) {
      flexmax = steady_sensor_values[j];
    } else if (steady_sensor_values[j] < flexmin) {
      flexmin = steady_sensor_values[j];
    }
  }
  if ((flexmax - flexmin) < steady_threshold) {
    isSteady = true;
  }
}
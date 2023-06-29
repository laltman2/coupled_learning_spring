// Plotting -------------------------------------

//sensor read out
void sensor_plot(int cflex) {
  Serial.print("d_min:");
  Serial.print(-30);
  Serial.print(",");
  Serial.print("d_max:");
  Serial.print(30);
  Serial.print(",");
  Serial.print("disp:");
  Serial.println(flex2disp(cumulative_value));
  // Serial.print("f_min:");
  // Serial.print(200);
  // Serial.print(",");
  // Serial.print("f_max:");
  // Serial.print(800);
  // Serial.print(",");
  // Serial.print("f_eq:");
  // Serial.print(480);
  // Serial.print(",");
  // Serial.print("flex:");
  // Serial.println(cumulative_value);
}

void layer_plot(int encpos, int setpoint) {
  const int arrLen = sizeof(table) / sizeof(table[0]);
  // for (int tableindex = 0; tableindex < arrLen; tableindex++){
  // }
  Serial.print("encpos_min:");
  Serial.print(table[0]);
  Serial.print(",");
  Serial.print("encpos_max:");
  Serial.print(table[arrLen - 1]);
  Serial.print(",");
  Serial.print("encoder_position:");
  Serial.print(encpos);
  Serial.print(",");
  Serial.print("set_point:");
  Serial.println(setpoint);
}
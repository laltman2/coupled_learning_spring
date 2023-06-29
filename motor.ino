//functions to read from encoder square wave and increment/decrement encoder count
void doEncoderA() {
  PastB ? encPos++ : encPos--;  // if pastB then add 1 to encpos else minus 1 to encops
}
void doEncoderB() {
  PastB = !PastB;  //flip pastb
}

// function to set min thresholds for PWM value
double output_limits(double Output) {
  if (Output > 0 && Output < 25) Output = 25;
  if (Output < 0 && Output > -25) Output = -25;
  return Output;
}
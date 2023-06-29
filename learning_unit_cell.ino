#include <PID_v1.h>
// #include "conversions.ino"

//Pin connections
#define flexPin A3
#define encPinA A0
#define encPinB A1
#define pwmPin 10
#define dirPin 12

// learning mode and hyperparams
int learning_mode;

// plotting mode (0: none, 1:flex sensor, or 2:encoder position)
int plot_mode;

// physical and learning DOF
float free_disp;
float clamp_disp;

// check if flex reading is steady
bool isSteady = false;
volatile boolean measured;  // tells whether a measurement has been taken or not
bool updated = false;

// how much does the encoder position move for every layer? (approx. subject to change)
int layer_increment = 350;

// flex sensor read value after LPF
int cumulative_value = 0;

//encoder (and motor?) variable definitions
double input, output, setpoint;
volatile int encPos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
volatile boolean CurrentA = 0;
volatile boolean CurrentB = 0;
float currentLayer = 0;
float goalLayer = 0;
int initFlag;
float k_p = 0.5;
float k_d = 0;
float k_i = 0;

//initialization of PID function for motor control
PID myPID(&input, &output, &setpoint, 0.4, 0, 0, DIRECT);

//Setup and Main Loop -------------------------------

void setup() {
  //sets the speed of communication to 9600 bauds
  Serial.begin(9600);

  //setup PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-250, 250);

  //pin definitions
  pinMode(flexPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encPinA, INPUT);
  pinMode(encPinB, INPUT);

  //initialize encoder bits
  PastA = (boolean)digitalRead(encPinA);
  // PastB = (boolean)digitalRead(encPinB);

  //declare interrupt service routines
  // attachInterrupt(digitalPinToInterrupt(encPinA), doEncoderA, RISING);
  // attachInterrupt(digitalPinToInterrupt(encPinB), doEncoderB, CHANGE);

  learning_mode = 0;
  plot_mode = 0;
}

void loop() {

  if (Serial.available() > 0) {  //reading the data from Serial port
    initFlag = Serial.read();
    if (initFlag == 77) {  //key press "M" to change the learning mode
      // mode = 0 is just readout, no updates and no saving flex values
      // mode = 1 is free state (store flex in memory)
      // mode = 2 is clamped state (store flex in memory)
      // mode = 3 is update mode (adjust number of layers)
      learning_mode++;
      learning_mode = learning_mode % 4;
      measured = false;
      updated = false;
      reset_sensor_vals();
    }

    else if (initFlag == 73) {  // key press "I" to set initial layers like so: I 1 -> sets initial layers to 1
      Serial.println("set initial layers!");
      float initLayer = Serial.parseFloat(SKIP_ALL);
      setpoint = lookup_table(initLayer);
      encPos = setpoint;
    }

    else if (initFlag == 65)  //key press "A" to set goal layers like so: A 2.5 - > sets goal layers to 2.5
    {
      Serial.println("set goal layers!");
      goalLayer = Serial.parseFloat(SKIP_ALL);
      setpoint = lookup_table(goalLayer);
    } 
    else if (initFlag == 80)  //key press "P" to change plot mode like so: P 1 -> sets flex plot mode
    // always turn off plotting before setting a new layer! It will mess up the encoder reading
    {
      plot_mode = Serial.parseFloat(SKIP_ALL);
    }
  }

  CurrentA = (boolean)digitalRead(encPinA);
  CurrentB = (boolean)digitalRead(encPinB);

  if (CurrentA != PastA && CurrentA == 1){
    if (CurrentA != CurrentB) {
      encPos ++;
    }
    else {
      encPos --;
    }
    Serial.print("setpoint:");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print("encPos:");
  Serial.println(encPos);
  }
  PastA = CurrentA;

  // Read the flex pin sensor, values vary between 0-1023
  int ADCflex = analogRead(flexPin);

  //read current encoder value
  input = encPos;

  //implement PID control (only proportional control is being used)
  if (abs(encPos - setpoint) < 10) {
    myPID.SetTunings(0, 0, 0);  //0.5
  } else {
    myPID.SetTunings(k_p, k_d, k_i);  //0.6
  }

  myPID.Compute();

  output = output_limits(output);

  //choose motor direction : clockwise/anticlockwise
  if (output > 0) {
    digitalWrite(dirPin, 0);
  } else {
    digitalWrite(dirPin, 1);
  }
  if (abs(encPos - setpoint) < 2) {
    output = 0;
  }
  analogWrite(pwmPin, abs(output));

  sensor_LPF(ADCflex);
  if (learning_mode == 1) {
    // free state
    if (!measured) {
      plot_mode = 1;
      isSteady = false;
      steadyflex(cumulative_value);
      if (isSteady) {
        int freeflex = ADCflex;
        free_disp = flex2disp(freeflex);
        Serial.print("Free Displacement [mm]");
        Serial.println(free_disp);
        measured = true;
        plot_mode = 0;
      }
    }
  } else if (learning_mode == 2) {
    // clamp state
    if (!measured) {
      plot_mode = 1;
      isSteady = false;
      // reset sensor values so it doesn't stop immediately
      steadyflex(cumulative_value);
      if (isSteady) {
        int clampflex = cumulative_value;
        clamp_disp = flex2disp(clampflex);
        Serial.print("Clamped Displacement [mm]");
        Serial.println(clamp_disp);
        measured = true;
        plot_mode = 0;
      }
    }
  } else if (learning_mode == 3) {
    // update state
    if (!updated){
      plot_mode = 0;
      float dLayer = updaterule_binary(free_disp, clamp_disp);
      setpoint = dLayer*layer_increment + input;
      updated = true;
    }
  }

  if (plot_mode == 1) {
    sensor_plot(cumulative_value);
  }
  else if (plot_mode == 2) {
    layer_plot(input, setpoint);
  }
}
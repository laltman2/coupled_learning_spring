// #include head files
#include <PID_v1.h>
#include <filter.h>
#include <conversion.h>
#include <updt.h>
filter myfilter;
conversion myconversion;
updt myupdt;

//define pin connections
#define flexPin A0
#define encPinA 3
#define encPinB 4
#define pwmPin 11
#define dirPin 12

// learning mode and hyperparams (0:none, 1:free state, 2:clamped state, 3:update state)
int learning_mode; 
// plotting mode (0:none, 1:flex sensor, 2:encoder position)
int plot_mode; 

// physical and learning DOF
float free_disp;
float clamp_disp;

// flex sensor read value after LPF
unsigned int cumulative_value;

// check if flex reading is steady
bool isSteady = false;
// check whether a measurement has been taken or not
volatile boolean measured;
bool updated = false;

// how much does the encoder position move for every layer? (approx. subject to change)
int layer_increment = 570;

//define variables
double input, output, setpoint;
volatile int encPos = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
volatile boolean CurrentA = 0;
volatile boolean CurrentB = 0;
float currentLayer = 0;
float goalLayer = 0;
int initFlag;
float k_p = 3.25;
float k_i = 0;
float k_d = 0;

//initialization of PID function for motor control
PID myPID(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);

//lookup table to relate encoder counts to number of layers (including half layers)
int table[] = {0, 557, 1143, 1713, 2287, 2853, 3428, 4001, 4558, 5129, 5685, 6240, 6812, 7380, 7934, 8508, 9037, 9577, 10088};
//function which reads lookup table
int lookup_table(float layers){
  int index = layers * 2 - 2;
  if (index < 0){
    Serial.println("Invalid layer input");
  }
  return table[index];
}


//functions to read from encoder square wave and increment/decrement encoder count 
void doEncoderA(){
  PastB ? encPos++ :  encPos--; // if pastB then add 1 to encpos else minus 1 to encops
}
void doEncoderB(){
  PastB = !PastB; //flip pastb
}

int PIDout = 10;
//function to set min thresholds for PWM value
double output_limits(double Output){
  if (Output > 0 && Output < PIDout) Output = PIDout;
  if (Output < 0 && Output > -1*PIDout) Output = -1*PIDout;
  return Output;
}


//function to plot sensor read out
void sensor_plot(int cflex) {
  Serial.print("d_min:");
  Serial.print(-30);
  Serial.print(",");
  Serial.print("d_max:");
  Serial.print(30);
  Serial.print(",");
  Serial.print("disp:");
  // Serial.println(cflex);
  // Serial.println(cflex);
  Serial.println(myconversion.flex2disp(cflex));
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


//Setup and Main Loop -------------------------------

void setup() {
  //setup serial communication
  Serial.begin(9600);

  //setup PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-250, 250);

  //setup pin modes
  pinMode(flexPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encPinA, INPUT);
  pinMode(encPinB, INPUT);

  //initialize encoder bits
  PastA = (boolean)digitalRead(encPinA);
  PastB = (boolean)digitalRead(encPinB);

  //declare interrupt service routines
  attachInterrupt(digitalPinToInterrupt(encPinA), doEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encPinB), doEncoderB, CHANGE);

  learning_mode = 0;
  plot_mode = 0;
}


void loop() {

  if (Serial.available() > 0)  {    //reading the data from Serial port
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
      myfilter.reset_sensor_vals();
    }

    else if (initFlag == 73){ // key press "I" to set initial layers like so: I 1 -> sets initial layers to 1
        Serial.println("set initial layers!");
        float initLayer  = Serial.parseFloat(SKIP_ALL);
        setpoint = lookup_table(initLayer);
        encPos = setpoint;   
    }
        
    else if(initFlag == 65) {//key press "A" to set goal layers like so: A 2.5 - > sets goal layers to 2.5 
        Serial.println("set goal layers!");
        goalLayer = Serial.parseFloat(SKIP_ALL);
        setpoint = lookup_table(goalLayer);
    } 

    else if (initFlag == 80) { //key press "P" to change plot mode like so: P 1 -> sets flex plot mode    
      // always turn off plotting before setting a new layer! It will mess up the encoder reading
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
  // Serial.print(0);
  // Serial.print(" ");
  // Serial.print(750);
  // Serial.print(" ");
  // Serial.println(ADCflex);
  //read current encoder value
  input = encPos;

  //implement PID control (only proportional control is being used)
  if (abs(encPos - setpoint) < 10){
    myPID.SetTunings(0, 0, 0); 
  }
  else{
    myPID.SetTunings(k_p, k_d, k_i); 
  }
  myPID.Compute();
  output = output_limits(output);

  //choose motor direction : clockwise/anticlockwise
  if (output > 0)
    {digitalWrite(dirPin,0);
  }
  else 
    {digitalWrite(dirPin,1);
  }
  if (abs(encPos-setpoint) < 2 ){
    output = 0;
  }
  analogWrite(pwmPin,abs(output));


  cumulative_value = myfilter.sensor_LPF(ADCflex, cumulative_value);
  if (learning_mode == 1) { // free state
    if (!measured) {
      plot_mode = 1;
      isSteady = false;
      myfilter.steadyflex(cumulative_value);
      if (isSteady) {
        int freeflex = ADCflex;
        free_disp = myconversion.flex2disp(freeflex);
        Serial.print("Free Displacement [mm]");
        Serial.println(free_disp);
        measured = true;
        plot_mode = 0;
      }
    }
  } 
  else if (learning_mode == 2) { // clamp state
    if (!measured) {
      plot_mode = 1;
      isSteady = false;
      // reset sensor values so it doesn't stop immediately
      myfilter.steadyflex(cumulative_value);
      if (isSteady) {
        int clampflex = cumulative_value;
        clamp_disp = myconversion.flex2disp(clampflex);
        Serial.print("Clamped Displacement [mm]");
        Serial.println(clamp_disp);
        measured = true;
        plot_mode = 0;
      }
    }
  } 
  else if (learning_mode == 3) { // update state
    if (!updated){
      plot_mode = 0;
      float dLayer = myupdt.updaterule_binary(free_disp, clamp_disp);
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
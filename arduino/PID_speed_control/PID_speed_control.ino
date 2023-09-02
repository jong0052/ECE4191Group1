#include <util/atomic.h>
////////////////////////////////////
// Establish the PID control class//
////////////////////////////////////
class PID_control{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral;
  public:
  // Establish the C constructor(initialise KP = 1)
  PID_control(): kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}
  // Set function
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
};

// Ignore this Yide, I promise it is for the good of society
void parseStringToFloats(String input, float &wl_current, float &wr_current, float &wl_goal, float &wr_goal) {
  int index = input.indexOf("[") + 1; // Find the starting index of the values
  int endIndex = input.indexOf("]"); // Find the ending index of the values

  String valuesStr = input.substring(index, endIndex); // Extract the values between '[' and ']'
  valuesStr.replace(" ", ""); // Remove any spaces
  
  int commaIndex = valuesStr.indexOf(","); // Find the index of the first comma

  wl_current = valuesStr.substring(0, commaIndex).toFloat(); // Convert the substring to a float and assign to variable
  valuesStr = valuesStr.substring(commaIndex + 1); // Remove the parsed value and comma
  
  commaIndex = valuesStr.indexOf(",");
  wr_current = valuesStr.substring(0, commaIndex).toFloat();
  valuesStr = valuesStr.substring(commaIndex + 1);
  
  commaIndex = valuesStr.indexOf(",");
  wl_goal = valuesStr.substring(0, commaIndex).toFloat();
  valuesStr = valuesStr.substring(commaIndex + 1);
  
  wr_goal = valuesStr.toFloat(); // The remaining value is the last one
}

////////////////////////////////////
// Define the Global Variables//////
////////////////////////////////////

// Define the number of motors
#define NUM_MOTORS 2
// Define the number of counts of encoder for a full revolution.
#define ENC_COUNT_REV 894 // For now we assume two motors are similar this may come to bite me in the future

// Define the pin ins and outs for 
const int enca[]={2, 3}; // Define the input pin for Encoder A
const int encb[]={6, 7}; // Define the input pin for Encoder B
const int pwm[]={10, 11};// Define the output pin for PWM
const int in[]={4, 5}; // Define the direction pin for Motor Driver

// Global Variables

// Velocity calculation variables
long prevT = 0; // Define the time constants
volatile int posPrev[] = {0,0}; // Define the prvious position of two motors
float v1[] ={0, 0}; // Define the previous RPM velocity
float v2[] ={0, 0}; // Define the current RPM velocity
  
// Low Pass Filter variable
volatile float v1Filt[] = {0, 0}; 
volatile float v1Prev[] = {0, 0};
volatile float v2Filt[] = {0, 0};
volatile float v2Prev[] = {0, 0};

// Interrupt function variables 
volatile long prevT_i[] = {0,0};
volatile int pos_i[] = {0,0};
volatile float velocity_i[] = {0,0};
volatile float deltaT_int[] = {0, 0};
volatile int increment[] = {0,0};
//Define PID_control class objects
PID_control pid[NUM_MOTORS];

// Serial Data
float wl_current = 0;
float wr_current = 0;
float wl_goal = 0;
float wr_goal = 0;

////////////////////////////////////
// Define the Set up LOOP///////////
////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // For two motors set up the pin output and input
  for(int k = 0; k < NUM_MOTORS; k++){
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in[k], OUTPUT);

    pid[k].setParams(10, 0, 10, 255);
  }
  
  // Trigger an interrupt when encoder A rises
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder2, RISING);
  
}

void loop() {
  //////////////////////////////////////////////////////
  //*************While Uninterrupted******************//
  //////////////////////////////////////////////////////
  
  // Read Serial Data!
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseStringToFloats(data, wl_current, wr_current, wl_goal, wr_goal);
    wl_current = v1Filt[1];
    wr_current = v1Filt[0];
    if (data.startsWith("Pi")){
      // Check for the serial ready to read 
      if(Serial.availableForWrite()>0){
        Serial.print("Nano: [");
        Serial.print(wl_current);
        Serial.print(", ");
        Serial.print(wr_current);
        Serial.print(", ");
        Serial.print(wl_goal);
        Serial.print(", ");
        Serial.print(wr_goal);
        Serial.println("]");
        }
   }
    }

  //----------------- 1. Define the target velocity-----------------------
  float vt[2]; 
  // 0: target velocity of the right motor (assume ball bearing at the front).
  // 1: target velocity of the left motor (assume ball bearing at the front). 
  vt[0] = wr_goal; // Right WHEELS?
  vt[1] = wl_goal;//-10*sin(prevT/1e6); Left WHEELS????
  
  //------------------ 2. Initialise position and make sure position updates----------------------
  int pos[] = {0, 0};// The current position of the motor recorded by encoder.
  float velocity2[] = {0, 0};//The current velocity of the motor recorded by encoder.
  // Atomic Block will block other code from running when executing. Therefore ensuring memeory integrity
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    // Note here "pos_i" and "velocity_i" are both code chunk from the interrrupt callback.
    pos[0] = pos_i[0];
    velocity2[0] = velocity_i[0];
    pos[1] = pos_i[1];
    velocity2[1] = velocity_i[1];
  }
  
  //------------------ 3. Update the velocity----------------------
  // Compute velocity with method 1 (fixed time interval)
  // Compute the change in speed 
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/1.0e6; // Change in T
  
  float velocity1[2]; // Define the current velocity
  // Velocity for Motor 1
  velocity1[0] = (pos[0] - posPrev[0])/deltaT; // Compute the difference between the current and previous encoder count.
  posPrev[0] = pos[0];
  // Velocity for Motor 2
  velocity1[1] = (pos[1] - posPrev[1])/deltaT; // Compute the difference between the current and previous encoder count.
  posPrev[1] = pos[1];
  
  prevT = currT;

  //------------------ 4. Convert counts/s to RPM ----------------------------
  v1[0] = (velocity1[0]/ENC_COUNT_REV)*60;
  v2[0] = (velocity2[0]/ENC_COUNT_REV)*60;

  v1[1] = (velocity1[1]/ENC_COUNT_REV)*60;
  v2[1] = (velocity2[1]/ENC_COUNT_REV)*60;
  
  // Low-pass Filter (40 Hz Cutoff Pretty Good) <- Like Really need to change
  
  v1Filt[0] = 0.843*v1Filt[0] + 0.0782*v1[0] + 0.0782*v1Prev[0];
  v1Prev[0] = v1[0]; 
  v2Filt[0] = 0.843*v2Filt[0] + 0.0782*v2[0] + 0.0782*v2Prev[0];
  v2Prev[0] = v2[0];

  v1Filt[1] = 0.843*v1Filt[1] + 0.0782*v1[1] + 0.0782*v1Prev[1];
  v1Prev[1] = v1[1]; 
  v2Filt[1] = 0.843*v2Filt[1] + 0.0782*v2[1] + 0.0782*v2Prev[1];
  v2Prev[1] = v2[1];
  
  //------------------ 5. Implement PID control ----------------------------
  for (int k = 0; k < NUM_MOTORS; k++){
    int pwr, dir;
    pid[k].evalu(v1Filt[k], vt[k], deltaT, pwr, dir);
    setMotor(dir, pwr, pwm[k], in[k]);
  }

  
   /*
  Serial.print(vt[0]);
  Serial.print(" ");
  Serial.print(v1Filt[0]);
  Serial.print(" ");
  Serial.print(increment[0]);
  Serial.println();
  */
  
  delay(1); // Ensure consistent sampling frequency
  
  
}

// Set the power and direction of the Motor
void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm, pwmVal);
  if (dir == 1){
    digitalWrite(in1, HIGH);  
  }
  else{
    digitalWrite(in1, LOW);
  }
}
// Read Encoder Module for Motor 1
void readEncoder(){
  int b = digitalRead(encb[0]);
  increment[0] = 0;
  if(b>0){
    increment[0] = 1;
  }
  else {
    increment[0] = -1;
  }
  pos_i[0] = pos_i[0] + increment[0];
  long currT1 = micros(); // Micros returns the number of microseconds since the board has begin running the program
  deltaT_int[0] = ((float) (currT1 - prevT_i[0]))/1.0e6; // Detects the time passed between the current signal being detected and the next signal convert the time into seconds. 
  velocity_i[0] = increment[0]/deltaT_int[0];
  prevT_i[0] = currT1;
}

// Read Encoder Module for Motor 2
void readEncoder2(){
  int b2 = digitalRead(encb[1]);
  increment[1] = 0;
  if(b2>0){
    increment[1] = -1;
  }
  else {
    increment[1] = 1;
  }
  pos_i[1] = pos_i[1] + increment[1];
  long currT2 = micros(); // Micros returns the number of microseconds since the board has begin running the program
  deltaT_int[1] = ((float) (currT2 - prevT_i[1]))/1.0e6; // Detects the time passed between the current signal being detected and the next signal convert the time into seconds. 
  velocity_i[1] = increment[1]/deltaT_int[1];
  prevT_i[1] = currT2;
}
